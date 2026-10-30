"""LLM layer: natural language -> PROPOSED mission JSON string.

Hard contract of this layer:
  * Returns a raw string only. No parsing into trusted objects, no validation,
    no execution, no vehicle I/O, no imports from executor/ or sim_bridge/.
  * Everything returned here is UNTRUSTED until validator.validate_mission()
    accepts it.

Backends:
  * HuggingFaceBackend — Hugging Face Inference Providers router
    (OpenAI-compatible chat completions over HTTPS, stdlib urllib, no SDK).
    Token: https://hf.co/settings/tokens -> fine-grained token with the
    "Make calls to Inference Providers" permission, exported as HF_TOKEN.
    Free monthly inference credits exist on free accounts. Model defaults to
    "openai/gpt-oss-120b" with an automatic fallback chain (override with
    HF_MODEL — provider/policy suffixes like ":cheapest" are allowed).
  * OfflineHeuristicBackend — deterministic keyword parser producing the same
    schema-shaped JSON. Lets the whole pipeline (and CI) run with zero network
    and zero keys, and doubles as an honest demonstration that the validator —
    not the language model — is the safety boundary.

  * GroqBackend - Groq's OpenAI-compatible API (free tier serves the
    gpt-oss models). Key: https://console.groq.com/keys exported as
    GROQ_API_KEY. Model defaults to "openai/gpt-oss-120b" with
    "openai/gpt-oss-20b" as fallback (override with GROQ_MODEL).

"auto" uses Groq when GROQ_API_KEY is set, else Hugging Face when HF_TOKEN is
set, else the offline parser.
"""
from __future__ import annotations

import json
import os
import re
import sys
import time
import urllib.error
import urllib.request
from typing import List, Optional

from .prompts import (STEP_SYSTEM_PROMPT, SYSTEM_PROMPT, build_step_user_prompt,
                      build_user_prompt)

HF_CHAT_URL = "https://router.huggingface.co/v1/chat/completions"
HF_MODELS_URL = "https://router.huggingface.co/v1/models"
HF_TOKEN_ENVS = ("HF_TOKEN", "HUGGINGFACE_API_KEY", "HF_API_TOKEN")
# Measured on this task (NL -> schema-constrained mission JSON, ~1.4k-token
# system prompt), 2026-07: gpt-oss-20b returned a valid plan 4/4 in ~3.9 s.
# gpt-oss-120b, Llama-3.3-70B and Qwen2.5-7B all answer trivial prompts but are
# refused (HTTP 403, Cloudflare at api.together.xyz) once the full schema prompt
# is attached, so the smaller model is both faster AND more reliable here.
# Model IDs MUST include the org prefix (e.g. "Qwen/..."), or the router replies
# "model does not exist". gpt-oss-20b is kept as the first fallback because, on
# this network, it routed to a provider that was NOT blocked when the bigger
# models got HTTP 403 (Together / Groq / Cerebras).
DEFAULT_HF_MODEL = "Qwen/Qwen2.5-72B-Instruct"
# Walked in order when the default is unavailable. NOT used when the user pins a
# model explicitly (HF_MODEL / ctor).
HF_FALLBACK_MODELS = (
    "openai/gpt-oss-20b",
    "meta-llama/Llama-3.1-8B-Instruct",
)
# Retried in order when a model returns empty content with finish_reason=length.
# A survey grid is ~40 waypoints of JSON, so the ceiling has to be generous.
TOKEN_BUDGETS = (6000, 16000)
GROQ_CHAT_URL = "https://api.groq.com/openai/v1/chat/completions"
GROQ_MODELS_URL = "https://api.groq.com/openai/v1/models"
DEFAULT_GROQ_MODEL = "openai/gpt-oss-120b"
GROQ_FALLBACK_MODELS = ("openai/gpt-oss-20b",)
# Groq sits behind Cloudflare, which rejects urllib's default User-Agent.
USER_AGENT = "drone-llm-pipeline/1.0"
# Attempts per model when a provider blocks us (HF rotates providers per call,
# so a plain retry usually routes around a Cloudflare-blocked one).
PROVIDER_ATTEMPTS = 3
PROVIDER_BACKOFF_S = 1.5
# "low" keeps reasoning models from burning the budget on chain-of-thought
# before emitting JSON. Set HF_REASONING_EFFORT="" to disable the field.
REASONING_EFFORT = os.environ.get("HF_REASONING_EFFORT", "low")


class LLMError(RuntimeError):
    pass


class _ModelNotFound(LLMError):
    """HTTP 404 for a specific model id on the router."""


class _ProviderUnavailable(LLMError):
    """A third-party provider behind the router refused us (403 / 429 / 5xx).

    Recoverable. HF fans each model out across providers and picks one per
    request, so a Cloudflare block at (say) Together says nothing about the
    others - simply retrying usually lands on a different provider.
    """


class _Truncated(LLMError):
    """Model hit the token ceiling before emitting content.

    Typical of reasoning models (gpt-oss, R1-style): the budget is consumed by
    internal chain-of-thought and `content` comes back empty. Recoverable by
    retrying with a bigger budget / lower reasoning effort.
    """


def _strip_code_fences(text: str) -> str:
    """Models occasionally wrap JSON in ``` fences despite instructions."""
    t = text.strip()
    m = re.match(r"^```(?:json)?\s*(.*?)\s*```$", t, flags=re.DOTALL)
    return m.group(1).strip() if m else t


def _extract_json_object(text: str) -> str:
    """Return the first balanced {...} block, ignoring any prose around it.

    Chattier models prefix explanations or append notes despite instructions;
    brace-matching (string- and escape-aware) recovers the payload instead of
    failing the whole mission on a stray sentence.
    """
    t = _strip_code_fences(text)
    start = t.find("{")
    if start == -1:
        return t
    depth = 0
    in_str = False
    esc = False
    for i, ch in enumerate(t[start:], start):
        if esc:
            esc = False
            continue
        if ch == "\\":
            esc = True
            continue
        if ch == '"':
            in_str = not in_str
            continue
        if in_str:
            continue
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                return t[start:i + 1]
    return t[start:]          # unbalanced -> let the validator report it


def _extract_chat_text(payload: dict) -> str:
    """Pull text out of an OpenAI-schema chat completion; loud on anomalies."""
    choices = payload.get("choices") or []
    if not choices:
        err = payload.get("error") or payload
        raise LLMError(f"HF router returned no choices: {str(err)[:300]}")
    choice = choices[0]
    msg = choice.get("message") or {}
    text = msg.get("content") or ""

    # Reasoning models sometimes park the answer in a reasoning field when the
    # budget runs out mid-stream; mine it before giving up.
    if not text.strip():
        for key in ("reasoning_content", "reasoning"):
            candidate = msg.get(key) or ""
            if "{" in candidate:
                text = candidate
                break

    if not text.strip():
        fr = choice.get("finish_reason", "unknown")
        if fr == "length":
            raise _Truncated("token budget exhausted before any content")
        raise LLMError(
            f"HF router returned empty message content (finish_reason={fr})")
    return text


# --------------------------------------------------------------------------- #
# Backend 1: Hugging Face Inference Providers (router)
# --------------------------------------------------------------------------- #

class HuggingFaceBackend:
    """Hugging Face Inference Providers backend (OpenAI-compatible router).

    Model selection: HF_MODEL env / constructor arg pins a model (no silent
    substitution). Otherwise the default is tried, then the fallback chain;
    if all are unavailable, the raised error lists models this token can call
    so the fix is always `export HF_MODEL=<one of them>`."""
    name = "huggingface"
    label = "HF router"
    chat_url = HF_CHAT_URL
    models_url = HF_MODELS_URL
    token_envs: tuple = HF_TOKEN_ENVS
    model_env = "HF_MODEL"
    default_model = DEFAULT_HF_MODEL
    fallback_models: tuple = HF_FALLBACK_MODELS
    missing_token = ("HF_TOKEN not set (create a fine-grained token with the "
                     "'Make calls to Inference Providers' permission at "
                     "https://hf.co/settings/tokens)")

    def __init__(self, api_token: Optional[str] = None,
                 model: Optional[str] = None, timeout_s: float = 120.0):
        self.api_token = api_token or next(
            (os.environ[v] for v in self.token_envs if os.environ.get(v)), "")
        if not self.api_token:
            raise LLMError(self.missing_token)
        env_model = os.environ.get(self.model_env)
        self.model = model or env_model or self.default_model
        self.model_pinned = bool(model or env_model)
        self.timeout_s = timeout_s
        self._token_live: Optional[bool] = None   # cached by _token_is_live()
        self._system_prompt = SYSTEM_PROMPT

    # ------------------------------------------------------------------ #

    def propose(self, natural_language_request: str) -> str:
        self._system_prompt = SYSTEM_PROMPT
        return self._complete(build_user_prompt(natural_language_request))

    def propose_step(self, request: str, state: dict, history: list) -> str:
        """One interactive step (see prompts.STEP_SYSTEM_PROMPT)."""
        self._system_prompt = STEP_SYSTEM_PROMPT
        return tidy_mission_name(self._complete(build_step_user_prompt(request, state, history)))

    def _complete(self, user_prompt: str) -> str:
        if self.model_pinned:
            chain: List[str] = [self.model]
        else:
            chain = [self.model] + [m for m in self.fallback_models
                                    if m != self.model]
        last: Optional[Exception] = None
        for m in chain:
            # Escalate the budget before writing a model off: complex missions
            # (grids, multi-phase plans) legitimately need more room, and
            # reasoning models spend most of it before emitting any content.
            for budget in TOKEN_BUDGETS:
                # The router picks a provider per request, so an outright
                # provider block is worth retrying before moving on.
                for attempt in range(PROVIDER_ATTEMPTS):
                    try:
                        payload = self._chat(m, user_prompt, budget)
                        if m != chain[0]:
                            print(f"[llm_layer] {self.label} model '{chain[0]}' unavailable "
                                  f"-> using fallback '{m}'", file=sys.stderr)
                        return _extract_json_object(_extract_chat_text(payload))
                    except _ProviderUnavailable as e:
                        last = e
                        if attempt < PROVIDER_ATTEMPTS - 1:
                            print(f"[llm_layer] {e} — retrying "
                                  f"({attempt + 2}/{PROVIDER_ATTEMPTS})",
                                  file=sys.stderr)
                            time.sleep(PROVIDER_BACKOFF_S * (attempt + 1))
                        continue
                    except _Truncated as e:
                        last = e
                        print(f"[llm_layer] '{m}' truncated at max_tokens={budget}"
                              f" -> retrying with a larger budget", file=sys.stderr)
                        break        # escalate the budget, not the provider
                    except _ModelNotFound as e:
                        last = e
                        break
                if isinstance(last, _ModelNotFound):
                    break            # budget will not fix a missing model
        available = self._list_models()
        raise LLMError(
            f"No {self.label} model from {chain} produced a usable mission "
            f"(last error: {last}). Models this token can call: "
            f"{available if available else '(model list lookup failed)'} "
            f"-> set {self.model_env} to one of them.") from last

    @staticmethod
    def _model_understands_reasoning_effort(model: str) -> bool:
        # "reasoning_effort" is an OpenAI gpt-oss-only setting.
        # gpt-oss models understand it. Every other model (Qwen, Llama,
        # Mistral, Kimi, ...) answers HTTP 400 "thinking mode openai_effort
        # is not supported" if we send it. So: send it ONLY to gpt-oss models.
        # This does NOT restrict which models you can use — non-gpt-oss models
        # simply get the request without this one extra field, and work fine.
        return "gpt-oss" in model.lower()

    def _chat(self, model: str, user_prompt: str, max_tokens: int,
              send_reasoning_effort: Optional[bool] = None) -> dict:
        # LAYER 1: decide whether to include the reasoning_effort setting.
        # Include it only if it is turned on AND this model understands it.
        if send_reasoning_effort is None:
            send_reasoning_effort = (
                bool(REASONING_EFFORT)
                and self._model_understands_reasoning_effort(model))

        payload = {
            "model": model,
            "messages": [
                {"role": "system", "content": self._system_prompt},
                {"role": "user", "content": user_prompt},
            ],
            "temperature": 0,
            "max_tokens": max_tokens,
            "stream": False,
        }
        if send_reasoning_effort:
            payload["reasoning_effort"] = REASONING_EFFORT
        body = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(
            self.chat_url,
            data=body,
            headers={
                "content-type": "application/json",
                "user-agent": USER_AGENT,
                "authorization": f"Bearer {self.api_token}",
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                return json.loads(resp.read().decode("utf-8"))
        except urllib.error.HTTPError as e:
            detail = e.read().decode("utf-8", errors="replace")[:500]
            # "Model does not exist" arrives as 404, or as a 400 with
            # code "model_not_found". Treat both the same: walk to the next
            # model in the chain instead of failing the whole request.
            if e.code == 404 or (e.code == 400 and re.search(
                    r"model_not_found|does not exist", detail, re.I)):
                raise _ModelNotFound(
                    f"HF model '{model}' not found on the router: {detail}") from e
            # LAYER 2 (safety net): a model rejected the reasoning_effort setting
            # with a 400 even though Layer 1 tried to send it. Retry the SAME
            # request once, this time without that field. Covers any model whose
            # name we did not recognise as gpt-oss.
            if (e.code == 400 and send_reasoning_effort
                    and re.search(r"reasoning|thinking|effort", detail, re.I)):
                return self._chat(model, user_prompt, max_tokens,
                                  send_reasoning_effort=False)
            # A 403/429/5xx usually comes from the third-party provider the
            # router happened to pick, not from HF or the token. Those are
            # retryable: another attempt normally lands on a different provider.
            retryable = e.code in (403, 429) or 500 <= e.code < 600
            # A 401 is ambiguous: it can be a dead HF token (fatal) OR the
            # provider the router picked rejecting ITS OWN credentials
            # (retryable). Settle it by asking HF directly - if the token still
            # lists models, the 401 came from downstream, not from us.
            if e.code == 401 and self._token_is_live():
                retryable = True
            if retryable:
                blocker = re.search(r"api\.[a-z0-9.-]+\.(?:xyz|ai|com)", detail)
                who = f" at {blocker.group(0)}" if blocker else ""
                raise _ProviderUnavailable(
                    f"provider refused (HTTP {e.code}){who}") from e
            hints = {
                401: " (401 = invalid/expired HF token — it no longer "
                     "authenticates against the HF router itself)",
                402: " (402 = Inference Providers credits exhausted — free "
                     "monthly quota used up; top up or wait for reset)",
            }
            raise LLMError(
                f"{self.label} HTTP {e.code}{hints.get(e.code, '') if self.name == 'huggingface' else ''}: {detail}") from e
        except urllib.error.URLError as e:
            raise LLMError(f"{self.label} unreachable: {e.reason}") from e

    def _token_is_live(self) -> bool:
        """Does the token still authenticate against the HF router itself?

        Used only to disambiguate a 401. Cached: the answer cannot change
        mid-run, and this must not add a round-trip to every retry.
        """
        if self._token_live is None:
            req = urllib.request.Request(
                self.models_url,
                headers={"authorization": f"Bearer {self.api_token}",
                         "user-agent": USER_AGENT})
            try:
                with urllib.request.urlopen(req, timeout=20) as resp:
                    self._token_live = resp.status == 200
            except urllib.error.HTTPError as e:
                self._token_live = e.code != 401
            except urllib.error.URLError:
                self._token_live = True      # network blip -> assume ours is ok
        return bool(self._token_live)

    def _list_models(self) -> list:
        """Best-effort GET /v1/models for actionable error messages."""
        req = urllib.request.Request(
            self.models_url,
            headers={"authorization": f"Bearer {self.api_token}",
                     "user-agent": USER_AGENT})
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                data = json.loads(resp.read().decode("utf-8"))
            names = [m.get("id", "") for m in data.get("data", [])]
            return sorted(n for n in names if n)[:25]
        except Exception:
            return []


class GroqBackend(HuggingFaceBackend):
    """Groq (OpenAI-compatible). Same request/retry/fallback logic as the HF
    router backend - only the endpoint, key and model defaults differ."""
    name = "groq"
    label = "Groq"
    chat_url = GROQ_CHAT_URL
    models_url = GROQ_MODELS_URL
    token_envs = ("GROQ_API_KEY",)
    model_env = "GROQ_MODEL"
    default_model = DEFAULT_GROQ_MODEL
    fallback_models = GROQ_FALLBACK_MODELS
    missing_token = ("GROQ_API_KEY not set (create a free key at "
                     "https://console.groq.com/keys)")


# --------------------------------------------------------------------------- #
# Backend 2: deterministic offline heuristic (no network, no keys)
# --------------------------------------------------------------------------- #

_WORD_COUNTS = {"once": 1, "twice": 2, "thrice": 3, "one": 1, "two": 2,
                "three": 3, "four": 4, "five": 5, "six": 6, "seven": 7,
                "eight": 8, "nine": 9, "ten": 10}


class OfflineHeuristicBackend:
    """Deterministic rule-based request parser. Same prompt in -> same JSON out."""
    name = "offline"

    # Documented defaults (mirror the LLM system prompt)
    DEFAULT_ALT = 10.0
    DEFAULT_DRONE_SPEED = 5.0
    DEFAULT_GROUND_SPEED = 0.5
    PERIMETER_HALF = 20.0  # "perimeter" = square with corners (±20, ±20)

    def propose(self, natural_language_request: str) -> str:
        text = natural_language_request.lower()

        ground = bool(re.search(r"\b(ground robot|rover|turtlebot|ugv)\b", text))
        vehicle = "ground_robot" if ground else "drone"

        alt = self.DEFAULT_ALT
        m = re.search(r"(\d+(?:\.\d+)?)\s*(?:m\b|meter|metre)", text)
        if m:
            alt = float(m.group(1))
        if ground:
            alt = 0.0

        speed = self.DEFAULT_GROUND_SPEED if ground else self.DEFAULT_DRONE_SPEED
        m = re.search(r"(\d+(?:\.\d+)?)\s*(?:m/s|mps|metres? per second)", text)
        if m:
            speed = float(m.group(1))

        count = 1
        m = re.search(r"(\d+)\s*(?:times|laps?|loops?|circuits?)", text)
        if m:
            count = int(m.group(1))
        else:
            for word, n in _WORD_COUNTS.items():
                if re.search(rf"\b{word}\b", text):
                    count = n
                    break

        half = self.PERIMETER_HALF
        m = re.search(r"(\d+(?:\.\d+)?)\s*(?:m\b|meter|metre)[a-z]*\s+(?:square|box)", text)
        if m:
            half = float(m.group(1)) / 2.0

        square = [
            {"x": half, "y": half, "alt": alt},
            {"x": half, "y": -half, "alt": alt},
            {"x": -half, "y": -half, "alt": alt},
            {"x": -half, "y": half, "alt": alt},
        ]

        name = re.sub(r"[^A-Za-z0-9 _-]", "", natural_language_request)[:60].strip() or "mission"
        mission = {
            "schema_version": "1.0",
            "mission_name": name,
            "vehicle": vehicle,
            "frame": "LOCAL_ENU_METERS",
            "cruise_speed_mps": speed,
            "commands": [],
        }
        if vehicle == "drone":
            mission["commands"].append({"type": "TAKEOFF", "alt": alt})
        mission["commands"].append({"type": "LOOP", "count": count, "waypoints": square})
        if vehicle == "drone":
            mission["commands"].append({"type": "RTL"})

        return json.dumps(mission, indent=2, sort_keys=True)


_DIRECTIONS = {
    # word -> (axis, sign); body axes follow the heading, world axes do not
    "left": ("right", -1), "right": ("right", 1),
    "forward": ("forward", 1), "forwards": ("forward", 1), "ahead": ("forward", 1),
    "back": ("forward", -1), "backward": ("forward", -1), "backwards": ("forward", -1),
    "north": ("north", 1), "south": ("north", -1),
    "east": ("east", 1), "west": ("east", -1),
    "up": ("up", 1), "down": ("up", -1),
}
_NUM = r"(\d+(?:\.\d+)?)"
_M = r"\s*(?:m\b|meters?\b|metres?\b)"
_KM = re.compile(r"(\d+(?:\.\d+)?)\s*(?:km\b|kms\b|kilomet(?:er|re)s?\b)")


def km_to_m(text: str) -> str:
    """'1.5 km' -> '1500 m', so every distance parser only has to know metres."""
    return _KM.sub(lambda m: f"{float(m.group(1)) * 1000:g} m", text)


_NAME_OK = re.compile(r"[^A-Za-z0-9 _-]+")


def tidy_mission_name(raw: str) -> str:
    """mission_name is only a label, but the schema restricts its characters;
    models write things like "patrol 1.5km square". Fix the label rather than
    reject a sound plan over punctuation. Nothing that flies is touched, and
    anything that is not a JSON object is returned unchanged for the validator."""
    try:
        data = json.loads(raw)
    except ValueError:
        return raw
    if isinstance(data, dict) and isinstance(data.get("mission_name"), str):
        name = _NAME_OK.sub(" ", data["mission_name"]).strip()[:80] or "step"
        if name != data["mission_name"]:
            data["mission_name"] = name
            return json.dumps(data)
    return raw


class OfflineStepParser:
    """Deterministic parser for interactive steps (no network, no keys).

    Understands the common one-line commands: relative moves, climb/descend,
    go to x,y, survey AxB, orbit, hover N s, photo, take off. Anything else
    raises LLMError so the operator is told plainly rather than guessed at.
    """

    def propose_step(self, request: str, state: dict, history: list) -> str:
        text = km_to_m(request.lower().replace("×", "x"))
        pos = state["position"]
        x, y, alt = pos["x"], pos["y"], pos["alt"]
        airborne = state["in_air"]
        cmds: list = []
        notes: list = []

        m = re.search(r"\b(?:take ?off|launch|lift ?off)\b(?:[^\d]*" + _NUM + _M + ")?", text)
        to_alt = float(m.group(1)) if m and m.group(1) else None
        if not airborne:
            to_alt = to_alt or 10.0
            cmds.append({"type": "TAKEOFF", "alt": to_alt})
            notes.append(f"take off to {to_alt:g} m")
            alt = to_alt

        at = re.search(r"\bat\s+" + _NUM + _M, text)
        work_alt = float(at.group(1)) if at else alt

        survey = re.search(r"\b(?:survey|map|scan|cover)\b.*?" + _NUM + r"\s*(?:m\s*)?(?:x|by)\s*"
                           + _NUM + r"(?:" + _M + r")?", text)
        orbit = re.search(r"\b(?:orbit|circle)\b", text)
        goto = re.search(r"\b(?:go|fly|move|head)\s+to\s*\(?\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\)?", text)
        climb_to = re.search(r"\b(?:climb|ascend|descend|go up|go down|altitude)\s*(?:to)\s*" + _NUM + _M, text)
        climb_by = re.search(r"\b(climb|ascend|rise|descend|drop|lower)\s*(?:by\s*)?" + _NUM + _M, text)
        hover = re.search(r"\b(?:hover|hold|wait|stay)\b.*?" + _NUM + r"\s*(?:s\b|sec|seconds?)", text)
        moves = re.findall(_NUM + _M + r"\s*(?:to the\s+)?(left|right|forwards?|ahead|backwards?|back|north|south|east|west|up|down)\b", text)
        moves += [(n, d) for d, n in re.findall(r"\b(left|right|forwards?|ahead|backwards?|back|north|south|east|west|up|down)\s*(?:by\s*)?" + _NUM + _M, text)]

        if survey:
            w, h = float(survey.group(1)), float(survey.group(2))
            sp = re.search(_NUM + _M + r"\s*(?:lanes?|spacing|apart)", text)
            spacing = float(sp.group(1)) if sp else max(2.0, min(10.0, min(w, h) / 4.0))
            grid = {"type": "GRID", "center_x": x, "center_y": y, "width": w, "height": h,
                    "spacing": spacing, "alt": work_alt}
            if re.search(r"\b(photo|photograph|picture|capture|image)", text):
                grid["capture"] = True
            cmds.append(grid)
            notes.append(f"survey {w:g} x {h:g} m centred here at {work_alt:g} m")
        elif orbit:
            r = re.search(r"(?:radius\s*(?:of\s*)?" + _NUM + _M + "|" + _NUM + _M + r"\s*radius)", text)
            radius = float(next(g for g in r.groups() if g)) if r else 15.0
            t = re.search(r"(\d+)\s*(?:times|turns|laps)|\b(once|twice|thrice)\b", text)
            turns = int(t.group(1)) if t and t.group(1) else {"twice": 2, "thrice": 3}.get(t.group(2), 1) if t else 1
            cmds.append({"type": "ORBIT", "x": x, "y": y, "alt": work_alt, "radius": radius,
                         "turns": turns, "points_per_turn": 12})
            notes.append(f"orbit {radius:g} m around here x{turns}")
        elif goto:
            gx, gy = float(goto.group(1)), float(goto.group(2))
            cmds.append({"type": "GOTO", "x": gx, "y": gy, "alt": work_alt})
            notes.append(f"go to ({gx:g}, {gy:g}) at {work_alt:g} m")
        elif climb_to:
            target = float(climb_to.group(1))
            cmds.append({"type": "GOTO", "x": x, "y": y, "alt": target})
            notes.append(f"change altitude to {target:g} m")
        elif climb_by:
            d = float(climb_by.group(2)) * (1 if climb_by.group(1) in ("climb", "ascend", "rise") else -1)
            cmds.append({"type": "MOVE", "up": d})
            notes.append(f"{'climb' if d > 0 else 'descend'} {abs(d):g} m")
        elif moves:
            body: dict = {}
            world: dict = {}
            for n, word in moves:
                axis, sign = _DIRECTIONS[word]
                bucket = world if axis in ("north", "east") else body
                bucket[axis] = bucket.get(axis, 0.0) + sign * float(n)
                notes.append(f"move {float(n):g} m {word}")
            if world:
                if "up" in body and not ("forward" in body or "right" in body):
                    world["up"] = body.pop("up")
                cmds.append({"type": "MOVE", **world})
            if body:
                cmds.append({"type": "MOVE", **body})
        elif hover:
            secs = float(hover.group(1))
            cmds.append({"type": "HOLD", "seconds": secs})
            notes.append(f"hover {secs:g} s")

        if re.search(r"\b(take|snap|grab|capture)\b.*\b(photo|picture|image|shot)\b", text) and not survey:
            cmds.append({"type": "CAPTURE", "label": "photo"})
            notes.append("take a photo")

        if re.search(r"\bthen\s+(land|return home|come home|go home|rtl)\b", text):
            land = "LAND" if "land" in text.split("then")[-1] else "RTL"
            cmds.append({"type": land})
            notes.append("land" if land == "LAND" else "return home")

        flight = [c for c in cmds if c["type"] != "TAKEOFF"]
        if not flight and not (cmds and not airborne and m):
            raise LLMError(
                f"offline parser could not understand {request!r}. Try e.g. "
                "'move 5 m left', 'climb 10 m', 'survey 40x40 m', 'orbit 20 m radius', "
                "'go to 30, 40', 'hover 10 s', 'take a photo'.")

        name = re.sub(r"[^A-Za-z0-9 _-]", "", request)[:60].strip() or "step"
        return json.dumps({
            "schema_version": "1.0", "mission_name": name, "vehicle": "drone",
            "frame": "LOCAL_ENU_METERS", "cruise_speed_mps": 5.0,
            "notes": ("Plan: " + ", then ".join(notes) + ".")[:500],
            "commands": cmds,
        }, sort_keys=True)


# --------------------------------------------------------------------------- #
# Public API
# --------------------------------------------------------------------------- #

def _auto_backend() -> str:
    if os.environ.get("GROQ_API_KEY"):
        return "groq"
    if any(os.environ.get(v) for v in HF_TOKEN_ENVS):
        return "huggingface"
    return "offline"


def propose_mission_json(natural_language_request: str, backend: str = "auto") -> str:
    """Return a PROPOSED (untrusted) mission JSON string.

    backend: "auto" (Groq if GROQ_API_KEY set, else Hugging Face if HF_TOKEN
             set, else offline), "groq", "huggingface" (alias "hf"), or "offline".
    """
    if backend == "auto":
        backend = _auto_backend()
        if backend == "offline":
            print("[llm_layer] no GROQ_API_KEY / HF_TOKEN set -> using deterministic "
                  "offline backend (export a key to use the real LLM).",
                  file=sys.stderr)

    if backend == "groq":
        return GroqBackend().propose(natural_language_request)
    if backend in ("huggingface", "hf"):
        return HuggingFaceBackend().propose(natural_language_request)
    if backend == "offline":
        return OfflineHeuristicBackend().propose(natural_language_request)
    raise LLMError(f"unknown LLM backend '{backend}' "
                   "(use auto|groq|huggingface|offline)")


def propose_step_json(request: str, state: dict, history: list,
                      backend: str = "auto") -> str:
    """Return a PROPOSED (untrusted) interactive-step JSON string.

    `state` is the live vehicle snapshot shown to the model; `history` is a
    list of {"operator": ..., "result": ...} turns (oldest first). Same
    backend selection as propose_mission_json().
    """
    if backend == "auto":
        backend = _auto_backend()
    if backend == "groq":
        return GroqBackend().propose_step(request, state, history)
    if backend in ("huggingface", "hf"):
        return HuggingFaceBackend().propose_step(request, state, history)
    if backend == "offline":
        return OfflineStepParser().propose_step(request, state, history)
    raise LLMError(f"unknown LLM backend '{backend}' "
                   "(use auto|groq|huggingface|offline)")
