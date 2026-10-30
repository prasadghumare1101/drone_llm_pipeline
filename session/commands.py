"""Instant operator commands that bypass the LLM.

stop / hold / land / return home must act immediately, so they are recognised
by a deterministic parser BEFORE any model is consulted. The parser is
deliberately strict: it only fires when the WHOLE message is one of these
commands (after dropping politeness and filler), so "survey the land behind
the barn" or "stop at the tree after the survey" go to the planner instead of
landing the drone.
"""
from __future__ import annotations

import re
from enum import Enum
from typing import Optional

from llm_layer.llm_client import km_to_m


class FastCommand(str, Enum):
    HOLD = "hold"
    LAND = "land"
    RTL = "rtl"
    RESUME = "resume"
    STATUS = "status"


_FILLER = {"please", "pls", "now", "immediately", "asap", "right", "quickly", "drone",
           "the", "ok", "okay", "hey", "just", "and", "it", "uav", "copter", "you",
           "can", "could", "would", "kindly", "go", "do"}

# phrases after filler removal (note "go" is filler, so "go home" -> "home")
_PHRASES = {
    FastCommand.HOLD: {
        "stop", "hold", "halt", "pause", "freeze", "wait", "hover", "abort", "cancel",
        "brake", "stay", "stay there", "stay here", "hold position", "hold on",
        "hold here", "stop here", "stop moving", "hover here", "hover there",
        "stop mission", "emergency stop", "stop stop",
    },
    FastCommand.LAND: {
        "land", "land here", "land there", "land at current position", "touch down",
        "touchdown", "set down", "emergency land", "land emergency",
    },
    FastCommand.RTL: {
        "home", "return home", "return to home", "come home", "come back", "return",
        "rtl", "rth", "return to launch", "return to base", "head home", "fly home",
        "back home", "back to home", "return to start", "come back home",
        "return home and land",
    },
    FastCommand.RESUME: {
        "resume", "continue", "carry on", "keep going", "on", "proceed",
        "resume mission", "continue mission", "hand back", "take control",
        "you have control", "release",
    },
    FastCommand.STATUS: {
        "status", "where are", "where are now", "report", "battery", "position",
        "what is your status", "what's your status", "whats your status",
        "status report", "battery status", "where is",
    },
}

_LOOKUP = {phrase: cmd for cmd, phrases in _PHRASES.items() for phrase in phrases}

# "come to home now", "bring it back to base", "fly straight back home": any
# message made ONLY of these words that names home/base/launch is a return
# home. Listing every phrasing does not scale; a message with any other word
# ("survey the field near home") still goes to the planner.
_HOME_WORDS = {"come", "coming", "return", "returning", "fly", "head", "bring", "get", "back",
               "to", "towards", "home", "base", "launch", "point", "pad", "start", "rtl", "rth",
               "straight", "directly", "me", "here", "your", "our"}
_HOME_ANCHORS = {"home", "base", "launch", "rtl", "rth"}


def normalise(text: str) -> str:
    t = re.sub(r"[^a-z0-9' ]+", " ", text.lower())
    words = [w for w in t.split() if w not in _FILLER]
    return " ".join(words)


def parse_fast_command(text: str) -> Optional[FastCommand]:
    """Return the instant command `text` is, or None if it needs the planner."""
    if len(text) > 60:          # a sentence that long is an instruction, not a reflex
        return None
    norm = normalise(text)
    hit = _LOOKUP.get(norm)
    if hit is not None:
        return hit
    words = set(norm.split())
    if words and words <= _HOME_WORDS and words & _HOME_ANCHORS:
        return FastCommand.RTL
    return None


# --------------------------------------------------------------------------- #
# Altitude change for the step that is flying right now
# --------------------------------------------------------------------------- #

_ALT_WORDS = re.compile(r"\b(height|altitude|alt|climb|ascend|descend|raise|lower|higher|up|"
                        r"down|increase|decrease|drop|rise|elevation)\b")
_UP = re.compile(r"\b(climb|ascend|raise|higher|up|increase|rise)\b")
_DOWN = re.compile(r"\b(descend|lower|down|decrease|drop)\b")
# anything that makes it more than a pure height instruction -> the planner
_NOT_ALT = re.compile(r"\b(left|right|forward|forwards|back|backward|backwards|north|south|east|"
                      r"west|survey|orbit|patrol|circle|grid|spiral|land|home|photo|picture|"
                      r"go to|fly to|then|and then|square|speed|faster|slower)\b")
_NUM_M = re.compile(r"(-?\d+(?:\.\d+)?)\s*(?:m\b|meters?\b|metres?\b|mtrs?\b)?")


def parse_altitude_change(text: str) -> Optional[tuple]:
    """("to", 40.0) for an absolute target, ("by", +10.0 / -10.0) for a relative
    change, or None if `text` is not purely an altitude instruction.

    "raise the height to 40 m" -> ("to", 40)   "climb 10 m" -> ("by", 10)
    "go down by 5 metres"      -> ("by", -5)   "altitude 25" -> ("to", 25)
    """
    t = km_to_m(text.lower().strip())
    if len(t) > 80 or not _ALT_WORDS.search(t) or _NOT_ALT.search(t):
        return None
    m = _NUM_M.search(t)
    if not m:
        return None
    n = abs(float(m.group(1)))
    up, down = bool(_UP.search(t)), bool(_DOWN.search(t))
    if re.search(r"\bto\s+-?\d", t) or not (up or down):
        return ("to", n)
    if up == down:
        return None                       # "up ... down" - ambiguous, ask the planner
    return ("by", n if up else -n)


# --------------------------------------------------------------------------- #
# "add waypoint at x, y [next | at the end]"
# --------------------------------------------------------------------------- #

_WP = re.compile(r"\b(?:add|insert|put|append)\b.*?\b(?:way ?points?|wps?|points?)\b.*?"
                 r"\(?\s*(-?\d+(?:\.\d+)?)\s*m?\s*,\s*(-?\d+(?:\.\d+)?)\s*m?\s*\)?")
_WP_NEXT = re.compile(r"\b(next|now|first|immediately|before|straight away|right away)\b")


def parse_waypoint_command(text: str) -> Optional[tuple]:
    """((x, y), "next" | "end") for "add waypoint at 120, 80 next" (x = East,
    y = North, metres or km), else None."""
    t = km_to_m(text.lower())
    m = _WP.search(t)
    if not m:
        return None
    return (float(m.group(1)), float(m.group(2))), ("next" if _WP_NEXT.search(t) else "end")
