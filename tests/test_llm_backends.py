"""LLM backend tests — fully offline: payload parsing, fences, selection,
and the model fallback chain. No network calls; router responses are faked.
The point is that the HF backend fails LOUDLY on every anomaly so the
pipeline never silently feeds garbage to the validator."""
import json

import pytest

from llm_layer import HuggingFaceBackend, LLMError, propose_mission_json
from llm_layer.llm_client import (
    DEFAULT_HF_MODEL,
    _extract_chat_text,
    _extract_json_object,
    _ModelNotFound,
    _strip_code_fences,
    _Truncated,
)

TOKEN_VARS = ("HF_TOKEN", "HUGGINGFACE_API_KEY", "HF_API_TOKEN")


def _clear_env(monkeypatch):
    for var in TOKEN_VARS + ("HF_MODEL",):
        monkeypatch.delenv(var, raising=False)


def test_strip_code_fences_variants():
    assert _strip_code_fences('```json\n{"a": 1}\n```') == '{"a": 1}'
    assert _strip_code_fences('```\n{"a": 1}\n```') == '{"a": 1}'
    assert _strip_code_fences('{"a": 1}') == '{"a": 1}'


def test_chat_payload_extraction_happy_path():
    payload = {"choices": [{"message": {"content": '{"x": 1}'},
                            "finish_reason": "stop"}]}
    assert _extract_chat_text(payload) == '{"x": 1}'


def test_no_choices_is_loud():
    with pytest.raises(LLMError, match="no choices"):
        _extract_chat_text({"error": {"message": "boom"}})


def test_empty_content_is_loud():
    """Empty content for a non-budget reason is a hard, loud failure."""
    with pytest.raises(LLMError, match="empty message content"):
        _extract_chat_text({"choices": [{"message": {"content": ""},
                                         "finish_reason": "stop"}]})


def test_budget_exhaustion_is_retryable_not_fatal():
    """finish_reason=length must raise the retryable _Truncated so propose()
    can escalate the budget instead of failing the whole mission."""
    with pytest.raises(_Truncated):
        _extract_chat_text({"choices": [{"message": {"content": ""},
                                         "finish_reason": "length"}]})


def test_reasoning_field_is_mined_when_content_empty():
    """Reasoning models sometimes park the JSON in a reasoning field."""
    payload = {"choices": [{
        "message": {"content": "", "reasoning_content": 'ok: {"x": 1}'},
        "finish_reason": "length"}]}
    assert '{"x": 1}' in _extract_chat_text(payload)


def test_json_object_extracted_from_prose_and_braces_in_strings():
    assert _extract_json_object('Here you go:\n{"a":{"b":2}}\nthanks') == '{"a":{"b":2}}'
    assert _extract_json_object('{"name":"a}b","c":3} junk') == '{"name":"a}b","c":3}'
    assert _extract_json_object(r'{"s":"esc \" }","d":4} junk') == r'{"s":"esc \" }","d":4}'


def test_backend_requires_token(monkeypatch):
    _clear_env(monkeypatch)
    with pytest.raises(LLMError, match="HF_TOKEN"):
        HuggingFaceBackend()


def test_auto_with_no_token_falls_back_to_offline(monkeypatch, capsys):
    _clear_env(monkeypatch)
    out = propose_mission_json("Patrol the perimeter once at 15 metres",
                               backend="auto")
    json.loads(out)                      # offline backend produced valid JSON
    assert "offline backend" in capsys.readouterr().err


def test_auto_prefers_hf_when_token_set(monkeypatch):
    """Selection only — the fake token must never reach the network."""
    _clear_env(monkeypatch)
    monkeypatch.setenv("HF_TOKEN", "fake-token-for-selection-test")
    import llm_layer.llm_client as lc

    class Marker(Exception):
        pass

    class FakeHF:
        def __init__(self, *a, **k):
            raise Marker()

    monkeypatch.setattr(lc, "HuggingFaceBackend", FakeHF)
    with pytest.raises(Marker):
        lc.propose_mission_json("x", backend="auto")


def test_model_fallback_chain_walks_on_404(monkeypatch):
    _clear_env(monkeypatch)
    b = HuggingFaceBackend(api_token="fake")
    calls = []

    def fake_chat(model, prompt, max_tokens):
        calls.append(model)
        if model == DEFAULT_HF_MODEL:
            raise _ModelNotFound("retired")
        return {"choices": [{"message": {"content": "{}"},
                             "finish_reason": "stop"}]}

    monkeypatch.setattr(b, "_chat", fake_chat)
    assert b.propose("x") == "{}"
    # a 404 must not be retried at a bigger budget - straight to the next model
    assert calls[0] == DEFAULT_HF_MODEL and len(calls) == 2


def test_truncation_retries_with_a_larger_budget(monkeypatch):
    """A reasoning model that blows the budget must be retried, not abandoned."""
    _clear_env(monkeypatch)
    b = HuggingFaceBackend(api_token="fake", model="pinned/model")
    budgets = []

    def fake_chat(model, prompt, max_tokens):
        budgets.append(max_tokens)
        if len(budgets) == 1:
            raise _Truncated("budget exhausted")
        return {"choices": [{"message": {"content": '{"ok":1}'},
                             "finish_reason": "stop"}]}

    monkeypatch.setattr(b, "_chat", fake_chat)
    assert b.propose("complex mission") == '{"ok":1}'
    assert len(budgets) == 2 and budgets[1] > budgets[0]


def test_pinned_model_never_silently_substituted(monkeypatch):
    _clear_env(monkeypatch)
    b = HuggingFaceBackend(api_token="fake", model="acme/NoSuchModel-9B")
    monkeypatch.setattr(
        b, "_chat",
        lambda m, p, mt: (_ for _ in ()).throw(_ModelNotFound("gone")))
    monkeypatch.setattr(b, "_list_models",
                        lambda: ["openai/gpt-oss-120b", "Qwen/Qwen3-8B"])
    with pytest.raises(LLMError, match="set HF_MODEL") as ei:
        b.propose("x")
    assert "gpt-oss-120b" in str(ei.value)   # actionable: lists callable models


def test_unknown_backend_rejected():
    with pytest.raises(LLMError, match="unknown LLM backend"):
        propose_mission_json("x", backend="gemini")
