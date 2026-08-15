"""LLM layer public interface. Proposes untrusted JSON strings; nothing more."""
from .llm_client import (
    HuggingFaceBackend,
    LLMError,
    OfflineHeuristicBackend,
    propose_mission_json,
)

__all__ = ["propose_mission_json", "HuggingFaceBackend",
           "OfflineHeuristicBackend", "LLMError"]
