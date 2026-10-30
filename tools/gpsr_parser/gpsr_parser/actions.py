"""Actions and data models for GPSR plans."""

from dataclasses import asdict, dataclass
from typing import Any, Dict, List, Optional


@dataclass
class Action:
    action: str
    params: Dict[str, Any]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "action": self.action,
            "params": self.params,
        }


@dataclass
class PlanResult:
    success: bool
    command: str
    actions: List[Dict[str, Any]]
    explanation: str
    error: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "success": self.success,
            "command": self.command,
            "actions": self.actions,
            "explanation": self.explanation,
            "error": self.error,
        }
