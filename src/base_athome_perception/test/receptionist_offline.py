"""Load actual state-machine methods without importing ROS or starting a node.

Only constants and selected method definitions are compiled from the script AST;
imports, __init__, main, hardware helpers and module-level side effects are excluded.
"""

import ast
import datetime
import re
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from types import SimpleNamespace
from typing import Dict

import numpy as np

from base_athome_perception.receptionist_attributes import (
    describe_guest_attributes, extract_guest_attributes, prefer_attributes,
)
from base_athome_perception.receptionist_doorbell import doorbell_wait_status

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE.parent / "base_athome_skills"))
from base_athome_skills.door_detector import window_distance  # noqa: E402


def load_receptionist():
    script = PACKAGE / "scripts" / "hri_receptionist_challenge.py"
    tree = ast.parse(script.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
               and n.name == "HriReceptionistChallengeNode")
    methods = {"step", "handle_key", "_scan_callback", "_ready_banner",
               "_prepare_door_arrival", "_step_doorbell_wait", "_doorbell_audio_callback",
               "clear_temporary_memory", "_face_worker_loop"}
    cls.bases = []
    cls.body = [n for n in cls.body if isinstance(n, ast.Assign)
                or isinstance(n, ast.FunctionDef) and n.name in methods]
    speech = next(n for n in tree.body if isinstance(n, ast.FunctionDef)
                  and n.name == "is_start_speech")
    voice_tree = ast.parse((PACKAGE / "base_athome_perception" / "voice_client.py").read_text())
    voice = next(n for n in voice_tree.body if isinstance(n, ast.ClassDef)
                 and n.name == "VoiceResult")
    module = ast.Module(body=[ast.ImportFrom(module="__future__", names=[
        ast.alias(name="annotations")], level=0), voice, speech, cls], type_ignores=[])
    namespace = dict(globals(), __name__=__name__)
    exec(compile(ast.fix_missing_locations(module), str(script), "exec"), namespace)
    return SimpleNamespace(**namespace)
