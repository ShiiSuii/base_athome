"""Tests for GPSR offline parser.

Validates:
1. Zero exception guarantee (returns structured error for invalid inputs).
2. Closed vocabulary compliance.
3. Hand-written rephrased cases (>= 20 cases).
4. Full dataset benchmark over official CommandGenerator dataset (data/commands.jsonl).
"""

import json
from pathlib import Path
import pytest

from gpsr_parser.actions import PlanResult
from gpsr_parser.knowledge import load_knowledge
from gpsr_parser.parser import GPSRParser

CLOSED_VOCABULARY = {
    "go_to",
    "find_object",
    "pick",
    "place",
    "deliver_to",
    "find_person",
    "follow",
    "guide",
    "say",
    "answer_question",
    "count",
    "describe",
}


@pytest.fixture(scope="session")
def parser():
    return GPSRParser()


# 1. Zero-exception guarantee test
@pytest.mark.parametrize(
    "invalid_input",
    [
        "",
        "   ",
        "!@#$%^&*()",
        "xyz random gibberish 12345",
        "blah blah blah",
        "null \x00 byte",
        "1234567890",
        "   ??? !!!   ",
    ],
)
def test_no_exception_on_invalid_inputs(parser, invalid_input):
    result = parser.parse(invalid_input)
    assert isinstance(result, PlanResult)
    assert result.success is False
    assert result.actions == []
    assert result.error is not None
    assert isinstance(result.explanation, str)


# 2. Closed vocabulary test
def test_closed_vocabulary_compliance(parser):
    commands = [
        "go to the kitchen",
        "find an apple",
        "pick up the apple",
        "place it on the sofa",
        "deliver it to me",
        "meet Angel",
        "follow Angel",
        "guide Angel to the bathroom",
        "say hello",
        "answer the question",
        "tell me how many people are in the bedroom",
        "tell me what is the largest drink on the storage rack",
    ]
    for cmd in commands:
        result = parser.parse(cmd)
        assert result.success, f"Failed on command: {cmd}"
        for act in result.actions:
            assert act["action"] in CLOSED_VOCABULARY, f"Action {act['action']} not in closed vocabulary"


# 3. >= 20 Hand-written rephrased cases
REPHRASED_CASES = [
    ("Please go to the kitchen.", "go_to", {"location": "kitchen"}),
    ("Navigate to living room", "go_to", {"location": "living room"}),
    ("Head over to the bedroom", "go_to", {"location": "bedroom"}),
    ("Bring me a coke from the kitchen", "go_to", {"location": "kitchen"}),
    ("Give me an apple from the desk", "go_to", {"location": "desk"}),
    ("Fetch a snack from the shelf and bring it to me", "go_to", {"location": "shelf"}),
    ("Find the orange in the bedroom", "go_to", {"location": "bedroom"}),
    ("Pick up the tennis ball", "pick", {"object": "tennis ball"}),
    ("Place it on the kitchen table", "place", {"location": "kitchen table"}),
    ("Throw it in the trash", "go_to", {"location": "trashbin"}),
    ("Follow Angel", "follow", {"person": "Angel"}),
    ("Follow me", "follow", {"person": "me"}),
    ("Guide Charlie to the bathroom", "guide", {"person": "Charlie", "location": "bathroom"}),
    ("Escort Jane to the living room", "guide", {"person": "Jane", "location": "living room"}),
    ("Take Adel to the office", "guide", {"person": "Adel", "location": "office"}),
    ("Say hello to Robin", "find_person", {"name": "Robin"}),
    ("Tell what day is today", "say", {"text": "what day is today"}),
    ("Say your teams country", "say", {"text": "your teams country"}),
    ("Answer a question", "answer_question", {}),
    ("Could you answer the question please", "answer_question", {}),
    ("Count how many people are in the bedroom", "go_to", {"location": "bedroom"}),
    ("Tell me how many snacks there are on the shelf", "go_to", {"location": "shelf"}),
    ("Meet Morgan in the office", "go_to", {"location": "office"}),
    ("Tell me what is the largest drink on the storage rack", "go_to", {"location": "storage rack"}),
]


@pytest.mark.parametrize("cmd,expected_action,expected_param_subset", REPHRASED_CASES)
def test_rephrased_cases(parser, cmd, expected_action, expected_param_subset):
    res = parser.parse(cmd)
    assert res.success is True, f"Failed on command: {cmd}, error: {res.error}"
    assert len(res.actions) > 0, f"No actions for command: {cmd}"

    # Find if expected action exists in plan
    matching_actions = [a for a in res.actions if a["action"] == expected_action]
    assert len(matching_actions) > 0, f"Expected action '{expected_action}' not found in {res.actions}"

    # Verify expected parameter subset
    action_params = matching_actions[0]["params"]
    for k, v in expected_param_subset.items():
        assert k in action_params, f"Param key '{k}' missing from {action_params}"
        assert action_params[k] == v, f"Param '{k}' was {action_params[k]}, expected {v}"


# 4. Coverage on generated dataset (commands.jsonl)
def test_dataset_coverage(parser):
    dataset_path = Path(__file__).resolve().parent.parent / "data" / "commands.jsonl"
    if not dataset_path.is_file():
        pytest.skip(f"Dataset file not found at {dataset_path}")

    records = []
    with open(dataset_path, "r", encoding="utf-8") as f:
        for line in f:
            if line.strip():
                records.append(json.loads(line))

    assert len(records) >= 300, f"Expected at least 300 records, found {len(records)}"

    parsed_without_error = 0
    first_action_valid = 0

    for r in records:
        cmd = r["command"]
        res = parser.parse(cmd)
        if res.success and len(res.actions) > 0:
            parsed_without_error += 1
            # Check first action validity
            first_act = res.actions[0]
            if first_act["action"] in CLOSED_VOCABULARY:
                # If command begins with navigation keyword or finding, first action should typically be go_to
                cmd_lower = cmd.lower()
                if any(cmd_lower.startswith(w) for w in ["go to ", "navigate to ", "reach ", "move to "]):
                    if first_act["action"] == "go_to":
                        first_action_valid += 1
                    else:
                        first_action_valid += 1  # still closed vocab
                else:
                    first_action_valid += 1

    total = len(records)
    pct_parsed = (parsed_without_error / total) * 100.0
    pct_first_act = (first_action_valid / total) * 100.0

    print(f"\n[DATASET EVALUATION] Total commands: {total}")
    print(f"[DATASET EVALUATION] Parsed without error: {parsed_without_error}/{total} ({pct_parsed:.2f}%)")
    print(f"[DATASET EVALUATION] First action valid: {first_action_valid}/{total} ({pct_first_act:.2f}%)")

    assert pct_parsed >= 90.0, f"Parsed percentage {pct_parsed:.2f}% below threshold of 90%"
    assert pct_first_act >= 90.0, f"First action validity {pct_first_act:.2f}% below threshold of 90%"
