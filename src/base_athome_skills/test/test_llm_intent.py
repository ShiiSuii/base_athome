"""Lo que la LLM propone se limita y se valida. Sin modelo: su salida se simula, tambien la mala."""
from pathlib import Path

from base_athome_skills.mission import llm_intent as li
from base_athome_skills.mission.command_parser import compile_gpsr_actions
from base_athome_skills.mission.places import load_places

MAP = Path(__file__).resolve().parents[3] / "simulation" / "base_athome_sim" / "config" / "sim_semantic_map.yaml"


def test_grammar_lists_only_known_places_and_objects():
    g = li.grammar(["kitchen", "living room"], ["cup"])
    assert '"\\"kitchen\\""' in g and '"\\"living room\\""' in g and '"\\"cup\\""' in g and "moon" not in g


def test_reply_becomes_actions():
    a = li.to_actions('[{"a": "go_to", "x": "kitchen"}, {"a": "pick", "x": "cup"}, {"a": "deliver_to_me"}]')
    assert [x["action"] for x in a] == ["go_to", "pick", "deliver_to"]


def test_empty_broken_or_unknown_reply_means_no_order():
    assert li.to_actions("[]") == []
    assert li.to_actions("nonsense") == []
    assert li.to_actions('{"a": "go_to"}') == []
    assert li.to_actions('[{"a": "go_to", "x": "kitchen"}, {"a": "self_destruct"}]') == []


def test_a_place_the_map_does_not_have_is_rejected_by_the_compiler():
    c = compile_gpsr_actions(li.to_actions('[{"a": "go_to", "x": "moon"}]'), load_places(str(MAP)))
    assert not c.ok


def test_without_server_there_is_no_answer_and_no_error():
    assert li.ask("go to the kitchen", ["kitchen"], ["cup"], url="http://127.0.0.1:9/none", timeout_s=0.5) == []
