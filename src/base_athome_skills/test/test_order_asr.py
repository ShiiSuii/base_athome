"""Offline ASR regressions: real Whisper errors and ordinary-word negatives."""

import pytest

from base_athome_skills.mission.order_asr import (
    match_menu_items,
    normalize_role,
    parse_yes_no,
)


MENU = ["Red Bull", "peach", "coke", "water", "coffee", "tea", "orange juice", "beer"]


@pytest.mark.parametrize(("text", "expected"), [
    ("red bowl", ["Red Bull"]),
    ("a red bowl please", ["Red Bull"]),
    ("peach and pork", ["peach", "coke"]),
    ("my order is peach and pork", ["peach", "coke"]),
    ("cock", ["coke"]),
    ("cocks", ["coke"]),
    ("my order is a cock and a red bowl", ["coke", "Red Bull"]),
    ("My order is the Red Bull, a peach and a coke.", ["Red Bull", "peach", "coke"]),
    ("PEACH, COKE!", ["peach", "coke"]),
    ("peaches and cokes", ["peach", "coke"]),
    ("two red bulls and coffees", ["Red Bull", "coffee"]),
    ("red bowls", ["Red Bull"]),
    ("orange juices and beers", ["orange juice", "beer"]),
    ("the water and some tea", ["water", "tea"]),
    ("cofee", ["coffee"]),
    ("coffie", ["coffee"]),
    ("watter", ["water"]),
    ("water, water and coke", ["water", "coke"]),
    ("coke then peach", ["coke", "peach"]),
    ("a pizza and water", ["water"]),
    ("tea-party", ["tea"]),
    ("", []),
    ("...", []),
    ("my order is", []),
    ("hello robot please bring my order here", []),
    ("I would like something please", []),
    ("cook the cake", []),
    ("the beach is here", []),
    ("a bear and a deer", []),
    ("the waiter is better", []),
    ("come here with your coat", []),
    ("cold weather later", []),
    ("read the bowl", []),
    ("a bull", []),
    ("pork", []),
    ("I want pork please", []),
    ("cocktail and beerhouse", []),
    ("watermelon and teaspoon", []),
    ("speech and teaching", []),
    ("yes correct robot no", []),
])
def test_menu_transcripts(text, expected):
    assert match_menu_items(text, MENU) == expected


@pytest.mark.parametrize(("text", "menu", "expected"), [
    ("cock red bowl peach and pork", [], []),
    ("cock red bowl peach and pork", ["tea"], []),
    ("pork", ["pork", "coke"], ["pork"]),
    ("peach and pork", ["peach", "pork", "coke"], ["peach", "pork"]),
    ("red bowl", ["red bowl", "Red Bull"], ["red bowl"]),
    ("cock", ["cock", "coke"], ["cock"]),
    ("orange juice", ["orange", "juice", "orange juice"], ["orange juice"]),
    ("coke", ["Coke", "Coke", ""], ["Coke"]),
    ("blueberries", ["blueberry"], ["blueberry"]),
    ("sandwiches", ["sandwich"], ["sandwich"]),
    ("milks", ["milk"], ["milk"]),
    ("a tea", ["pie", "tea"], ["tea"]),
    ("the", ["tea"], []),
    ("cofee", ["coffee", "coofee"], []),
    ("peaches", ["peach", "peaches"], ["peaches"]),
    ("my order is water", ["order", "water"], ["water"]),
    ("water", ["", "  ", "water"], ["water"]),
])
def test_menu_scope_and_precedence(text, menu, expected):
    assert match_menu_items(text, menu) == expected


@pytest.mark.parametrize(("text", "expected"), [
    ("robot yes", True),
    ("yes", True),
    ("yeah", True),
    ("correct", True),
    ("robot no", False),
    ("no", False),
    ("nope", False),
    ("robot yet", True),
    ("roberto yes", True),
    ("roberto no", False),
    ("Robot, YES!", True),
    (" robot yeah ", True),
    ("robot correct", True),
    ("yes please", True),
    ("no thanks", False),
    ("robot no thank you", False),
    ("", None),
    ("robot", None),
    ("yet", None),
    ("not yet", None),
    ("robot not yet", None),
    ("yes no", None),
    ("no yes", None),
    ("yes but no", None),
    ("incorrect", None),
    ("yesterday", None),
    ("nobody", None),
    ("I said yes yesterday", None),
    ("robot order water", None),
    ("not correct", None),
    ("robot yes no", None),
    ("roberto yesterday", None),
])
def test_confirmations(text, expected):
    assert parse_yes_no(text) is expected


@pytest.mark.parametrize(("text", "expected"), [
    ("barman", "barman"),
    ("borman", "barman"),
    ("bman", "barman"),
    ("bar man", "barman"),
    ("  BAR   MAN! ", "barman"),
    ("Borman.", "barman"),
    ("customer", "customer"),
    ("Batman", "batman"),
    ("bar", "bar"),
    ("man", "man"),
    ("the barman is here", "the barman is here"),
    ("", ""),
])
def test_roles(text, expected):
    assert normalize_role(text) == expected
