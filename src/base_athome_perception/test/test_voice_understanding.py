"""Interpretacion de lo que Whisper transcribe: nombres, bebidas y si/no.

Las frases salen de transcripciones reales del robot (logs de la Jetson y asr_benchmark, 2026-11-03).
"""
import pytest

from base_athome_perception.guest_dialog import extract_name_and_drink, parse_yes_no
from base_athome_perception.guest_vocabulary import GuestVocabulary, phonetic_key


@pytest.fixture(scope="module")
def vocab():
    return GuestVocabulary()  # listas oficiales de tools/gpsr_parser/data (o las incorporadas)


@pytest.mark.parametrize("heard,name,drink", [
    ("My name is Robin. My favorite drink is ice tea.", "Robin", "Iced Tea"),          # real
    ("My name is Robin, and my favorite drink is ice cream.", "Robin", "Iced Tea"),    # real (luego se confirma)
    ("My name is Adele and my favorite drink is Orange Juice.", "Adel", "Orange Juice"),  # real (distil)
    ("My name is Jewels and my favorite drink is a coke", "Jules", "Cola"),
    ("my name is rob in and i like milk", "Robin", "Milk"),
    ("Hello my name is Axle, I'd like red wine", "Axel", "Red Wine"),
    ("My name is Simon and my favorite drink is tropical juice", "Simone", "Tropical Juice"),
    ("Hi, I am Jules. I would like a Cola, please.", "Jules", "Cola"),                 # real
    ("My name is Morgan and my favorite drink is Juice Pack.", "Morgan", "Juice Pack"),  # real
    ("hi, my name is Morgen and I like iced tea", "Morgan", "Iced Tea"),
])
def test_names_and_drinks_from_real_transcripts(vocab, heard, name, drink):
    assert extract_name_and_drink(heard, vocab) == (name, drink)


def test_partial_answers_keep_what_was_understood(vocab):
    assert extract_name_and_drink("My favorite drink is orange juice.", vocab) == (None, "Orange Juice")
    n, d = extract_name_and_drink("My name is Morgan and my favorite drink", vocab)
    assert n == "Morgan" and d is None


def test_common_words_are_not_names(vocab):
    assert vocab.match_name("drink") is None
    assert vocab.match_name("is") is None
    assert vocab.find_name_in("I would like some water please") is None


def test_phonetic_key():
    assert phonetic_key("Jewels") == phonetic_key("Jules")
    assert phonetic_key("Axle") == phonetic_key("Axel")
    assert phonetic_key("Charley") == phonetic_key("Charlie")


@pytest.mark.parametrize("heard,ans", [
    ("Yes.", True), ("No.", False), ("Know that is wrong.", False),  # "No, that is wrong." mal transcrito (real)
    ("Yes, that is correct.", True), ("No, that is wrong.", False), ("That's right", True),
    ("That is not correct", False), ("Yeah sure", True), ("Nope", False), ("Correct", True),
    ("hmm", None), ("", None), (None, None),
])
def test_yes_no(heard, ans):
    assert parse_yes_no(heard) is ans


@pytest.mark.parametrize('drink', ['Apple Juice', 'Mineral Water', 'Sparkling Water',
                                  'Green Tea', 'Black Tea', 'Red Wine', 'White Wine'])
def test_specific_known_drink_survives_shorter_fallback_vocabulary(drink):
    fallback = GuestVocabulary(data_dirs=['/nonexistent'])
    assert extract_name_and_drink(f'My name is Morgen and I like {drink}', fallback) == ('Morgan', drink)
