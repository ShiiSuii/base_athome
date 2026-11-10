"""Conservative, offline helpers for English restaurant ASR transcripts.

These functions extract words, not intent: the caller must still confirm an order.
Only the supplied menu defines valid items; no ROS or third-party imports are used.
"""

from collections.abc import Sequence
from difflib import SequenceMatcher
import re
import unicodedata


# Never fuzz ordinary dialogue words or common near-homophones of menu items.
_COMMON_WORDS = frozenset("""
    a an the and or but i me my you your we our it is are was be been am
    order orders want wants would could can have has please thanks thank
    hello hi robot roberto yes no yeah correct not nothing something anything
    some one two three for to of in on at with this that then there here
    bring give get take tell say stop cancel don't do ready done
    cook cooks cake cakes coat code cold come beach beaches bear bears
    dear deer better weather waiter later read bowl bull reach speech teach
""".split())


def _tokens(text: str) -> tuple[str, ...]:
    folded = unicodedata.normalize("NFKD", text.casefold())
    folded = "".join(c for c in folded if not unicodedata.combining(c))
    return tuple(re.findall(r"[a-z0-9]+(?:'[a-z]+)?", folded))


def _phonetic(word: str) -> str:
    """Small consonant key, used only alongside spelling and word guards."""
    word = word.replace("ph", "f").replace("ck", "k")
    word = word.translate(str.maketrans({"c": "k", "q": "k", "z": "s"}))
    word = re.sub(r"[aeiouy]", "", word)
    return re.sub(r"(.)\1+", r"\1", word)


def _plural(word: str) -> str:
    if word.endswith("y") and len(word) > 1 and word[-2] not in "aeiou":
        return word[:-1] + "ies"
    if word.endswith(("s", "x", "z", "ch", "sh")):
        return word + "es"
    return word + "s"


def _word_score(heard: str, expected: str) -> float:
    if heard == expected:
        return 1.0
    if heard == _plural(expected):
        return 0.99
    if heard in _COMMON_WORDS or expected in _COMMON_WORDS:
        return 0.0
    # Short words (tea, pie, etc.) require exact matches or explicit aliases.
    if min(len(heard), len(expected)) < 4:
        return 0.0
    ratio = SequenceMatcher(None, heard, expected, autojunk=False).ratio()
    if _phonetic(heard) == _phonetic(expected) and ratio >= 0.72:
        return ratio
    # Permit a small spelling error in longer names, with the same initial.
    if min(len(heard), len(expected)) >= 6 and heard[0] == expected[0] and ratio >= 0.88:
        return ratio
    return 0.0


def match_menu_items(text: str, menu: Sequence[str]) -> list[str]:
    """Return canonical menu names once each, in transcript order.

    Match whole tokens, prefer exact names and longer phrases, and discard tied
    fuzzy guesses. Articles and order prefixes may surround items. Known Whisper
    confusions are menu-scoped; ``pork`` means coke only in ``peach and pork``
    and only if pork itself is absent from the menu. This is deliberately not a
    general phonetic dictionary (e.g. ``cook`` must not become ``coke``).
    """
    heard = _tokens(text)
    entries = [(name, _tokens(name)) for name in menu if _tokens(name)]
    result: list[str] = []
    index = 0
    while index < len(heard):
        if heard[index:index + 3] == ("my", "order", "is"):
            index += 3
            continue
        candidates = []
        for name, words in entries:
            size = len(words)
            chunk = heard[index:index + size]
            if len(chunk) != size:
                continue
            score = min(_word_score(a, b) for a, b in zip(chunk, words))
            if words == ("red", "bull") and chunk in {
                ("red", "bowl"), ("red", "bowls"),
            }:
                score = 0.96
            if words == ("coke",) and chunk in {("cock",), ("cocks",)}:
                score = 0.96
            if (words == ("coke",) and chunk == ("pork",)
                    and index >= 2 and heard[index - 2:index] == ("peach", "and")
                    and not any(w == ("pork",) for _, w in entries)):
                score = 0.96
            if score:
                candidates.append((score, size, name))
        if not candidates:
            index += 1
            continue
        candidates.sort(key=lambda c: (c[0], c[1]), reverse=True)
        score, size, name = candidates[0]
        # Duplicate menu entries are harmless, competing fuzzy names are not.
        ambiguous = any(other != name and other_score == score and other_size == size
                        for other_score, other_size, other in candidates[1:])
        if not ambiguous and name not in result:
            result.append(name)
        index += size
    return result


def parse_yes_no(text: str) -> bool | None:
    """Parse a complete confirmation; unrelated or conflicting speech is unknown.

    ``yet`` is accepted only after a robot wake word, to avoid interpreting an
    ordinary time adverb as consent. No fuzzy matching is applied to consent.
    """
    words = _tokens(text)
    addressed = bool(words and words[0] in {"robot", "roberto"})
    if addressed:
        words = words[1:]
    if words[-2:] == ("thank", "you"):
        words = words[:-2]
    elif words and words[-1] in {"please", "thanks"}:
        words = words[:-1]
    if words in {("yes",), ("yeah",), ("correct",)}:
        return True
    if addressed and words == ("yet",):
        return True
    if words in {("no",), ("nope",)}:
        return False
    return None


def normalize_role(text: str) -> str:
    """Normalize a role label; preserve unknown labels as lowercase words.

    This consumes a label, not a sentence, so ``the barman is here`` is not
    interpreted as identifying a role.
    """
    role = " ".join(_tokens(text))
    if role in {"barman", "borman", "bman", "bar man"}:
        return "barman"
    return role
