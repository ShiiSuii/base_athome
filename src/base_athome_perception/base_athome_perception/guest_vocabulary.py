#!/usr/bin/env python3
"""Vocabulario de la competencia para escuchar mejor a los invitados.

Las listas oficiales de nombres y bebidas (formato RoboCupAtHome/CompetitionTemplate:
names/names.md y objects/objects.md) se usan para:
  1. armar el initial_prompt de Whisper (sesga el ASR hacia esas palabras);
  2. corregir lo transcrito con coincidencia aproximada ("Sofie" -> "Sophia").
Si los archivos no estan, se usa una lista minima incorporada.
"""
import difflib
import os
import re
from typing import Iterable, List, Optional

DEFAULT_DATA_DIRS = [
    os.path.expanduser("~/base_athome_ws/tools/gpsr_parser/data"),
    os.path.expanduser("~/.local/share/base_athome/competition_data"),
]
FALLBACK_NAMES = ["Adel", "Angel", "Axel", "Charlie", "Jane", "Jules", "Morgan", "Paris", "Robin", "Simone"]
FALLBACK_DRINKS = ["Water", "Coffee", "Tea", "Juice", "Milk", "Cola", "Lemonade", "Orange Juice", "Iced Tea"]


def _table_cells(path: str) -> List[str]:
    out = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            m = re.match(r"^\|\s*([A-Za-z][A-Za-z _'-]*?)\s*\|", line)
            if m and m.group(1).lower() not in ("names", "objectname"):
                out.append(m.group(1).strip())
    return out


def _drinks_from_objects(path: str) -> List[str]:
    drinks, in_drinks = [], False
    with open(path, encoding="utf-8") as f:
        for line in f:
            if line.startswith("# Class"):
                in_drinks = "drink" in line.lower()
                continue
            m = re.match(r"^\|\s*([a-z_]+)\s*\|", line)
            if in_drinks and m and m.group(1) != "objectname":
                drinks.append(m.group(1).replace("_", " ").title())
    return drinks


# Palabras que nunca son un nombre (aparecen al lado de "my name is" o se parecen a nombres)
NOT_NAMES = {"my", "name", "is", "and", "the", "a", "i", "im", "am", "me", "call", "this", "it", "its",
             "favorite", "favourite", "drink", "like", "hello", "hi", "hey", "yes", "no", "please", "robot",
             "nice", "to", "meet", "you", "so", "uh", "um", "well", "oh", "okay", "ok", "called", "just"}
# Variantes frecuentes de Whisper para los nombres oficiales (agregar las que se observen en logs)
NAME_ALIASES = {"adele": "adel", "adell": "adel", "adelle": "adel", "angela": "angel", "angie": "angel",
                "axle": "axel", "axell": "axel", "charley": "charlie", "charly": "charlie", "jayne": "jane",
                "jain": "jane", "jewels": "jules", "jewel": "jules", "julz": "jules", "joules": "jules",
                "morgen": "morgan", "morgann": "morgan", "parris": "paris", "robyn": "robin", "robbin": "robin",
                "robbie": "robin", "simon": "simone", "simona": "simone", "symone": "simone"}
DRINK_ALIASES = {"ice tea": "Iced Tea", "ice t": "Iced Tea", "iced t": "Iced Tea", "ice tee": "Iced Tea",
                 "icetea": "Iced Tea", "coke": "Cola", "coca cola": "Cola", "coca-cola": "Cola", "pepsi": "Cola",
                 "juice box": "Juice Pack", "juicepack": "Juice Pack", "red vine": "Red Wine",
                 "o j": "Orange Juice", "oj": "Orange Juice"}
NAME_SLOT = r"(?:name is|name's|names|i am|i'm|im|call me|this is|it's|they call me|me llamo|soy)\b"
DRINK_SLOT = r"(?:drink is|drink would be|drinks? |i like|i love|i want|i'd like|i would like|i prefer|" \
             r"give me|bring me|have some|have a|favorite is|favourite is)"
FILLER = {"my", "is", "and", "the", "a", "some", "please", "drink", "favorite", "favourite", "i", "like",
          "would", "be", "glass", "of", "cup", "bottle", "can", "uh", "um", "very", "much", "really"}


def phonetic_key(word: str) -> str:
    """Clave fonetica simple (tipo metaphone) para nombres en ingles."""
    w = re.sub(r"[^a-z]", "", word.lower())
    if not w:
        return ""
    for a, b in (("x", "ks"), ("ph", "f"), ("ck", "k"), ("sch", "sk"), ("ch", "X"), ("sh", "X"), ("qu", "kw"),
                 ("q", "k"), ("z", "s"), ("wh", "w"), ("dg", "j"), ("gh", ""), ("kn", "n"), ("y", "i")):
        w = w.replace(a, b)
    w = re.sub(r"c(?=[eiy])", "s", w).replace("c", "k")
    first, rest = w[0], re.sub(r"[aeiouhw]", "", w[1:])
    key = first + rest
    key = re.sub(r"(.)\1+", r"\1", key)
    if first in "aeiou":
        key = "a" + key[1:]
    return key


class GuestVocabulary:
    def __init__(self, data_dirs: Optional[Iterable[str]] = None):
        self.names: List[str] = []
        self.drinks: List[str] = []
        self.source = "incorporado"
        for d in (data_dirs or DEFAULT_DATA_DIRS):
            names_md = os.path.join(d, "names", "names.md")
            objects_md = os.path.join(d, "objects", "objects.md")
            try:
                if os.path.isfile(names_md):
                    self.names = _table_cells(names_md)
                if os.path.isfile(objects_md):
                    self.drinks = _drinks_from_objects(objects_md)
                if self.names:
                    self.source = d
                    break
            except OSError:
                continue
        self.names = self.names or list(FALLBACK_NAMES)
        # Bebidas comunes ademas de las oficiales: los invitados piden lo que quieren
        self.drinks = list(dict.fromkeys(self.drinks + FALLBACK_DRINKS))

    # ------------------------------------------------------------------ nombres
    def match_name(self, word: Optional[str], cutoff: float = 0.75) -> Optional[str]:
        """Nombre oficial mas parecido (texto + fonetica: "Jewels"->Jules, "Axle"->Axel, "Adele"->Adel)."""
        if not word:
            return None
        w = re.sub(r"[^a-z]", "", word.lower())
        if len(w) < 2 or w in NOT_NAMES:
            return None
        best, best_score = None, 0.0
        for n in self.names:
            ln = n.lower()
            if w == ln or NAME_ALIASES.get(w) == ln:
                return n
            score = difflib.SequenceMatcher(None, w, ln).ratio()
            if phonetic_key(w) == phonetic_key(ln) and len(w) >= 3:
                score = max(score, 0.9)
            if score > best_score:
                best, best_score = n, score
        return best if best_score >= cutoff else None

    def find_name_in(self, text: str) -> Optional[str]:
        """Nombre oficial en la frase. Primero en la posicion de nombre ("my name is X", "I'm X",
        "call me X") con tolerancia alta al ASR; si no, cualquier palabra con coincidencia estricta."""
        t = text.lower()
        for m in re.finditer(NAME_SLOT, t):
            words = re.findall(r"[a-z']+", t[m.end():])[:3]
            for i, w in enumerate(words):
                if w in NOT_NAMES:
                    continue
                cand = [w] + ([w + words[i + 1]] if i + 1 < len(words) else [])  # "rob in" -> robin
                for c in cand:
                    hit = self.match_name(c, cutoff=0.62)
                    if hit:
                        return hit
                break  # solo la primera palabra util despues de "my name is"
        for w in re.findall(r"[A-Za-z']+", text):
            if len(w) >= 3:
                m = self.match_name(w, cutoff=0.8)
                if m:
                    return m
        return None

    # ------------------------------------------------------------------ bebidas
    def match_drink(self, text: Optional[str], cutoff: float = 0.8) -> Optional[str]:
        """Bebida de la lista en la frase. Busca n-gramas de 3, 2 y 1 palabras (la mas especifica
        primero: "iced tea" antes que "tea"), alias ("ice tea", "coke") y similitud; en la posicion de
        bebida ("my favorite drink is X") acepta mas error del ASR."""
        if not text:
            return None
        t = re.sub(r"[^a-z' ]", " ", text.lower())
        lower = {d.lower(): d for d in self.drinks}
        for alias, target in DRINK_ALIASES.items():
            if re.search(r"\b" + re.escape(alias) + r"\b", t) and target.lower() in lower:
                # el alias solo gana si no hay una bebida mas larga escrita literalmente
                exact = [d for d in lower if re.search(r"\b" + re.escape(d) + r"\b", t) and len(d) > len(alias)]
                if not exact:
                    return lower[target.lower()]
        for d in sorted(lower, key=len, reverse=True):
            if re.search(r"\b" + re.escape(d) + r"\b", t):
                return lower[d]
        slot = re.search(DRINK_SLOT, t)
        regions = [(t[slot.end():], 0.66)] if slot else []
        regions.append((t, cutoff if len(t.split()) > 3 else min(cutoff, 0.75)))
        for region, cut in regions:
            words = [w for w in region.split() if w not in FILLER]
            for n in (3, 2, 1):
                best, best_score = None, 0.0
                for i in range(len(words) - n + 1):
                    gram = " ".join(words[i:i + n])
                    for d in lower:
                        if abs(len(d.split()) - n) > 1:
                            continue
                        s = difflib.SequenceMatcher(None, gram, d).ratio()
                        if s > best_score:
                            best, best_score = d, s
                if best and best_score >= cut:
                    return lower[best]
        return None

    def whisper_prompt(self) -> str:
        return ("Guests introduce themselves at a party. Names: " + ", ".join(self.names)
                + ". Drinks: " + ", ".join(self.drinks) + ". My name is Robin and my favorite drink is iced tea.")
