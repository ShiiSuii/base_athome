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

    def match_name(self, word: Optional[str], cutoff: float = 0.75) -> Optional[str]:
        if not word:
            return None
        lower = {n.lower(): n for n in self.names}
        hit = difflib.get_close_matches(word.lower(), list(lower), n=1, cutoff=cutoff)
        return lower[hit[0]] if hit else None

    def find_name_in(self, text: str) -> Optional[str]:
        """Primer nombre oficial (exacto o aproximado) presente en la frase."""
        for w in re.findall(r"[A-Za-z']+", text):
            if len(w) >= 3:
                m = self.match_name(w, cutoff=0.8)
                if m:
                    return m
        return None

    def match_drink(self, text: Optional[str], cutoff: float = 0.8) -> Optional[str]:
        if not text:
            return None
        lower = {d.lower(): d for d in self.drinks}
        t = text.lower()
        for d in sorted(lower, key=len, reverse=True):
            if re.search(r"\b" + re.escape(d) + r"\b", t):
                return lower[d]
        hit = difflib.get_close_matches(t.strip(), list(lower), n=1, cutoff=cutoff)
        return lower[hit[0]] if hit else None

    def whisper_prompt(self) -> str:
        return ("Guests introduce themselves at a party. Names: " + ", ".join(self.names)
                + ". Drinks: " + ", ".join(self.drinks) + ". My name is Robin and my favorite drink is iced tea.")
