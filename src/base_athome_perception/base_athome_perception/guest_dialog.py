#!/usr/bin/env python3
"""Dialogo con invitados: extraer nombre y bebida de lo que dice la persona (ingles/espanol).

Compartido por el receptionist y el secuenciador de misiones.
"""
import re
from typing import Optional, Tuple

COMMON_DRINKS = [
    'coca cola', 'coke', 'pepsi', 'fanta', 'sprite', 'water', 'mineral water',
    'sparkling water', 'beer', 'wine', 'red wine', 'white wine', 'coffee',
    'tea', 'iced tea', 'green tea', 'black tea', 'milk', 'orange juice',
    'apple juice', 'lemonade', 'juice', 'soda', 'cider', 'hot chocolate',
    'agua', 'cafe', 'cerveza', 'vino', 'jugo', 'leche', 'te'
]

DRINK_TRANSLATIONS = {
    'agua': 'Water',
    'cafe': 'Coffee',
    'cerveza': 'Beer',
    'vino': 'Wine',
    'jugo': 'Juice',
    'leche': 'Milk',
    'te': 'Tea',
    'coca cola': 'Coke',
}

STOP_WORDS = {
    'robot', 'please', 'hello', 'hi', 'yes', 'no', 'drink', 'name', 'favorite',
    'favourite', 'the', 'and', 'my', 'is', 'like', 'i', 'am', 'call', 'me', 'y',
    'por', 'favor', 'hola', 'mi', 'nombre', 'es', 'bebida', 'favorita', 'un', 'una',
    'some', 'would', 'prefer', 'want', 'gusta', 'tomar', 'followed', 'by'
}


def extract_name_and_drink(text: str, vocab=None) -> Tuple[Optional[str], Optional[str]]:
    """Extrae (nombre, bebida) a partir de una frase hablada en inglés o español.

    Con `vocab` (GuestVocabulary) se priorizan los nombres/bebidas oficiales de la
    competencia y se corrigen errores del ASR por similitud ("Morgen" -> "Morgan").
    """
    name, drink = _extract_raw(text)
    if vocab is not None:
        official = vocab.find_name_in(text)
        if official:
            name = official
        elif name:
            name = vocab.match_name(name) or name
        drink = vocab.match_drink(text) or drink
    return name, drink


def _extract_raw(text: str) -> Tuple[Optional[str], Optional[str]]:
    text_clean = text.strip()
    name = None
    drink = None

    text_lower = text_clean.lower()
    for d in sorted(COMMON_DRINKS, key=len, reverse=True):
        if re.search(r'\b' + re.escape(d) + r'\b', text_lower):
            drink = DRINK_TRANSLATIONS.get(d, d.title())
            break

    if not drink:
        drink_patterns = [
            r'(?:favorite drink is|favourite drink is|drink is|i like|i love|i prefer|i want|have a|some|drink|bebida favorita es|me gusta el|me gusta la|tomar|beber)\s+([a-zA-Z\s]+?)(?:$|\.|\band\b|\by\b)',
        ]
        for pat in drink_patterns:
            m = re.search(pat, text_lower)
            if m:
                cand = m.group(1).strip()
                cand_words = [w for w in cand.split() if w not in STOP_WORDS]
                if cand_words:
                    d_raw = ' '.join(cand_words)
                    drink = DRINK_TRANSLATIONS.get(d_raw, d_raw.title())
                    break

    name_patterns = [
        r'(?:my name is|i am|i\'m|call me|this is|they call me)\s+([A-Za-z]+)',
        r'(?:me llamo|mi nombre es|yo soy|soy)\s+([A-Za-z]+)',
    ]
    for pat in name_patterns:
        m = re.search(pat, text_clean, re.IGNORECASE)
        if m:
            cand = m.group(1).strip().title()
            if cand.lower() not in STOP_WORDS:
                name = cand
                break

    if not name:
        rem = text_clean
        if drink:
            rem = re.sub(re.escape(drink), '', rem, flags=re.IGNORECASE)
            for k, v in DRINK_TRANSLATIONS.items():
                if v == drink:
                    rem = re.sub(re.escape(k), '', rem, flags=re.IGNORECASE)
        rem = re.sub(
            r'^(?:hello|hi|please|i am|i\'m|my name is|and|i like|hola|me llamo|mi nombre es)\s*',
            '',
            rem,
            flags=re.IGNORECASE,
        ).strip()
        words = [w.strip('.!?,') for w in rem.split() if w.strip('.!?,')]
        valid_words = [w for w in words if w.lower() not in STOP_WORDS and len(w) >= 3 and w.isalpha()]
        if valid_words:
            name = valid_words[0].title()

    return name, drink


# Respuestas si/no tal como las transcribe Whisper (incluye errores tipicos: "No." -> "Know")
YES_WORDS = {'yes', 'yeah', 'yep', 'yup', 'yah', 'ya', 'yea', 'correct', 'right', 'sure', 'exactly',
             'affirmative', 'absolutely', 'definitely', 'true', 'okay', 'ok', 'perfect', 'si', 'sí', 'claro'}
NO_WORDS = {'no', 'nope', 'nah', 'not', 'wrong', 'incorrect', 'negative', 'nein', "don't", 'isnt', "isn't",
            'mistake', 'never'}
NO_AT_START = {'know', 'now', 'noh', 'nor'}


def parse_yes_no(text: Optional[str]) -> Optional[bool]:
    """True = si, False = no, None = no se entiende. Gana la primera palabra si/no de la frase
    ("No, that is right" -> False), salvo frases hechas ("that's right", "not really" ...)."""
    if not text:
        return None
    t = text.lower().strip()
    if re.search(r"\b(that'?s|that is|it'?s|it is) (not )?(right|correct|wrong)\b", t):
        neg = re.search(r"\b(not (right|correct)|wrong)\b", t)
        lead = re.match(r"\W*(yes|yeah|yep|no|nope)\b", t)
        if lead:
            return lead.group(1) in ('yes', 'yeah', 'yep')
        return not neg
    words = re.findall(r"[a-záéíóú']+", t)
    if not words:
        return None
    if words[0] in NO_AT_START and len(words) <= 4:
        return False
    for w in words:
        if w in YES_WORDS:
            return True
        if w in NO_WORDS:
            return False
    return None
