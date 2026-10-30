"""Lectura y edicion acotada de `xacro:property` en properties.xacro.

La edicion NO reescribe el XML: localiza el span exacto del atributo `value`
de cada propiedad y reemplaza solo ese texto. Comentarios, espaciado, includes y
formulas `${...}` quedan byte a byte. Tras editar se re-parsea y se verifica que
solo cambiaron las propiedades pedidas.
"""
from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass

XACRO_NS = "http://www.ros.org/wiki/xacro"
_PROP_TAG = f"{{{XACRO_NS}}}property"

# Elemento `<xacro:property .../>` autocerrado. Se valida contra el parser XML:
# si el numero de coincidencias no coincide, se rechaza la edicion.
_ELEMENT_RE = re.compile(r"<xacro:property\b[^<>]*?/>", re.S)
_ATTR_RE = re.compile(r"""\s(name|value)\s*=\s*"([^"]*)\"""")
_NUMBER_RE = re.compile(r"^[+-]?(\d+(\.\d*)?|\.\d+)([eE][+-]?\d+)?$")


class XacroPropsError(ValueError):
    pass


@dataclass(frozen=True)
class Prop:
    name: str
    raw: str             # texto literal del atributo value
    kind: str            # "numeric" | "derived" | "other"
    value_span: tuple[int, int]  # offsets del texto de value dentro del archivo

    @property
    def number(self) -> float | None:
        return float(self.raw) if self.kind == "numeric" else None


def classify(raw: str) -> str:
    if "${" in raw:
        return "derived"
    if _NUMBER_RE.match(raw.strip()):
        return "numeric"
    return "other"


def parse_props(text: str) -> dict[str, Prop]:
    """Devuelve las propiedades en orden de aparicion."""
    try:
        root = ET.fromstring(text.encode("utf-8"))
    except ET.ParseError as exc:
        raise XacroPropsError(f"properties.xacro no es XML valido: {exc}") from exc
    xml_props = [(e.get("name"), e.get("value")) for e in root.iter(_PROP_TAG)]

    props: dict[str, Prop] = {}
    for m in _ELEMENT_RE.finditer(text):
        attrs = {}
        for a in _ATTR_RE.finditer(m.group(0)):
            attrs[a.group(1)] = (a.group(2), m.start() + a.start(2), m.start() + a.end(2))
        if "name" not in attrs or "value" not in attrs:
            raise XacroPropsError(f"xacro:property sin name/value en offset {m.start()}")
        name = attrs["name"][0]
        raw, start, end = attrs["value"]
        if name in props:
            raise XacroPropsError(f"propiedad duplicada: {name}")
        props[name] = Prop(name, raw, classify(raw), (start, end))

    scanned = [(p.name, p.raw) for p in props.values()]
    if scanned != xml_props:
        raise XacroPropsError(
            "El escaneo textual no coincide con el parser XML; formato no soportado para edicion segura.")
    return props


def format_number(value: float) -> str:
    return repr(round(float(value), 6))


def apply_values(text: str, values: dict[str, float]) -> str:
    """Aplica valores ya validados. Solo se pueden editar propiedades numericas."""
    props = parse_props(text)
    edits = []
    for name, value in values.items():
        prop = props.get(name)
        if prop is None:
            raise XacroPropsError(f"propiedad inexistente: {name}")
        if prop.kind != "numeric":
            raise XacroPropsError(f"propiedad no numerica (derivada o texto): {name}")
        if not math.isfinite(value):
            raise XacroPropsError(f"valor no finito para {name}")
        if math.isclose(prop.number, value, rel_tol=0, abs_tol=1e-9):
            continue  # sin cambio real: conservar el texto original (p. ej. "1.30")
        edits.append((prop.value_span, format_number(value)))

    out = text
    for (start, end), new in sorted(edits, reverse=True):
        out = out[:start] + new + out[end:]

    # Verificacion posterior: solo cambiaron las propiedades pedidas.
    after = parse_props(out)
    if list(after) != list(props):
        raise XacroPropsError("la edicion altero el conjunto de propiedades")
    for name, prop in props.items():
        if name in values:
            if not math.isclose(float(after[name].raw), float(format_number(values[name]))):
                raise XacroPropsError(f"verificacion fallida para {name}")
        elif after[name].raw != prop.raw:
            raise XacroPropsError(f"la edicion altero {name}")
    return out
