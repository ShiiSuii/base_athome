"""Loads dynamic competition knowledge (names, locations, rooms, objects, categories)

Reads from markdown files in a data directory (compatible with CompetitionTemplate).
Does not hardcode competition items.
"""

import os
import re
import warnings
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple


@dataclass
class CompetitionKnowledge:
    names: List[str]
    locations: List[str]
    placement_locations: List[str]
    rooms: List[str]
    objects: List[str]
    categories_singular: List[str]
    categories_plural: List[str]
    objects_by_category: Dict[str, List[str]]
    storages: Dict[str, List[str]]

    @property
    def all_locations(self) -> List[str]:
        # Return unique combined locations and rooms sorted by length descending
        locs = list(set(self.locations + self.placement_locations + self.rooms))
        locs.sort(key=lambda s: len(s), reverse=True)
        return locs


def _read_file(path: Path) -> str:
    if not path.is_file():
        return ""
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def load_knowledge(data_dir: Optional[str] = None) -> CompetitionKnowledge:
    """Load competition entities from data_dir."""
    if data_dir is None:
        # Default to tools/gpsr_parser/data if available
        base = Path(__file__).resolve().parent.parent / "data"
        if base.is_dir():
            data_dir = str(base)

    if not data_dir or not os.path.isdir(data_dir):
        # Return minimal fallback if directory not found
        return CompetitionKnowledge(
            names=[],
            locations=[],
            placement_locations=[],
            rooms=[],
            objects=[],
            categories_singular=[],
            categories_plural=[],
            objects_by_category={},
            storages={},
        )

    data_path = Path(data_dir)

    # 1. Names
    names_file = data_path / "names" / "names.md"
    names_content = _read_file(names_file)
    parsed_names = re.findall(r"\|\s*([A-Za-z]+)\s*\|", names_content, re.DOTALL)
    names = [n.strip() for n in parsed_names if n.strip() and n.strip() != "Names"]

    # 2. Locations and Placement Locations
    locs_file = data_path / "maps" / "location_names.md"
    locs_content = _read_file(locs_file)
    locations = []
    placement_locations = []
    storages: Dict[str, List[str]] = {}
    for line in locs_content.splitlines():
        line = line.strip()
        if not line.startswith("|") or line.startswith("| ---") or "Number" in line:
            continue
        parts = [p.strip() for p in line.split("|") if p.strip()]
        if not parts:
            continue
        if parts[0].isdigit():
            name = parts[1] if len(parts) > 1 else ""
            cat = parts[2] if len(parts) > 2 else ""
        else:
            name = parts[0]
            cat = parts[1] if len(parts) > 1 else ""
        is_plcmt = "(p)" in name
        clean_name = name.replace("(p)", "").strip()
        if clean_name:
            locations.append(clean_name)
            if is_plcmt:
                placement_locations.append(clean_name)
            if cat:
                storages.setdefault(cat, []).append(clean_name)

    # 3. Rooms
    rooms_file = data_path / "maps" / "room_names.md"
    rooms_content = _read_file(rooms_file)
    rooms = []
    for line in rooms_content.splitlines():
        line = line.strip()
        if line.startswith("|") and not line.startswith("| ---"):
            parts = [p.strip() for p in line.split("|") if p.strip()]
            for p in parts:
                if p.lower() not in ("name", "names", "room", "rooms"):
                    rooms.append(p)

    # 4. Objects and Categories
    objs_file = data_path / "objects" / "objects.md"
    objs_content = _read_file(objs_file)
    class_blocks = re.split(r"# Class\s+", objs_content)
    class_blocks = [b.strip() for b in class_blocks if b.strip()]

    objects_by_category: Dict[str, List[str]] = {}
    categories_singular = []
    categories_plural = []
    objects = []

    for block in class_blocks:
        header_match = re.match(r"([\w_]+)\s*\(([\w_]+)\)", block)
        if not header_match:
            continue
        plural = header_match.group(1).replace("_", " ").strip()
        singular = header_match.group(2).replace("_", " ").strip()
        if plural and plural not in categories_plural:
            categories_plural.append(plural)
        if singular and singular not in categories_singular:
            categories_singular.append(singular)

        block_objs = re.findall(r"\|\s*([\w_]+)\s*\|", block)
        cleaned_objs = [
            o.replace("_", " ").strip()
            for o in block_objs
            if o.strip() and o.strip() != "Objectname"
        ]
        objects_by_category[plural] = cleaned_objs
        objects.extend(cleaned_objs)

    # Remove duplicates preserving order
    names = list(dict.fromkeys(names))
    locations = list(dict.fromkeys(locations))
    placement_locations = list(dict.fromkeys(placement_locations))
    rooms = list(dict.fromkeys(rooms))
    objects = list(dict.fromkeys(objects))

    return CompetitionKnowledge(
        names=names,
        locations=locations,
        placement_locations=placement_locations,
        rooms=rooms,
        objects=objects,
        categories_singular=categories_singular,
        categories_plural=categories_plural,
        objects_by_category=objects_by_category,
        storages=storages,
    )
