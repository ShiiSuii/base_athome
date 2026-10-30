"""Deterministic GPSR command parser (rules/grammar, offline, zero-network).

Parses natural language GPSR instructions into structured JSON plans
with a closed action vocabulary:
- go_to(location)
- find_object(object|category)
- pick(object)
- place(location)
- deliver_to(person|location)
- find_person(name|gesture|posture|clothing)
- follow(person)
- guide(person, location)
- say(text)
- answer_question()
- count(...)
- describe(...)
"""

import re
from typing import Any, Dict, List, Optional, Tuple

from gpsr_parser.actions import Action, PlanResult
from gpsr_parser.knowledge import CompetitionKnowledge, load_knowledge

SYNONYMS_LOCATION = {
    "trash": "trashbin",
    "trash can": "trashbin",
    "garbage": "trashbin",
    "bin": "trashbin",
    "fridge": "refrigerator",
    "couch": "sofa",
    "dining table": "kitchen table",
}

SYNONYMS_OBJECT = {
    "coke": "cola",
    "coca cola": "cola",
    "water": "iced tea",
    "soda": "cola",
}

GESTURES = [
    "pointing to the left",
    "pointing to the right",
    "raising their left arm",
    "raising their right arm",
    "raising their arm",
    "waving",
]

POSTURES = [
    "standing",
    "sitting",
    "lying",
]

NAV_VERBS = [
    "go to",
    "navigate to",
    "reach",
    "move to",
    "head over to",
    "head to",
    "walk to",
    "travel to",
]


class GPSRParser:
    """Offline rule-based parser for RoboCup@Home GPSR commands."""

    def __init__(self, knowledge: Optional[CompetitionKnowledge] = None, data_dir: Optional[str] = None):
        if knowledge is not None:
            self.knowledge = knowledge
        else:
            self.knowledge = load_knowledge(data_dir)

        # Precompute search lists sorted by length descending to match longest phrases first
        self._all_locations = sorted(
            list(set(self.knowledge.all_locations + list(SYNONYMS_LOCATION.keys()))),
            key=lambda s: len(s),
            reverse=True,
        )

        all_objs = list(self.knowledge.objects) + list(SYNONYMS_OBJECT.keys())
        self._all_objects = sorted(list(set(all_objs)), key=lambda s: len(s), reverse=True)

        all_cats = list(self.knowledge.categories_singular) + list(self.knowledge.categories_plural) + ["object", "objects", "item", "items"]
        self._all_categories = sorted(list(set(all_cats)), key=lambda s: len(s), reverse=True)

        self._all_names = sorted(list(set(self.knowledge.names)), key=lambda s: len(s), reverse=True)

    def canonical_location(self, loc: str) -> str:
        loc_clean = loc.strip().lower()
        return SYNONYMS_LOCATION.get(loc_clean, loc.strip())

    def canonical_object(self, obj: str) -> str:
        obj_clean = obj.strip().lower()
        return SYNONYMS_OBJECT.get(obj_clean, obj.strip())

    def match_location(self, text: str) -> Optional[Tuple[str, int, int]]:
        """Return (canonical_loc, start, end) for the first/best location match in text."""
        text_lower = text.lower()
        for loc in self._all_locations:
            pattern = r"\b" + re.escape(loc.lower()) + r"\b"
            match = re.search(pattern, text_lower)
            if match:
                return self.canonical_location(loc), match.start(), match.end()
        return None

    def match_name(self, text: str) -> Optional[Tuple[str, int, int]]:
        text_lower = text.lower()
        for name in self._all_names:
            pattern = r"\b" + re.escape(name.lower()) + r"\b"
            match = re.search(pattern, text_lower)
            if match:
                return name, match.start(), match.end()
        return None

    def match_object_or_category(self, text: str) -> Optional[Tuple[str, bool, int, int]]:
        """Return (name, is_category, start, end)."""
        text_lower = text.lower()
        for obj in self._all_objects:
            pattern = r"\b" + re.escape(obj.lower()) + r"\b"
            match = re.search(pattern, text_lower)
            if match:
                return self.canonical_object(obj), False, match.start(), match.end()
        for cat in self._all_categories:
            pattern = r"\b" + re.escape(cat.lower()) + r"\b"
            match = re.search(pattern, text_lower)
            if match:
                return cat, True, match.start(), match.end()
        return None

    def match_person_descriptor(self, text: str) -> Optional[Dict[str, str]]:
        """Extract person descriptor (name, gesture, posture, clothing)."""
        text_lower = text.lower()
        if re.search(r"\b(me|myself|operator|user)\b", text_lower):
            return {"name": "me"}

        name_match = self.match_name(text)
        if name_match:
            return {"name": name_match[0]}

        for gesture in GESTURES:
            if gesture in text_lower:
                return {"gesture": gesture}

        for posture in POSTURES:
            pattern = r"\b" + re.escape(posture) + r"(\s+person)?\b"
            if re.search(pattern, text_lower):
                return {"posture": posture}

        clothing_match = re.search(r"wearing\s+(?:a\s+|an\s+)?([a-z\s]+)", text_lower)
        if clothing_match:
            return {"clothing": clothing_match.group(1).strip()}

        return None

    def parse(self, command: str) -> PlanResult:
        """Deterministically parse a GPSR command into a PlanResult."""
        try:
            clean_cmd = " ".join(command.strip().split())
            clean_cmd = clean_cmd.rstrip(".?!;,")
            if not clean_cmd:
                return PlanResult(
                    success=False,
                    command=command,
                    actions=[],
                    explanation="Empty command provided.",
                    error="Empty command",
                )

            actions = self._parse_command_internal(clean_cmd)

            if not actions:
                return PlanResult(
                    success=False,
                    command=command,
                    actions=[],
                    explanation="Could not generate an action plan for this command.",
                    error=f"Unrecognized command pattern: '{command}'",
                )

            dict_actions = [a.to_dict() for a in actions]
            explanation = self.explain_plan(dict_actions)

            return PlanResult(
                success=True,
                command=command,
                actions=dict_actions,
                explanation=explanation,
                error=None,
            )
        except Exception as e:
            return PlanResult(
                success=False,
                command=command,
                actions=[],
                explanation="Internal parsing error occurred.",
                error=f"{type(e).__name__}: {str(e)}",
            )

    def _parse_command_internal(self, cmd: str) -> List[Action]:
        cmd_lower = cmd.lower()

        # 1. Answer question commands
        if "answer" in cmd_lower and "question" in cmd_lower:
            loc_match = self.match_location(cmd_lower)
            desc = self.match_person_descriptor(cmd_lower)
            actions = []
            if loc_match:
                actions.append(Action("go_to", {"location": loc_match[0]}))
            if desc and desc.get("name") != "me":
                actions.append(Action("find_person", desc))
            actions.append(Action("answer_question", {}))
            return actions

        # 2. Counting commands
        # e.g.: "tell me how many people in the living room are wearing gray jackets"
        # e.g.: "tell me how many snacks there are on the shelf"
        # e.g.: "count how many people are in the bedroom"
        if "how many" in cmd_lower or cmd_lower.startswith("count "):
            count_action = self._parse_count_command(cmd)
            if count_action:
                return count_action

        # 3. Object comparison / description commands
        # e.g.: "tell me what is the largest drink on the storage rack"
        desc_match = re.search(r"tell\s+me\s+what\s+is\s+the\s+(\w+)\s+(.*?)\s+(?:on|in|at)\s+(?:the\s+)?([a-z\s]+)", cmd_lower)
        if desc_match:
            prop = desc_match.group(1).strip()
            target = desc_match.group(2).strip()
            loc_str = desc_match.group(3).strip()
            loc_match = self.match_location(loc_str)
            loc = loc_match[0] if loc_match else self.canonical_location(loc_str)
            return [
                Action("go_to", {"location": loc}),
                Action("describe", {"property": prop, "target": target, "location": loc}),
            ]

        # 4. Person property inspection commands
        person_info_match = re.search(
            r"tell\s+(?:me\s+)?the\s+(pose|gesture|name)\s+of\s+the\s+person\s+(?:at|in)\s+(?:the\s+)?([a-z\s]+?)\s+to\s+the\s+person\s+(?:at|in)\s+(?:the\s+)?([a-z\s]+)",
            cmd_lower,
        )
        if person_info_match:
            prop = person_info_match.group(1).strip()
            loc1_str = person_info_match.group(2).strip()
            loc2_str = person_info_match.group(3).strip()
            l1 = self.match_location(loc1_str)
            l2 = self.match_location(loc2_str)
            loc1 = l1[0] if l1 else self.canonical_location(loc1_str)
            loc2 = l2[0] if l2 else self.canonical_location(loc2_str)
            return [
                Action("go_to", {"location": loc1}),
                Action("find_person", {"location": loc1}),
                Action("describe", {"target": "person", "property": prop, "location": loc1}),
                Action("go_to", {"location": loc2}),
                Action("find_person", {"location": loc2}),
                Action("say", {"text": f"person at {loc1}'s {prop}"}),
            ]

        # 5. "bring me X from Y" or "give me X from Y"
        bring_me_match = re.match(
            r"^(?:please\s+)?(?:bring|give)\s+me\s+(?:a|an|the)?\s*([a-z\s]+?)\s+from\s+(?:the\s+)?([a-z\s]+)$",
            cmd_lower,
        )
        if bring_me_match:
            item_str = bring_me_match.group(1).strip()
            loc_str = bring_me_match.group(2).strip()
            loc_match = self.match_location(loc_str)
            loc = loc_match[0] if loc_match else self.canonical_location(loc_str)
            item_match = self.match_object_or_category(item_str)
            item_name = item_match[0] if item_match else self.canonical_object(item_str)
            is_cat = item_match[1] if item_match else False

            find_param = {"category": item_name} if is_cat else {"object": item_name}
            return [
                Action("go_to", {"location": loc}),
                Action("find_object", find_param),
                Action("pick", {"object": item_name}),
                Action("deliver_to", {"person": "me"}),
            ]

        # 6. "take/fetch/get/grasp X from Y and [deliver/put/throw...]"
        take_from_match = re.match(
            r"^(?:please\s+)?(?:take|get|fetch|grasp)\s+(?:a|an|the)?\s*([a-z\s]+?)\s+from\s+(?:the\s+)?([a-z\s]+?)\s+and\s+(.*)$",
            cmd_lower,
        )
        if take_from_match:
            item_str = take_from_match.group(1).strip()
            loc_str = take_from_match.group(2).strip()
            rest = take_from_match.group(3).strip()

            loc_match = self.match_location(loc_str)
            loc = loc_match[0] if loc_match else self.canonical_location(loc_str)
            item_match = self.match_object_or_category(item_str)
            item_name = item_match[0] if item_match else self.canonical_object(item_str)
            is_cat = item_match[1] if item_match else False
            find_param = {"category": item_name} if is_cat else {"object": item_name}

            actions = [
                Action("go_to", {"location": loc}),
                Action("find_object", find_param),
                Action("pick", {"object": item_name}),
            ]
            tail_actions = self._parse_delivery_or_placement(rest, default_item=item_name)
            actions.extend(tail_actions)
            return actions

        # 7. Escort/guide/lead/take/follow from loc1 to loc2 (or just to loc2)
        # e.g.: "guide Charlie to the bathroom"
        # e.g.: "escort Jane to the living room"
        # e.g.: "take Adel to the office"
        # e.g.: "escort Angel from the entrance to the refrigerator"
        # e.g.: "follow Angel from the trashbin to the living room"
        guide_match_from_to = re.match(
            r"^(?:please\s+)?(guide|escort|lead|take|follow)\s+(?:the\s+)?(.*?)\s+from\s+(?:the\s+)?([a-z\s]+?)\s+to\s+(?:the\s+)?([a-z\s]+)$",
            cmd_lower,
        )
        if guide_match_from_to:
            verb = guide_match_from_to.group(1).strip()
            target_person_str = guide_match_from_to.group(2).strip()
            loc1_str = guide_match_from_to.group(3).strip()
            loc2_str = guide_match_from_to.group(4).strip()
            l1 = self.match_location(loc1_str)
            l2 = self.match_location(loc2_str)
            loc1 = l1[0] if l1 else self.canonical_location(loc1_str)
            loc2 = l2[0] if l2 else self.canonical_location(loc2_str)

            p_desc = self.match_person_descriptor(target_person_str) or {"name": target_person_str}
            actions = [
                Action("go_to", {"location": loc1}),
                Action("find_person", p_desc),
            ]
            person_name = p_desc.get("name") or "them"
            if verb == "follow":
                actions.append(Action("follow", {"person": person_name, "destination": loc2}))
            else:
                actions.append(Action("guide", {"person": person_name, "location": loc2}))
            return actions

        guide_match_to = re.match(
            r"^(?:please\s+)?(guide|escort|lead|take)\s+(?:the\s+)?(.*?)\s+to\s+(?:the\s+)?([a-z\s]+)$",
            cmd_lower,
        )
        if guide_match_to:
            target_person_str = guide_match_to.group(2).strip()
            loc_str = guide_match_to.group(3).strip()
            loc_match = self.match_location(loc_str)
            dest_loc = loc_match[0] if loc_match else self.canonical_location(loc_str)
            p_desc = self.match_person_descriptor(target_person_str) or {"name": target_person_str}
            person_name = p_desc.get("name") or target_person_str
            return [
                Action("guide", {"person": person_name, "location": dest_loc}),
            ]

        # 8. Speech to person: "say/tell <something> to <person> [in/at <loc>]"
        speech_match = re.match(
            r"^(?:please\s+)?(?:say|tell)\s+(.*?)\s+to\s+(?:the\s+)?([a-z\s]+)$",
            cmd_lower,
        )
        if speech_match:
            what_to_say = speech_match.group(1).strip()
            target_str = speech_match.group(2).strip()
            # If target_str is "me", it's a command like "tell me what day is today" or "tell me the time"
            if target_str in ("me", "us"):
                return [Action("say", {"text": what_to_say})]
            return self._parse_speech_to_person(what_to_say, target_str)

        # 9. Simple speech without explicit recipient: "say <something>", "tell <something>"
        simple_speech_match = re.match(
            r"^(?:please\s+)?(?:say|tell)\s+(?:me\s+)?(.*)$",
            cmd_lower,
        )
        if simple_speech_match and not any(k in cmd_lower for k in ["how many", "what is"]):
            text_to_say = simple_speech_match.group(1).strip()
            return [Action("say", {"text": text_to_say})]

        # 10. General sequence split by " then " or " and "
        return self._parse_clause_sequence(cmd)

    def _parse_count_command(self, cmd: str) -> Optional[List[Action]]:
        cmd_lower = cmd.lower()
        loc_match = self.match_location(cmd_lower)
        loc = loc_match[0] if loc_match else "room"

        count_params: Dict[str, Any] = {"location": loc}

        # Check clothing
        clothing_match = re.search(r"wearing\s+([a-z\s]+)", cmd_lower)
        if clothing_match:
            count_params["clothing"] = clothing_match.group(1).strip()

        # Check gesture
        for g in GESTURES:
            if g in cmd_lower:
                count_params["gesture"] = g
                break

        # Check posture
        for p in POSTURES:
            if p in cmd_lower:
                count_params["posture"] = p
                break

        # Target category or persons
        cat_match = self.match_object_or_category(cmd_lower)
        if cat_match:
            count_params["target"] = cat_match[0]
        elif "people" in cmd_lower or "person" in cmd_lower or "persons" in cmd_lower:
            count_params["target"] = "people"
        else:
            count_params["target"] = "objects"

        return [
            Action("go_to", {"location": loc}),
            Action("count", count_params),
        ]

    def _parse_speech_to_person(self, what_to_say: str, target_str: str) -> List[Action]:
        actions = []
        loc_match = self.match_location(target_str)
        loc = loc_match[0] if loc_match else None

        desc = self.match_person_descriptor(target_str) or {}
        if loc:
            actions.append(Action("go_to", {"location": loc}))
            if desc and desc.get("name") != "me":
                actions.append(Action("find_person", desc))
            elif not desc:
                actions.append(Action("find_person", {"location": loc}))
        else:
            if desc and desc.get("name") != "me":
                actions.append(Action("find_person", desc))

        actions.append(Action("say", {"text": what_to_say}))
        return actions

    def _parse_delivery_or_placement(self, text: str, default_item: str = "it") -> List[Action]:
        text_lower = text.lower().strip()
        actions = []

        if "trash" in text_lower or "waste basket" in text_lower:
            actions.append(Action("go_to", {"location": "trashbin"}))
            actions.append(Action("place", {"location": "trashbin"}))
            return actions

        place_match = re.search(r"(?:place|put)\s+(?:it\s+)?(?:on|in)\s+(?:the\s+)?([a-z\s]+)", text_lower)
        if place_match:
            dest_str = place_match.group(1).strip()
            loc_match = self.match_location(dest_str)
            dest_loc = loc_match[0] if loc_match else self.canonical_location(dest_str)
            actions.append(Action("go_to", {"location": dest_loc}))
            actions.append(Action("place", {"location": dest_loc}))
            return actions

        if re.search(r"(?:bring|give|deliver)\s+(?:it\s+)?to\s+me", text_lower):
            actions.append(Action("deliver_to", {"person": "me"}))
            return actions

        deliv_match = re.search(r"(?:bring|give|deliver)\s+(?:it\s+)?to\s+(?:the\s+)?([a-z\s]+)", text_lower)
        if deliv_match:
            target_str = deliv_match.group(1).strip()
            loc_match = self.match_location(target_str)
            desc = self.match_person_descriptor(target_str)
            if loc_match:
                actions.append(Action("go_to", {"location": loc_match[0]}))
                if desc and desc.get("name") != "me":
                    actions.append(Action("find_person", desc))
                    actions.append(Action("deliver_to", {"person": desc.get("name", "person")}))
                else:
                    actions.append(Action("deliver_to", {"location": loc_match[0]}))
            elif desc:
                actions.append(Action("find_person", desc))
                actions.append(Action("deliver_to", {"person": desc.get("name", "person")}))
            else:
                actions.append(Action("deliver_to", {"person": target_str}))
            return actions

        return actions

    def _parse_clause_sequence(self, cmd: str) -> List[Action]:
        clauses = [c.strip() for c in re.split(r"\s+then\s+", cmd, flags=re.IGNORECASE) if c.strip()]
        all_actions: List[Action] = []
        current_loc: Optional[str] = None
        current_person: Optional[str] = None
        current_obj: Optional[str] = None

        for clause in clauses:
            sub_actions = self._parse_subclause(clause, current_loc, current_person, current_obj)
            for a in sub_actions:
                all_actions.append(a)
                if a.action == "go_to":
                    current_loc = a.params.get("location")
                elif a.action == "find_person":
                    current_person = a.params.get("name")
                elif a.action == "find_object":
                    current_obj = a.params.get("object") or a.params.get("category")
                elif a.action == "pick":
                    current_obj = a.params.get("object")

        return all_actions

    def _parse_subclause(
        self,
        clause: str,
        current_loc: Optional[str],
        current_person: Optional[str],
        current_obj: Optional[str],
    ) -> List[Action]:
        c_lower = clause.lower().strip()
        actions: List[Action] = []

        # Navigation: "go to / navigate to / reach / move to / head to <loc>"
        nav_pattern = r"^(?:please\s+)?(?:" + "|".join(re.escape(v) for v in NAV_VERBS) + r")\s+(?:the\s+)?([a-z\s]+)$"
        nav_match = re.match(nav_pattern, c_lower)
        if nav_match:
            loc_str = nav_match.group(1).strip()
            loc_match = self.match_location(loc_str)
            loc = loc_match[0] if loc_match else self.canonical_location(loc_str)
            return [Action("go_to", {"location": loc})]

        # Throw in trash atomic
        if c_lower in ("throw it in the trash", "throw it into the trash", "trash it"):
            return [
                Action("go_to", {"location": "trashbin"}),
                Action("place", {"location": "trashbin"}),
            ]

        # Single question answering
        if "answer" in c_lower and "question" in c_lower:
            loc_match = self.match_location(c_lower)
            if loc_match:
                actions.append(Action("go_to", {"location": loc_match[0]}))
            desc = self.match_person_descriptor(c_lower)
            if desc and desc.get("name") != "me":
                actions.append(Action("find_person", desc))
            actions.append(Action("answer_question", {}))
            return actions

        # Subclause with ' and '
        sub_parts = [p.strip() for p in re.split(r"\s+and\s+", clause, flags=re.IGNORECASE) if p.strip()]
        if len(sub_parts) > 1:
            for idx, part in enumerate(sub_parts):
                p_lower = part.lower()

                # Navigation
                nav_match = re.match(nav_pattern, p_lower)
                if nav_match:
                    loc_match = self.match_location(nav_match.group(1).strip())
                    if loc_match:
                        actions.append(Action("go_to", {"location": loc_match[0]}))
                        current_loc = loc_match[0]
                    continue

                # Meet / salute / greet person
                if any(p_lower.startswith(w) for w in ["meet ", "salute ", "greet ", "say hello to ", "introduce yourself to "]):
                    name_match = self.match_name(p_lower)
                    loc_match = self.match_location(p_lower)
                    if loc_match and loc_match[0] != current_loc:
                        actions.append(Action("go_to", {"location": loc_match[0]}))
                        current_loc = loc_match[0]
                    person_name = name_match[0] if name_match else (current_person or "person")
                    actions.append(Action("find_person", {"name": person_name}))
                    current_person = person_name

                    if any(p_lower.startswith(w) for w in ["salute ", "greet ", "say hello to "]):
                        actions.append(Action("say", {"text": "hello"}))
                    elif p_lower.startswith("introduce yourself to"):
                        actions.append(Action("say", {"text": "introduction"}))
                    continue

                # Locate / find person
                if any(p_lower.startswith(w) for w in ["locate ", "find ", "look for "]) and ("person" in p_lower or any(n.lower() in p_lower for n in self._all_names)):
                    loc_match = self.match_location(p_lower)
                    if loc_match and loc_match[0] != current_loc:
                        actions.append(Action("go_to", {"location": loc_match[0]}))
                        current_loc = loc_match[0]
                    desc = self.match_person_descriptor(p_lower) or {"name": current_person or "person"}
                    actions.append(Action("find_person", desc))
                    if "name" in desc:
                        current_person = desc["name"]
                    continue

                # Locate / find object
                if any(p_lower.startswith(w) for w in ["locate ", "find ", "look for "]):
                    loc_match = self.match_location(p_lower)
                    if loc_match and loc_match[0] != current_loc:
                        actions.append(Action("go_to", {"location": loc_match[0]}))
                        current_loc = loc_match[0]
                    item_match = self.match_object_or_category(p_lower)
                    if item_match:
                        item_name, is_cat = item_match[0], item_match[1]
                        current_obj = item_name
                        find_param = {"category": item_name} if is_cat else {"object": item_name}
                        actions.append(Action("find_object", find_param))
                    continue

                # Grasp / pick / take / get
                if any(p_lower.startswith(w) for w in ["grasp ", "pick ", "take ", "get ", "fetch "]):
                    item_match = self.match_object_or_category(p_lower)
                    obj_to_pick = item_match[0] if item_match else (current_obj or "object")
                    current_obj = obj_to_pick
                    actions.append(Action("pick", {"object": obj_to_pick}))
                    continue

                # Put / place
                if any(p_lower.startswith(w) for w in ["put ", "place ", "drop "]):
                    loc_match = self.match_location(p_lower)
                    loc = loc_match[0] if loc_match else "table"
                    if loc != current_loc:
                        actions.append(Action("go_to", {"location": loc}))
                        current_loc = loc
                    actions.append(Action("place", {"location": loc}))
                    continue

                # Trash
                if "trash" in p_lower or "waste basket" in p_lower:
                    actions.append(Action("go_to", {"location": "trashbin"}))
                    actions.append(Action("place", {"location": "trashbin"}))
                    current_loc = "trashbin"
                    continue

                # Deliver / bring / give
                if any(p_lower.startswith(w) for w in ["bring ", "give ", "deliver "]):
                    tail_acts = self._parse_delivery_or_placement(p_lower, default_item=current_obj or "it")
                    actions.extend(tail_acts)
                    continue

                # Follow
                if p_lower.startswith("follow"):
                    loc_match = self.match_location(p_lower)
                    name_match = self.match_name(p_lower)
                    desc = self.match_person_descriptor(p_lower)
                    person = name_match[0] if name_match else (desc.get("name") if desc else (current_person or "them"))
                    if loc_match:
                        actions.append(Action("follow", {"person": person, "destination": loc_match[0]}))
                    else:
                        actions.append(Action("follow", {"person": person}))
                    continue

                # Guide / take / lead / escort
                if any(p_lower.startswith(w) for w in ["guide ", "take ", "lead ", "escort "]):
                    loc_match = self.match_location(p_lower)
                    name_match = self.match_name(p_lower)
                    dest_loc = loc_match[0] if loc_match else "room"
                    person = name_match[0] if name_match else (current_person or "them")
                    actions.append(Action("guide", {"person": person, "location": dest_loc}))
                    continue

                # Say / tell
                if p_lower.startswith(("say ", "tell ")):
                    speech_text = re.sub(r"^(?:say|tell)\s+", "", part).strip()
                    actions.append(Action("say", {"text": speech_text}))
                    continue

            if actions:
                return actions

        return self._parse_atomic_clause(clause, current_loc, current_person, current_obj)

    def _parse_atomic_clause(
        self,
        clause: str,
        current_loc: Optional[str],
        current_person: Optional[str],
        current_obj: Optional[str],
    ) -> List[Action]:
        c_lower = clause.lower().strip()

        # Follow
        if c_lower.startswith("follow"):
            loc_match = self.match_location(c_lower)
            desc = self.match_person_descriptor(c_lower)
            person = desc.get("name") if desc else (current_person or "them")
            if loc_match and ("at " in c_lower or "in " in c_lower):
                return [
                    Action("go_to", {"location": loc_match[0]}),
                    Action("find_person", desc or {"location": loc_match[0]}),
                    Action("follow", {"person": person}),
                ]
            elif loc_match and "to " in c_lower:
                return [Action("follow", {"person": person, "destination": loc_match[0]})]
            return [Action("follow", {"person": person})]

        # Meet / salute someone in loc
        if any(c_lower.startswith(w) for w in ["meet ", "salute ", "greet "]):
            loc_match = self.match_location(c_lower)
            name_match = self.match_name(c_lower)
            actions = []
            if loc_match:
                actions.append(Action("go_to", {"location": loc_match[0]}))
            person = name_match[0] if name_match else (current_person or "person")
            actions.append(Action("find_person", {"name": person}))
            if not c_lower.startswith("meet"):
                actions.append(Action("say", {"text": "hello"}))
            return actions

        # Find person in loc
        if any(c_lower.startswith(w) for w in ["find ", "locate ", "look for "]) and "person" in c_lower:
            loc_match = self.match_location(c_lower)
            desc = self.match_person_descriptor(c_lower) or {}
            actions = []
            if loc_match:
                actions.append(Action("go_to", {"location": loc_match[0]}))
            actions.append(Action("find_person", desc))
            return actions

        # Find object in loc: "find an orange in the bedroom"
        if any(c_lower.startswith(w) for w in ["find ", "locate ", "look for "]):
            loc_match = self.match_location(c_lower)
            item_match = self.match_object_or_category(c_lower)
            actions = []
            if loc_match:
                actions.append(Action("go_to", {"location": loc_match[0]}))
            if item_match:
                find_param = {"category": item_match[0]} if item_match[1] else {"object": item_match[0]}
                actions.append(Action("find_object", find_param))
            return actions

        # Pick / pick up object: "pick up the tennis ball"
        pick_match = re.match(r"^(?:please\s+)?(?:pick\s+up|pick|grasp|take|grab)\s+(?:the\s+|a\s+|an\s+)?([a-z\s]+)$", c_lower)
        if pick_match:
            item_str = pick_match.group(1).strip()
            item_match = self.match_object_or_category(item_str)
            obj_name = item_match[0] if item_match else self.canonical_object(item_str)
            return [Action("pick", {"object": obj_name})]

        # Place on loc: "place it on the kitchen table"
        place_match = re.match(r"^(?:please\s+)?(?:place|put|drop|leave)\s+(?:it\s+)?(?:on|in|at)\s+(?:the\s+)?([a-z\s]+)$", c_lower)
        if place_match:
            loc_str = place_match.group(1).strip()
            loc_match = self.match_location(loc_str)
            loc = loc_match[0] if loc_match else self.canonical_location(loc_str)
            return [Action("place", {"location": loc})]

        # Deliver to me / person: "deliver it to me"
        deliv_match = re.match(r"^(?:please\s+)?(?:deliver|bring|give)\s+(?:it\s+)?to\s+(?:the\s+)?([a-z\s]+)$", c_lower)
        if deliv_match:
            target_str = deliv_match.group(1).strip()
            if target_str in ("me", "myself", "operator"):
                return [Action("deliver_to", {"person": "me"})]
            loc_match = self.match_location(target_str)
            if loc_match:
                return [Action("deliver_to", {"location": loc_match[0]})]
            return [Action("deliver_to", {"person": target_str})]

        # Say / tell
        if c_lower.startswith(("say ", "tell ")):
            text = re.sub(r"^(?:say|tell)\s+", "", clause, flags=re.IGNORECASE).strip()
            return [Action("say", {"text": text})]

        return []

    def explain_plan(self, actions: List[Dict[str, Any]]) -> str:
        """Generate human-readable English explanation for speech output."""
        if not actions:
            return "No actions planned."

        clauses = []
        for a in actions:
            act_type = a.get("action")
            params = a.get("params", {})
            if act_type == "go_to":
                clauses.append(f"go to the {params.get('location', 'location')}")
            elif act_type == "find_object":
                target = params.get("object") or params.get("category", "object")
                clauses.append(f"find the {target}")
            elif act_type == "pick":
                clauses.append(f"pick up the {params.get('object', 'object')}")
            elif act_type == "place":
                clauses.append(f"place it on the {params.get('location', 'location')}")
            elif act_type == "deliver_to":
                person = params.get("person")
                loc = params.get("location")
                if person == "me":
                    clauses.append("deliver it to you")
                elif person:
                    clauses.append(f"deliver it to {person}")
                elif loc:
                    clauses.append(f"deliver it to the {loc}")
                else:
                    clauses.append("deliver it")
            elif act_type == "find_person":
                if "name" in params:
                    clauses.append(f"meet {params['name']}")
                elif "gesture" in params:
                    clauses.append(f"find the person {params['gesture']}")
                elif "posture" in params:
                    clauses.append(f"find the {params['posture']} person")
                elif "clothing" in params:
                    clauses.append(f"find the person wearing {params['clothing']}")
                else:
                    clauses.append("find the person")
            elif act_type == "follow":
                person = params.get("person", "them")
                dest = params.get("destination")
                if dest:
                    clauses.append(f"follow {person} to the {dest}")
                else:
                    clauses.append(f"follow {person}")
            elif act_type == "guide":
                person = params.get("person", "them")
                loc = params.get("location", "destination")
                clauses.append(f"guide {person} to the {loc}")
            elif act_type == "say":
                clauses.append(f"say {params.get('text', '')}")
            elif act_type == "answer_question":
                clauses.append("answer the question")
            elif act_type == "count":
                target = params.get("target", "items")
                loc = params.get("location")
                loc_phrase = f" at the {loc}" if loc else ""
                clauses.append(f"count the {target}{loc_phrase}")
            elif act_type == "describe":
                prop = params.get("property", "details")
                target = params.get("target", "target")
                loc = params.get("location")
                loc_phrase = f" at the {loc}" if loc else ""
                clauses.append(f"identify the {prop} {target}{loc_phrase}")
            else:
                clauses.append(f"perform {act_type}")

        if len(clauses) == 1:
            return f"I will {clauses[0]}."
        elif len(clauses) == 2:
            return f"I will {clauses[0]}, and {clauses[1]}."
        else:
            first_part = ", ".join(clauses[:-1])
            return f"I will {first_part}, and {clauses[-1]}."
