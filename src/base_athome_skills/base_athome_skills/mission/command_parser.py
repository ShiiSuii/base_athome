"""Commander: compila un pedido en lenguaje natural (espanol/ingles) a un plan de mision. Logica pura y determinista.

  "Anda a recepcion, saluda a la persona, despues dirigite a la mesa, busca una botella y llevala a la cocina"
  -> navigate_to reception, speak, navigate_to table, arm_pose pre_grasp, detect_object bottle,
     pick_object bottle, navigate_to kitchen, place_object

Reglas: nunca se adivina (lo que no se entiende es un error explicito y no se ejecuta nada); los pasos implicitos
(observar antes de detectar, detectar y agarrar antes de llevar) se agregan segun lo que el robot ya tiene; el plan
termina con una recuperacion (avisar y volver a casa) y success_if en la ultima accion util.
"""
import re
import unicodedata
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from .places import resolve

PLACE_ALIASES = {
    "reception": ["recepcion", "reception", "la recepcion", "lobby"],
    "door": ["puerta", "entrada", "door", "entrance"],
    "living_room": ["sala", "sala de estar", "living", "living room", "livingroom", "comedor"],
    "kitchen": ["cocina", "kitchen"],
    "table": ["mesa", "table"],
    "bedroom": ["dormitorio", "cuarto", "habitacion", "bedroom"],
    "bathroom": ["bano", "bathroom"],
    "home": ["casa", "home", "inicio", "base"],
}
OBJECT_ALIASES = {
    "bottle": ["botella", "bottle"], "can": ["lata", "can"], "cup": ["taza", "vaso", "cup", "glass"],
    "apple": ["manzana", "apple"], "banana": ["banana", "platano"], "book": ["libro", "book"],
    "pringles": ["pringles", "papas"], "remote": ["control remoto", "remote"], "bag": ["bolsa", "bag"],
}
FILLERS = ("primero", "luego", "despues", "entonces", "tambien", "por favor", "robot", "first", "then", "next",
           "after that", "afterwards", "also", "please", "and", "y", "finalmente", "finally", "por ultimo")
ARTICLES = r"(?:el|la|los|las|un|una|al|a|del|de|en|hacia|hasta|the|an|to|at|in|on|into|towards?)"
DEFAULT_GREETING = "Hello! Nice to meet you."
RE_NUM = {"un": 1, "una": 1, "dos": 2, "tres": 3, "cuatro": 4, "cinco": 5, "diez": 10, "one": 1, "two": 2, "three": 3,
          "four": 4, "five": 5, "ten": 10}


def norm(text: str) -> str:
    t = unicodedata.normalize("NFD", text.lower())
    return "".join(c for c in t if unicodedata.category(c) != "Mn")


@dataclass
class Compiled:
    ok: bool
    plan: Optional[Dict[str, Any]] = None
    steps_text: List[str] = field(default_factory=list)
    errors: List[str] = field(default_factory=list)
    cancel: bool = False       # el pedido es "parar/cancelar": cancelar la mision en curso


PRONOUNS = {"eso", "esa", "ese", "esto", "esta", "este", "lo", "la", "it", "that", "this", "them", "ello", "objeto", "object"}
PERSON_WORDS = r"\b(?:persona|personas|invitado|invitada|anfitrion|gente|alguien|person|people|guest|host|someone|operator|operador)\b"
CANCEL_RE = re.compile(r"^(?:robot\s*,?\s*)?(?:para|parate|detenete|frena|alto|cancela|cancelar|aborta|abortar|stop|cancel|abort|halt)\b[\s.!]*(?:todo|la mision|the mission|everything|ya|now)?[\s.!]*$")
FAIL_COND_RE = re.compile(r"^(?:si\s+no\s+(?:la|lo|las|los)?\s*(?:ves|encontras|encuentras|detectas|hay)\b|if\s+(?:you\s+)?(?:can\s*not|can't|cant|don't|dont|do\s+not)\s+(?:find|see|detect)(?:\s+(?:it|them))?\b)\s*(.*)$")


@dataclass
class _Ctx:
    places: Dict[str, Any]
    steps: List[Dict[str, Any]] = field(default_factory=list)
    text: List[str] = field(default_factory=list)
    errors: List[str] = field(default_factory=list)
    holding: Optional[str] = None
    seen: Optional[str] = None
    observing: bool = False
    last_verb: Optional[str] = None
    last_obj: Optional[str] = None
    last_detect: Optional[str] = None     # id del ultimo detect_object
    cond: Optional[Dict[str, str]] = None # "if" a aplicar a los pasos que se agregan
    n: int = 0

    def add(self, skill, args, desc, retries=0, recover=True, timeout=None):
        self.n += 1
        st = {"id": f"s{self.n}_{skill}", "skill": skill, "args": args, "retries": retries}
        if timeout:
            st["timeout_s"] = timeout
        if recover:
            st["on_failure"] = "avisar"
        if self.cond:
            st["if"] = dict(self.cond)
        self.steps.append(st)
        state = (self.cond or {}).get("state")
        self.text.append({"FAILED": f"si no lo encuentra: {desc}", "SUCCEEDED": f"si lo encuentra: {desc}"}.get(state, desc))


def _strip_articles(t: str) -> str:
    t = t.strip(" .,;:!?¡¿")
    while True:
        new = re.sub(rf"^{ARTICLES}\s+", "", t)
        if new == t:
            return t
        t = new


def find_place(phrase: str, places: Dict[str, Any]) -> Tuple[Optional[str], str]:
    """(nombre en el mapa o None, texto limpio). Prueba alias -> nombre del mapa, y el nombre literal."""
    p = _strip_articles(phrase)
    p = re.sub(r"^(?:punto|waypoint|point|lugar)\s+", "", p).strip()
    if not p:
        return None, p
    if resolve(places, p) is not None:
        return next(k for k in places if k.lower() == p.replace(" ", "_").replace("-", "_").lower()), p
    for canon, aliases in PLACE_ALIASES.items():
        if p in aliases and resolve(places, canon) is not None:
            return canon, p
    return None, p


def find_object(phrase: str) -> Optional[str]:
    """Nombre canonico del objeto, o None si no hay (pronombres como 'eso' = el objeto en contexto)."""
    p = _strip_articles(phrase)
    if p in PRONOUNS or not p:
        return None
    for canon, aliases in OBJECT_ALIASES.items():
        for a in aliases:
            if re.search(rf"\b{a}\b", p):
                return canon
    p = re.sub(r"^(?:algun|alguna|some|any)\s+", "", p).strip()
    return p if re.fullmatch(r"[a-z][a-z ]{1,24}", p) else None


def _named_place(c: "_Ctx", phrase: str, raw: str) -> Tuple[str, bool]:
    """'una botella en la cocina' -> navega a la cocina y devuelve 'una botella'."""
    m = re.search(r"\b(en|sobre|on|in|at|from|de|del)\s+((?:la |el |the )?[\w ]+)$", phrase)
    if not m:
        return phrase, True
    place, cleaned = find_place(m.group(2), c.places)
    if place is None:
        if m.group(1) in ("de", "del"):      # "bolsa de papas": parte del nombre
            return phrase, True
        c.errors.append(f"'{raw}': no conozco el lugar '{cleaned}'")
        return phrase, False
    _go(c, m.group(2), raw)
    return phrase[:m.start()].strip(), True


def _go(c: _Ctx, phrase: str, raw: str) -> None:
    place, cleaned = find_place(phrase, c.places)
    if place is None:
        c.errors.append(f"no conozco el lugar '{cleaned or raw}' (conozco: {', '.join(sorted(c.places)) or 'ninguno'})")
        return
    c.observing = False
    c.seen = None                           # lo visto antes ya no sirve en otro lugar
    if place in ("home", "default"):
        c.add("return_home", {}, "volver a casa", retries=1, timeout=180)
    else:
        c.add("navigate_to", {"place": place}, f"ir a '{place}'", retries=1, timeout=180)


def _observe_and_detect(c: _Ctx, obj: str) -> None:
    if not c.observing:
        c.add("arm_pose", {"pose": "pre_grasp"}, "llevar el brazo a la pose de observacion", timeout=60)
        c.observing = True
    c.add("detect_object", {"object": obj}, f"buscar '{obj}' con la camara", retries=2)
    c.last_detect = c.steps[-1]["id"]
    c.seen = c.last_obj = obj


def _pick(c: _Ctx, obj: Optional[str], raw: str) -> None:
    obj = obj or c.seen or c.last_obj
    if obj is None:
        c.errors.append(f"'{raw}': no se que objeto agarrar (nombralo o pedi buscarlo antes)")
        return
    if c.holding:
        c.errors.append(f"'{raw}': ya tengo '{c.holding}' en la pinza; primero hay que dejarlo")
        return
    if c.seen != obj:
        _observe_and_detect(c, obj)
    c.add("pick_object", {"object": obj}, f"agarrar '{obj}'", retries=1, timeout=90)
    c.holding, c.observing = obj, False


def _place(c: _Ctx, dest: Optional[str], raw: str) -> None:
    if not c.holding:
        c.errors.append(f"'{raw}': no tengo nada en la pinza para dejar")
        return
    args = {"place": dest} if dest else {}
    c.add("place_object", args, f"colocar '{c.holding}'" + (f" en '{dest}'" if dest else ""), timeout=60)
    c.holding, c.seen, c.observing = None, None, False


VERBS = [
    ("home", r"(?:volve|volver|regresa|regresar|vuelve|retorna)\b(?:\s+a\s+(?:casa|home|inicio|la base|base))?|(?:go|come|return)\s+(?:back\s+)?(?:home|to base)|return\b|go back\b"),
    ("carry", r"(?:llevala|llevalo|llevar|lleva|traela|traelo|traeme|trae|traer|bring|carry|deliver|take(?=\s.+\sto\s))\b"),
    ("pick", r"(?:agarrala|agarralo|agarrar|agarra|tomala|tomalo|tomar|toma|levantala|levantalo|levantar|levanta|recogela|recogelo|recoger|recoge|pick\s+up|pick|grab|take)\b"),
    ("place", r"(?:dejala|dejalo|dejar|deja|ponela|ponelo|poner|pone|soltala|soltalo|soltar|suelta|colocala|colocalo|colocar|coloca|put|place|drop|leave|release)\b"),
    ("find", r"(?:buscala|buscalo|buscar|busca|encontra|encontrar|encuentra|identifica|identificar|localiza|localizar|detecta|detectar|find|look\s+for|locate|identify|detect|search\s+for)\b"),
    ("greet", r"(?:saludala|saludalo|saludar|saluda|greet|say\s+hello(?:\s+to)?)\b"),
    ("say", r"(?:decile|deci|decir|di|dile|anuncia|say|tell)\b"),
    ("ask", r"(?:preguntale|pregunta|preguntar|ask)\b"),
    ("listen", r"(?:escucha|escuchar|listen)\b"),
    ("wait", r"(?:espera|esperar|aguarda|wait)\b"),
    ("go", r"(?:andate|anda|ir|ve|vete|vaya|dirigite|dirigirse|dirigete|acercate|move\s+to|go\s+to|go|head\s+to|head|navigate\s+to|navigate|walk\s+to|walk|drive\s+to)\b"),
]
_VERB_RE = [(k, re.compile(rf"^(?:{r})")) for k, r in VERBS]
SPLIT_RE = re.compile(r"\s*(?:[,;.]\s*|\s(?:y\s+despues|y\s+luego|y\s+then|and\s+then|despues\s+de\s+eso|luego|despues|entonces|y|and|then)\s)\s*")


def _split(text: str) -> Tuple[List[str], Dict[str, str]]:
    quotes: Dict[str, str] = {}

    def keep(m):
        k = f"qq{len(quotes)}qq"
        quotes[k] = m.group(1)
        return k
    t = re.sub(r"[\"“«]([^\"”»]+)[\"”»]", keep, text)
    parts = [p.strip(" .!?¡¿") for p in SPLIT_RE.split(t)]
    return [p for p in parts if p], quotes


def _clause(c: _Ctx, raw: str, quotes: Dict[str, str]) -> None:
    t = raw
    changed = True
    while changed:
        changed = False
        for f in FILLERS:
            if t.startswith(f + " "):
                t = t[len(f) + 1:].strip()
                changed = True
    if t in FILLERS or not t:
        return
    fc = FAIL_COND_RE.match(t)
    if fc:
        if not c.last_detect:
            c.errors.append(f"'{raw}': 'si no lo ves' necesita una busqueda antes")
            return
        action = fc.group(1).strip()
        if not action:
            c.errors.append(f"'{raw}': que hago si no lo veo?")
            return
        det = next(s for s in c.steps if s["id"] == c.last_detect)
        det["on_failure"] = "continue"       # no encontrarlo es un caso previsto, no un error
        saved = (c.seen, c.observing, c.holding)
        c.cond = {"step": c.last_detect, "state": "FAILED"}
        _clause(c, action, quotes)
        c.seen, c.observing, c.holding = saved
        c.cond = {"step": c.last_detect, "state": "SUCCEEDED"}   # lo que sigue exige haberlo visto
        return
    verb, rest = None, t
    for k, rx in _VERB_RE:
        m = rx.match(t)
        if m:
            verb, rest = k, t[m.end():].strip()
            break
    if verb is None and c.last_verb == "go":          # "... despues al B": mismo verbo
        verb, rest = "go", t
    if verb is None:
        if re.match(r"(?:segui|seguir|sigue|follow|guia|guiar|guide|acompana|acompanar|escort|responde|answer|contesta)\b", t):
            c.errors.append(f"'{raw}': seguir o guiar personas y responder preguntas todavia no esta implementado")
        else:
            c.errors.append(f"no entendi '{raw}'")
        return
    c.last_verb = verb
    if re.search(PERSON_WORDS, rest) and verb in ("find", "pick", "carry"):
        c.errors.append(f"'{raw}': buscar o llevar personas todavia no esta implementado (find_person, guide_person)")
        return
    if verb == "home":
        _go(c, "home", raw)
    elif verb == "go":
        _go(c, rest, raw)
    elif verb == "greet":
        c.add("speak", {"text": DEFAULT_GREETING}, "saludar", recover=False)
    elif verb == "say":
        rest = re.sub(r"^(?:a\s+(?:la\s+persona|el\s+invitado|la\s+gente)|to\s+the\s+(?:person|guest))\s*", "", rest).strip()
        text = quotes.get(rest.strip()) or re.sub(r"^(?:que|that)\s+", "", rest).strip()
        if not text:
            c.errors.append(f"'{raw}': falta que decir")
            return
        c.add("speak", {"text": text}, f"decir: {text}", recover=False)
    elif verb == "ask":
        q = quotes.get(rest.strip()) or re.sub(r"^(?:que|si|to)\s+", "", rest).strip()
        c.add("listen", {"prompt": q} if q else {}, f"preguntar: {q}", recover=False)
        c.steps[-1]["on_failure"] = "continue"
    elif verb == "listen":
        c.add("listen", {}, "escuchar una respuesta", recover=False)
        c.steps[-1]["on_failure"] = "continue"
    elif verb == "wait":
        m = re.search(r"(\d+(?:[.,]\d+)?|\b(?:un|una|dos|tres|cuatro|cinco|diez|one|two|three|four|five|ten)\b)", rest)
        if not m:
            c.errors.append(f"'{raw}': cuanto tiempo espero? (p. ej. 'espera 5 segundos')")
            return
        v = m.group(1)
        sec = float(v.replace(",", ".")) if v[0].isdigit() else float(RE_NUM[v])
        if re.search(r"\bminut", rest):
            sec *= 60.0
        c.add("wait", {"seconds": sec}, f"esperar {sec:g} s", recover=False, timeout=sec + 5.0)
    elif verb == "find":
        rest, ok = _named_place(c, rest, raw)
        if not ok:
            return
        obj = find_object(rest) or c.last_obj
        if obj is None:
            c.errors.append(f"'{raw}': que objeto busco?")
            return
        _observe_and_detect(c, obj)
    elif verb == "pick":
        rest = re.sub(r"\bup\b", "", rest).strip()
        rest, ok = _named_place(c, rest, raw)
        if not ok:
            return
        _pick(c, find_object(rest), raw)
    elif verb == "place":
        rest = re.sub(r"\bdown\b", "", rest).strip()
        m = re.search(r"\b(?:en|sobre|on|in|at|into)\s+(.+)$", rest)
        dest = None
        named = find_object(rest[:m.start()] if m else rest)
        if named and not c.holding:           # "pone la botella en la mesa": agarrarla primero
            _pick(c, named, raw)
            if c.errors:
                return
        if m:
            dest, cleaned = find_place(m.group(1), c.places)
            if dest is None:
                c.errors.append(f"no conozco el lugar '{cleaned}'")
                return
            _go(c, m.group(1), raw)
        _place(c, dest, raw)
    elif verb == "carry":
        m = re.search(r"\b(?:a|al|hasta|hacia|to|into)\s+((?:la |el |the )?[\w ]+)$", rest)
        obj_txt, dest_txt = rest, None
        if m:
            cand, _ = find_place(m.group(1), c.places)
            if cand is None and not re.fullmatch(r"(?:mi|me|ti)", m.group(1).strip()):
                c.errors.append(f"no conozco el lugar '{_strip_articles(m.group(1))}'")
                return
            if cand is not None:
                dest_txt, obj_txt = m.group(1), rest[:m.start()]
        obj = find_object(obj_txt) if obj_txt.strip() else None
        if c.holding and (obj is None or obj == c.holding):
            pass
        else:
            _pick(c, obj, raw)
            if c.errors:
                return
        if dest_txt is not None:
            _go(c, dest_txt, raw)
            _place(c, find_place(dest_txt, c.places)[0], raw)
        else:
            c.add("speak", {"text": "Here you are."}, "avisar que trae el objeto", recover=False)


def compile_command(text: str, places: Dict[str, Any], name: str = "pedido") -> Compiled:
    if not text or not text.strip():
        return Compiled(False, errors=["pedido vacio"])
    if CANCEL_RE.match(norm(text).strip()):
        return Compiled(False, cancel=True)
    parts, quotes = _split(norm(text))
    c = _Ctx(places=places)
    for raw in parts:
        _clause(c, raw, quotes)
    if not c.errors and not c.steps:
        c.errors.append("no encontre ninguna accion en el pedido")
    if c.errors:
        return Compiled(False, errors=c.errors, steps_text=c.text)
    # objetivo: la ultima accion util que no sea la rama alternativa ("si no lo ves ...")
    goal = next((s["id"] for s in reversed(c.steps) if s["skill"] not in ("speak", "listen", "wait")
                 and s.get("if", {}).get("state") != "FAILED"), None)
    recovery = [{"id": "avisar", "skill": "speak", "args": {"text": "I could not complete the request."}}]
    if resolve(places, "home") is not None or resolve(places, "default") is not None:
        recovery.append({"id": "volver", "skill": "return_home", "timeout_s": 180})
    plan = {"name": name, "steps": c.steps, "recovery": recovery}
    if goal:
        plan["success_if"] = goal
    if not any(s.get("on_failure") == "avisar" for s in c.steps):
        plan.pop("recovery")
    return Compiled(True, plan=plan, steps_text=c.text)
