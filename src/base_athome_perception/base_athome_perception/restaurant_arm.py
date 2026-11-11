"""Adaptador Restaurant sobre Manipulacion; no importa ROS ni crea nodos al cargarlo.

Las cuatro poses restaurant_* se agregan desde restaurant_arm_poses.yaml sin
sobrescribir poses existentes. Todas deben tener calibrated: true para mover.
Para ensenarlas en LAB_SUPERVISADO: ejecutar
``python3 -m base_athome_perception.restaurant_arm_teach``. El panel TeachPanel
existente ofrece GRABAR/GUARDAR; save_pose guarda feedback medido SOLO en el
archivo extra, siempre con calibrated: false. Ensenar pregrasp frente al objeto,
postgrasp retirado del mostrador, carry dentro de la envolvente de transporte y
handover frente al cliente. Revisar limites, colisiones y recorridos completos
con supervision antes de marcar cada entrada calibrated: true en ese YAML.
Volver a ensenar invalida esa marca. No modificar arm_poses_and_limits.yaml.
Coke/cola/beer accept bottle OR can detections (YOLO-World fallback); the
original visual label is retained. A recognized drink is not proof of a grasp.
"""

from copy import deepcopy
import math
import threading
import time

from . import pose_store

RESTAURANT_POSES = (
    "restaurant_pregrasp", "restaurant_postgrasp", "restaurant_carry", "restaurant_handover",
)
# Clases COCO; bebidas embotelladas y bebidas servidas se distinguen por el menu.
# No hay clase fiable para muchos snacks: esos items quedan en la bandeja.
ITEM_YOLO_CLASSES = {
    "apple": "apple", "orange": "orange", "banana": "banana",
    "bottle": "bottle", "cup": "cup", "wine glass": "wine glass",
    "water": "bottle", "coke": "bottle|can", "cola": "bottle|can", "beer": "bottle|can",
    "soda": "bottle", "fanta": "bottle", "sprite": "bottle",
    "juice": "bottle", "orange juice": "bottle", "apple juice": "bottle",
    "milk": "bottle", "chocolate milk": "bottle", "lemonade": "bottle",
    "coffee": "cup", "tea": "cup", "wine": "wine glass",
    "sandwich": "sandwich", "cake": "cake",
}
# Mismos valores que arm_manipulation (no se importa: este modulo no carga ROS).
GRIPPER_OPEN_MM = 105.0
GRIPPER_CLOSE_ITEM_MM = 0.0   # cierra con la fuerza del controlador; para en el objeto
GRIPPER_EMPTY_MAX_MM = 2.0    # cerro casi del todo: no hay nada entre los dedos
RESTAURANT_COCO_CLASSES = (39, 40, 41, 46, 47, 48, 49, 61)


class RestaurantItemDetector:
    """Filtra el objetivo pedido antes del tracker, incluso al recurrir a YOLO-World."""

    def __init__(self, detector):
        self.detector = detector
        self.detector.coco_classes = list(RESTAURANT_COCO_CLASSES)
        self.label = None

    @property
    def source(self):
        return self.detector.source

    def detect(self, frame):
        # detect_blocking already accepts | alternatives. Preserve the detector's
        # actual label (a can must not be relabelled a bottle); filter distractors.
        labels = set((self.label or '').casefold().split('|'))
        if not self.label:
            return []
        return [d for d in self.detector.detect(frame) if d[2].casefold() in labels]


def item_yolo_class(item):
    return ITEM_YOLO_CLASSES.get(" ".join(str(item).casefold().split()))


def merge_restaurant_poses(current, extra_path, *, limits_path=None):
    """Agrega solamente poses Restaurant nuevas, validando contra limites actuales."""
    merged = deepcopy(current)
    extra = pose_store.load_named_poses(extra_path)
    for name in RESTAURANT_POSES:
        if name not in merged and name not in extra:
            continue
        pose = merged[name] if name in merged else extra[name]
        if not isinstance(pose, dict):
            raise ValueError(f"pose invalida: {name}")
        joints = pose.get("joints_deg", [])
        if (not pose_store.validate_joints(joints, path=limits_path)
                or not all(math.isfinite(float(v)) for v in joints)):
            raise ValueError(f"articulaciones invalidas: {name}")
        grip = float(pose.get("gripper_mm", -1))
        if not math.isfinite(grip) or not 0 <= grip <= 105:
            raise ValueError(f"pinza invalida: {name}")
        merged.setdefault(name, deepcopy(pose))
    return merged


# Pedido del usuario 10/11: el objeto viaja en postgrasp hasta el cliente; carry es opcional
# (solo recuperacion, y solo si tambien esta calibrada).
RESTAURANT_REQUIRED_POSES = ("restaurant_pregrasp", "restaurant_postgrasp", "restaurant_handover")


def _calibrated(poses, name):
    return isinstance(poses.get(name), dict) and poses[name].get("calibrated") is True


def poses_calibrated(poses):
    return all(_calibrated(poses, n) for n in RESTAURANT_REQUIRED_POSES)


class RestaurantMotionGuard:
    """Mixin de Manipulacion: nunca habilita torque, exige feedback y FJT.

    La guarda tambien se ejecuta antes de cada trayectoria y comando de pinza;
    el controlador conserva sus propios interlocks y rechazo por feedback viejo.
    """

    require_feedback_action = True

    def _vision_loop(self):
        self.detector = RestaurantItemDetector(self.detector)
        super()._vision_loop()

    def select_item(self, label):
        if isinstance(self.detector, RestaurantItemDetector):
            self.detector.label = label
        else:
            raise RuntimeError("El detector Restaurant aun no esta listo")
        with self.lock:
            self.target = self.detection = None
            # Evitar continuidad con el item del pedido anterior.
            self.tracker.box = None
            self.tracker.widths.clear()

    def _status_cb(self, msg):
        # Un JSON malformado no renueva la validez de la ultima muestra.
        import json
        try:
            data = json.loads(msg.data)
            q = data.get("joints_deg", [])
            valid = (data.get("feedback_ok") is True and len(q) == 6
                     and all(math.isfinite(float(v)) for v in q)
                     and math.isfinite(float(data["gripper_mm"])))
        except (TypeError, ValueError, KeyError, AttributeError):
            valid = False
        self.restaurant_feedback_at = time.monotonic() if valid else 0.0
        if valid:
            super()._status_cb(msg)
        else:
            self.feedback_ok = False

    def motion_ready(self):
        return (self.restaurant_permitted() and self.enabled and self.feedback_ok
                and 0 <= time.monotonic() - self.restaurant_feedback_at < 1.0)

    def set_torque(self, on):
        self.status = "Restaurant nunca habilita ni deshabilita torque."
        return False

    def _execute(self, points):
        if (self.abort or not self.motion_ready() or self.fjt is None
                or not self.fjt.wait_for_server(timeout_sec=0.5)):
            self.last_exec_msg = "Restaurant: movimiento bloqueado por guarda/feedback/FJT"
            return False
        return super()._execute(points)

    def gripper(self, mm):
        if self.abort or not self.motion_ready():
            raise RuntimeError("Restaurant: pinza bloqueada por guarda/feedback")
        if getattr(self, "restaurant_holding_gripper", False):
            return  # go_to_blocking aplica el ancho guardado; en transporte no comandar la pinza
        return super().gripper(mm)

    def go_to_holding(self, name):
        """Mueve articulaciones sin publicar comandos de pinza (conserva la sujecion)."""
        if not self.motion_ready() or self.abort:
            return False, "Restaurant: sin autorizacion o feedback fresco"
        self.restaurant_holding_gripper = True
        try:
            return super().go_to_blocking(name)
        finally:
            self.restaurant_holding_gripper = False


class RestaurantArm:
    """Callbacks sin ROS, con backend inyectable y recuperacion tras fallo de agarre."""

    def __init__(self, get_manip_fn, poses, permitted_fn, *, unsafe_fn, warn_fn=lambda _: None):
        self.get_manip = get_manip_fn
        self.poses = poses
        self.permitted = permitted_fn
        self.unsafe = unsafe_fn
        self.warn = warn_fn
        self.lock = threading.Lock()

    def _backend(self):
        if not self.permitted() or not poses_calibrated(self.poses):
            return None
        m = self.get_manip()
        return m if m is not None and m.motion_ready() else None

    def _run(self, fn):
        with self.lock:
            try:
                m = self._backend()
            except Exception as exc:
                self.warn(f"Restaurant: backend de brazo no disponible ({type(exc).__name__})")
                return False
            if m is None:
                return False
            m.abort = False
            done = threading.Event()

            def watch():
                while not done.wait(0.05):
                    if not self.permitted() or not m.motion_ready():
                        m.abort = True
                        return
            watcher = threading.Thread(target=watch, daemon=True)
            watcher.start()
            try:
                return bool(fn(m)) and self.permitted() and not m.abort and m.motion_ready()
            except Exception as exc:
                self.warn(f"Restaurant brazo: {exc}")
                self.unsafe()
                return False
            finally:
                done.set()
                watcher.join(timeout=0.2)

    def detect_item(self, item, timeout_s):
        label = item_yolo_class(item)
        if label is None or not math.isfinite(timeout_s) or timeout_s <= 0:
            return False
        def detect(m):
            m.select_item(label)
            return m.detect_blocking(label, timeout_s=timeout_s)[0]
        return self._run(detect)

    def _recover(self, m):
        for name in ("restaurant_carry", "home"):
            if not self.permitted() or m.abort or not m.motion_ready():
                break
            if name == "restaurant_carry" and not _calibrated(m.poses, name):
                continue
            if name in m.poses and m.go_to_holding(name)[0]:
                return True
        self.warn("No se confirmo carry/home; se detiene la sesion para impedir transporte inseguro.")
        self.unsafe()
        return False

    def pick_item(self, item):
        label = item_yolo_class(item)
        if label is None:
            return False

        def pick(m):
            try:
                m.select_item(label)
                ok = m.go_to_blocking("restaurant_pregrasp")[0]
                ok = ok and m.pick_blocking(label, retreat=False)[0]
                ok = ok and m.go_to_holding("restaurant_postgrasp")[0]  # viaja asi hasta el cliente
            except Exception:
                self._recover(m)
                raise
            if not ok:
                self._recover(m)
            return ok
        return self._run(pick)

    def present_gripper(self):
        """Plan simple 10/11: brazo en restaurant_pregrasp con la pinza abierta para que el barman ponga el objeto."""
        def present(m):
            if not m.go_to_blocking("restaurant_pregrasp")[0]:
                self._recover(m)
                return False
            m.gripper(GRIPPER_OPEN_MM)
            return True
        return self._run(present)

    def grip_item(self):
        """Cierra, confirma por ancho que hay algo entre los dedos y sube a postgrasp (asi viaja al cliente)."""
        def grip(m):
            try:
                m.gripper(GRIPPER_CLOSE_ITEM_MM)
                time.sleep(2.0)
                if float(m.gripper_mm) <= GRIPPER_EMPTY_MAX_MM:
                    m.gripper(GRIPPER_OPEN_MM)  # cerro vacio: no viajar con la pinza cerrada sin objeto
                    self._recover(m)
                    return False
                ok = m.go_to_holding("restaurant_postgrasp")[0]
            except Exception:
                self._recover(m)
                raise
            if not ok:
                self._recover(m)
            return ok
        return self._run(grip)

    def handover(self):
        return self._run(lambda m: m.go_to_holding("restaurant_handover")[0])

    def release(self):
        # give_blocking ya abre y confirma el ancho medido; aqui no vuelve a esperar al cliente.
        return self._run(lambda m: m.give_blocking(wait_s=0.0)[0])

    def stow(self):
        return self._run(self._recover)


def create_restaurant_manipulation(poses, permitted_fn):
    """Mismo backend/nodo que MissionExecutorNode, una sola instancia bajo demanda."""
    from .arm_manipulation import Manipulacion

    class RestaurantManipulacion(RestaurantMotionGuard, Manipulacion):
        def __init__(self):
            self.restaurant_permitted = permitted_fn
            self.restaurant_feedback_at = 0.0
            super().__init__()
            # No reemplazar la HOME ni los limites que Manipulacion acaba de leer.
            for name in RESTAURANT_POSES:
                self.poses.setdefault(name, deepcopy(poses[name]))

    return RestaurantManipulacion()
