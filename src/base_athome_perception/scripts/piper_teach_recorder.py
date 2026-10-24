#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ==============================================================================
# Base@Home - Sistema Interactivo Teach Pendant, Grabación y Reproducción de Trayectorias
# Brazo Robótico AgileX Piper 6-DOF
# RoboCup @Home (Open Platform League)
#
# Modos Principales:
#   1. RECORD: Registro de trayectoria manual / área segura y cálculo de envolvente
#   2. PLAYBACK & VERIFY: Reproducción suave con S-Curve e interbloqueo Hard-Stop
#   3. CALIBRATE: Calibración y almacenamiento de poses nombradas ('home', 'cube_stow', etc.)
# ==============================================================================

import argparse
import datetime
import math
import os
import signal
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple, Union

import yaml

# ROS 2 Imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float32, String, Bool
    from std_srvs.srv import SetBool, Trigger
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

# Piper Kinematics
try:
    from base_athome_perception.piper_kinematics import (
        SAFE_JOINT_LIMITS_DEG,
        SAFE_CARTESIAN_BOX_M,
        DEFAULT_NAMED_POSES,
        is_in_safe_joint_envelope,
        is_in_safe_cartesian_box,
        cosine_s_curve,
        PiperKinematicsSolver,
    )
except ImportError:
    # Fallback si se ejecuta de forma aislada
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from base_athome_perception.piper_kinematics import (
        SAFE_JOINT_LIMITS_DEG,
        SAFE_CARTESIAN_BOX_M,
        DEFAULT_NAMED_POSES,
        is_in_safe_joint_envelope,
        is_in_safe_cartesian_box,
        cosine_s_curve,
        PiperKinematicsSolver,
    )

DEFAULT_CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "config",
    "safe_arm_trajectories.yaml",
)
JETSON_HOST = "ubuntu@10.42.0.12"


# ==============================================================================
# ADAPTADOR DE COMUNICACIÓN CON EL BRAZO (ROS 2 / CAN / JETSON SSH BRIDGE)
# ==============================================================================

class ArmBridge:
    """Adaptador polimórfico de conexión con el brazo Piper (ROS 2 o CAN directo)."""

    def __init__(self, backend: str = "auto", ros_node: Optional[any] = None):
        self.backend = backend
        self.ros_node = ros_node
        self.connected = False
        self.current_joints_deg = [0.0] * 6
        self.current_gripper_mm = 70.0
        self.current_xyz_mm = [0.0, 0.0, 0.0]
        self.is_moving = False
        self.lock = threading.Lock()

        # CAN SDK Directo (si está disponible localmente)
        self.piper_sdk_instance = None

        self._init_connection()

    def _init_connection(self):
        if self.backend in ["auto", "ros2"] and HAS_ROS2 and self.ros_node is not None:
            # Comprobar suscripciones a ROS 2
            self.connected = True
            print("[Bridge] Modo ROS 2 activo: escuchando /joint_states y /arm/status")
        else:
            # Intentar conexión directa o vía Jetson SSH
            self._init_direct_sdk()

    def _init_direct_sdk(self):
        try:
            from piper_sdk import C_PiperInterface_V2
            for port in ["can0", "can1"]:
                try:
                    p = C_PiperInterface_V2(port)
                    p.ConnectPort()
                    time.sleep(0.2)
                    st = p.GetArmStatus().arm_status
                    if st is not None:
                        self.piper_sdk_instance = p
                        self.connected = True
                        print(f"[Bridge] Conexión local piper_sdk activa en {port}!")
                        return
                except Exception:
                    pass
        except ImportError:
            pass

        # Fallback a SSH Jetson si se especifica
        print(f"[Bridge] Interfaz CAN local no disponible. Conectando a Jetson ({JETSON_HOST})...")
        self.connected = True

    def update_from_ros_joint_state(self, msg: JointState):
        with self.lock:
            # Buscar índices joint1..joint6
            name_to_idx = {name: i for i, name in enumerate(msg.name)}
            found = 0
            for i in range(1, 7):
                j_name = f"joint{i}"
                if j_name in name_to_idx:
                    pos = msg.position[name_to_idx[j_name]]
                    # Convertir a grados si está en radianes
                    if abs(pos) <= 3.5:
                        self.current_joints_deg[i - 1] = math.degrees(pos)
                    else:
                        self.current_joints_deg[i - 1] = pos
                    found += 1
            if found < 6 and len(msg.position) >= 6:
                for k in range(6):
                    v = msg.position[k]
                    self.current_joints_deg[k] = math.degrees(v) if abs(v) <= 3.5 else v

            # Gripper (joint7 o posición 6)
            if "joint7" in name_to_idx:
                half_m = msg.position[name_to_idx["joint7"]]
                self.current_gripper_mm = min(70.0, max(0.0, half_m * 2000.0))
            elif len(msg.position) >= 7:
                self.current_gripper_mm = min(70.0, max(0.0, msg.position[6]))

    def update_from_ros_status(self, data: dict):
        with self.lock:
            self.is_moving = data.get("is_moving", self.is_moving)
            self.current_gripper_mm = data.get("gripper_mm", self.current_gripper_mm)
            self.current_joints_deg = data.get("joints_deg", self.current_joints_deg)
            self.current_xyz_mm = data.get("xyz_mm", self.current_xyz_mm)

    def read_telemetry_live(self) -> Tuple[List[float], float]:
        """Obtiene la lectura más reciente de las 6 articulaciones y la pinza."""
        if self.piper_sdk_instance:
            try:
                j_msg = self.piper_sdk_instance.GetArmJointMsgs().joint_state
                raw = [j_msg.joint_1, j_msg.joint_2, j_msg.joint_3, j_msg.joint_4, j_msg.joint_5, j_msg.joint_6]
                g_msg = self.piper_sdk_instance.GetArmGripperMsgs().gripper_state
                with self.lock:
                    self.current_joints_deg = [v / 1000.0 for v in raw]
                    self.current_gripper_mm = g_msg.grippers_angle / 1000.0
            except Exception:
                pass
        with self.lock:
            return list(self.current_joints_deg), float(self.current_gripper_mm)

    def set_teaching_mode(self, free_guide: bool):
        """Habilita o libera motores para guiado físico por el usuario."""
        if free_guide:
            print("\n[!] Modo Teach: Motores liberados para guiado manual por el usuario.")
        else:
            print("\n[*] Modo Hold: Motores re-activados en posición actual.")

        if self.piper_sdk_instance:
            try:
                if free_guide:
                    self.piper_sdk_instance.DisableArm(7)
                else:
                    self.piper_sdk_instance.EnableArm(7)
            except Exception:
                pass

        if self.ros_node and hasattr(self.ros_node, "cli_enable"):
            if self.ros_node.cli_enable.service_is_ready():
                req = SetBool.Request()
                req.data = not free_guide
                self.ros_node.cli_enable.call_async(req)

    def send_joint_cmd(self, joints_deg: List[float], duration_s: float = 1.0, gripper_mm: Optional[float] = None):
        """Envía comando articular al brazo."""
        if self.ros_node and hasattr(self.ros_node, "pub_joint_cmd"):
            js = JointState()
            js.header.stamp = self.ros_node.get_clock().now().to_msg()
            js.name = [f"joint{i}" for i in range(1, 7)]
            js.position = [math.radians(d) for d in joints_deg]
            if gripper_mm is not None:
                js.name.append("gripper")
                js.position.append(gripper_mm)
            js.velocity = [duration_s]
            self.ros_node.pub_joint_cmd.publish(js)

        if self.piper_sdk_instance:
            try:
                raw_int = [int(d * 1000.0) for d in joints_deg]
                self.piper_sdk_instance.MotionCtrl_2(0x01, 0x01, 35, 0x00)
                self.piper_sdk_instance.JointCtrl(*raw_int)
                if gripper_mm is not None:
                    g_int = int(min(70.0, max(0.0, gripper_mm)) * 1000.0)
                    self.piper_sdk_instance.GripperCtrl(g_int, 1500, 0x01, 0)
            except Exception:
                pass


# ==============================================================================
# NODO ROS 2 AUXILIAR PARA PUBLICACIÓN / SUSCRIPCIÓN
# ==============================================================================

class TeachRosNode(Node):
    """Nodo ROS 2 para telemetría continua y control de articulaciones."""

    def __init__(self):
        super().__init__("piper_teach_recorder")
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.bridge: Optional[ArmBridge] = None

        self.sub_joints = self.create_subscription(
            JointState, "/joint_states", self.joint_state_cb, qos
        )
        self.sub_status = self.create_subscription(
            String, "/arm/status", self.status_cb, 10
        )

        self.pub_joint_cmd = self.create_publisher(JointState, "/arm/joint_cmd", 5)
        self.pub_gripper_cmd = self.create_publisher(Float32, "/arm/gripper_cmd", 5)
        self.cli_enable = self.create_client(SetBool, "/arm/set_enable")

    def joint_state_cb(self, msg: JointState):
        if self.bridge:
            self.bridge.update_from_ros_joint_state(msg)

    def status_cb(self, msg: String):
        import json
        try:
            data = json.loads(msg.data)
            if self.bridge:
                self.bridge.update_from_ros_status(data)
        except Exception:
            pass


# ==============================================================================
# GESTOR DEL ARCHIVO YAML DE ENVOLVENTES Y TRAYECTORIAS
# ==============================================================================

class TrajectoryStorage:
    """Gestiona la carga y persistencia en safe_arm_trajectories.yaml."""

    def __init__(self, file_path: str = DEFAULT_CONFIG_PATH):
        self.file_path = file_path
        self.data: dict = {
            "safety_envelope": {
                "joint_limits_deg": SAFE_JOINT_LIMITS_DEG,
                "cartesian_limits_m": SAFE_CARTESIAN_BOX_M,
                "hard_stop_margin_deg": 2.0,
            },
            "named_poses": dict(DEFAULT_NAMED_POSES),
            "trajectories": {},
        }
        self.load()

    def load(self):
        if os.path.exists(self.file_path):
            try:
                with open(self.file_path, "r", encoding="utf-8") as f:
                    content = yaml.safe_load(f)
                    if isinstance(content, dict):
                        self.data.update(content)
                print(f"[*] Archivo de configuracion cargado: {self.file_path}")
            except Exception as e:
                print(f"[!] Error leyendo YAML ({e}), usando configuracion por defecto.")

    def save(self):
        os.makedirs(os.path.dirname(os.path.abspath(self.file_path)), exist_ok=True)
        try:
            with open(self.file_path, "w", encoding="utf-8") as f:
                yaml.dump(self.data, f, sort_keys=False, indent=2, allow_unicode=True)
            print(f"[+] Archivo guardado con exito en: {self.file_path}")
            return True
        except Exception as e:
            print(f"[ERROR] No se pudo guardar archivo YAML: {e}")
            return False

    def get_joint_envelope(self) -> Dict[str, Tuple[float, float]]:
        env = self.data.get("safety_envelope", {}).get("joint_limits_deg", SAFE_JOINT_LIMITS_DEG)
        res = {}
        for k, v in env.items():
            res[k] = (float(v[0]), float(v[1]))
        return res

    def save_recorded_trajectory(self, name: str, waypoints: List[dict], rate_hz: float, computed_envelope: dict):
        duration = len(waypoints) / max(1.0, rate_hz)
        self.data["trajectories"][name] = {
            "metadata": {
                "created_at": datetime.datetime.utcnow().isoformat() + "Z",
                "point_count": len(waypoints),
                "rate_hz": float(rate_hz),
                "duration_sec": round(duration, 2),
            },
            "waypoints": waypoints,
        }

        # Actualizar envolvente calculada en safety_envelope
        if "recorded_envelopes" not in self.data["safety_envelope"]:
            self.data["safety_envelope"]["recorded_envelopes"] = {}
        self.data["safety_envelope"]["recorded_envelopes"][name] = computed_envelope

        self.save()

    def save_named_pose(self, name: str, joints_deg: List[float], gripper_mm: float, description: str = ""):
        self.data["named_poses"][name] = {
            "description": description or f"Pose calibrada {name}",
            "joints_deg": [round(float(v), 2) for v in joints_deg],
            "joints_rad": [round(math.radians(float(v)), 4) for v in joints_deg],
            "gripper_mm": round(float(gripper_mm), 1),
        }
        self.save()


# ==============================================================================
# PIPER TEACH RECORDER - CONTROLADOR PRINCIPAL
# ==============================================================================

class PiperTeachRecorder:
    """Controlador de grabación, verificación y calibración del brazo Piper."""

    def __init__(self, config_path: str = DEFAULT_CONFIG_PATH):
        self.storage = TrajectoryStorage(config_path)
        self.solver = PiperKinematicsSolver()

        self.running = True
        self.ros_node: Optional[TeachRosNode] = None
        self.ros_thread: Optional[threading.Thread] = None

        if HAS_ROS2:
            try:
                rclpy.init()
                self.ros_node = TeachRosNode()
                self.bridge = ArmBridge(backend="auto", ros_node=self.ros_node)
                self.ros_node.bridge = self.bridge

                def spin_loop():
                    while self.running and rclpy.ok():
                        try:
                            rclpy.spin_once(self.ros_node, timeout_sec=0.02)
                        except Exception:
                            break

                self.ros_thread = threading.Thread(target=spin_loop, daemon=True)
                self.ros_thread.start()
            except Exception as e:
                print(f"[Aviso ROS 2]: {e}")
                self.bridge = ArmBridge(backend="can")
        else:
            self.bridge = ArmBridge(backend="can")

        # Esperar 0.3s para recepción inicial de telemetría
        time.sleep(0.3)

    def shutdown(self):
        self.running = False
        time.sleep(0.05)
        if HAS_ROS2 and self.ros_node:
            try:
                self.ros_node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass

    # --------------------------------------------------------------------------
    # MODO 1: GRABAR TRAYECTORIA SEGURA / CALCULAR ENVOLVENTE
    # --------------------------------------------------------------------------
    def record_trajectory_mode(self, traj_name: Optional[str] = None, rate_hz: float = 10.0):
        print("\n==========================================================================")
        print("   MODO 1: REGISTRO DE TRAYECTORIA SEGURA Y CALCULO DE ENVOLVENTE       ")
        print("==========================================================================")
        print(" Instrucciones:")
        print("   1. Al iniciar la grabacion, los motores se liberaran para permitir")
        print("      guiado fisico manual por el operador.")
        print("   2. Mueva el extremo del brazo suavemente a traves del 'area segura'")
        print("      frente al robot y sobre la mesa, sin golpear la columna o pantalla.")
        print("   3. Presione [ENTER] para comenzar la grabacion.")
        print("   4. Presione [ENTER] nuevamente o [Ctrl+C] para finalizar.")
        print("--------------------------------------------------------------------------")

        input(" Presione [ENTER] cuando este listo para comenzar a guiar el brazo...")

        # Liberar motores para guiado físico
        self.bridge.set_teaching_mode(free_guide=True)

        period = 1.0 / max(1.0, rate_hz)
        waypoints: List[dict] = []

        start_time = time.time()
        stop_recording = False

        print(f"\n[*] Grabando waypoints a {rate_hz:.1f} Hz... (Presione [ENTER] para detener)")

        # Hilo de detección de teclado no bloqueante
        def wait_for_enter():
            nonlocal stop_recording
            try:
                sys.stdin.readline()
            except Exception:
                pass
            stop_recording = True

        t_key = threading.Thread(target=wait_for_enter, daemon=True)
        t_key.start()

        try:
            while not stop_recording:
                t_now = time.time() - start_time
                joints, grip = self.bridge.read_telemetry_live()

                # Guardar waypoint
                wp = {
                    "t": round(t_now, 3),
                    "joints_deg": [round(v, 2) for v in joints],
                    "gripper_mm": round(grip, 1),
                }
                waypoints.append(wp)

                # Calcular cinemática directa actual
                rads = [math.radians(v) for v in joints]
                xyz_m = self.solver.forward_position_m(rads)

                # Barra en tiempo real
                j_str = ", ".join(f"{v:6.1f}°" for v in joints)
                print(
                    f"\r   [{t_now:5.1f}s | #{len(waypoints):4d}] J:[{j_str}] | Pinza: {grip:4.1f}mm | XYZ: [{xyz_m[0]:.2f}, {xyz_m[1]:.2f}, {xyz_m[2]:.2f}]m",
                    end="",
                    flush=True,
                )

                time.sleep(period)
        except KeyboardInterrupt:
            print("\n[*] Grabacion interrumpida por teclado.")

        # Re-activar motores en posición actual
        self.bridge.set_teaching_mode(free_guide=False)
        print(f"\n\n[+] Grabacion finalizada. Se registraron {len(waypoints)} waypoints.")

        if len(waypoints) < 3:
            print("[!] Muy pocos waypoints registrados. Descartando trayectoria.")
            return

        # Calcular Envolvente Bounding [min, max] para cada articulación
        print("\n--------------------------------------------------------------------------")
        print("          ENVOLVENTE ARTICULAR RECOGIDA vs LIMITES DE SEGURIDAD           ")
        print("--------------------------------------------------------------------------")
        joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        computed_env = {}

        allowed_limits = self.storage.get_joint_envelope()

        for i, name in enumerate(joint_names):
            vals = [wp["joints_deg"][i] for wp in waypoints]
            j_min = min(vals)
            j_max = max(vals)
            computed_env[name] = [round(j_min, 2), round(j_max, 2)]

            allow_min, allow_max = allowed_limits.get(name, (-180.0, 180.0))
            is_safe = (j_min >= allow_min - 1.0) and (j_max <= allow_max + 1.0)
            status_tag = "[OK - SEGURO]" if is_safe else "[AVISO: FUERA DE RANGO]"

            print(
                f"   {name:<7}: Registrado: [{j_min:6.1f}°, {j_max:6.1f}°]  |  Permitido: [{allow_min:6.1f}°, {allow_max:6.1f}°]  {status_tag}"
            )

        print("--------------------------------------------------------------------------")

        if traj_name is None:
            default_name = f"teach_safe_{datetime.datetime.now().strftime('%H%M%S')}"
            choice = input(f" Ingrese nombre para guardar la trayectoria [{default_name}]: ").strip()
            traj_name = choice if choice else default_name

        self.storage.save_recorded_trajectory(
            name=traj_name,
            waypoints=waypoints,
            rate_hz=rate_hz,
            computed_envelope=computed_env,
        )
        print(f"[+] Trayectoria '{traj_name}' y envolvente guardadas exitosamente en YAML.")

    # --------------------------------------------------------------------------
    # MODO 2: REPRODUCCIÓN & VERIFICACIÓN CON S-CURVE Y HARD STOP
    # --------------------------------------------------------------------------
    def playback_trajectory_mode(
        self,
        traj_name: Optional[str] = None,
        speed_scale: float = 0.5,
        verify_only: bool = False,
    ):
        print("\n==========================================================================")
        print("   MODO 2: VERIFICACION Y REPRODUCCION SUAVE CON S-CURVE E INTERBLOQUEO   ")
        print("==========================================================================")

        trajs = self.storage.data.get("trajectories", {})
        if not trajs:
            print("[!] No hay trayectorias guardadas en el archivo YAML.")
            return

        if traj_name is None or traj_name not in trajs:
            print(" Trayectorias disponibles:")
            traj_keys = list(trajs.keys())
            for idx, k in enumerate(traj_keys, 1):
                dur = trajs[k].get("metadata", {}).get("duration_sec", 0.0)
                pts = trajs[k].get("metadata", {}).get("point_count", 0)
                print(f"   [{idx}] {k} ({pts} waypoints, {dur:.1f}s)")

            sel = input(f" Seleccione numero o nombre de trayectoria [1]: ").strip()
            if not sel:
                traj_name = traj_keys[0]
            elif sel.isdigit() and 1 <= int(sel) <= len(traj_keys):
                traj_name = traj_keys[int(sel) - 1]
            elif sel in trajs:
                traj_name = sel
            else:
                print("[!] Opcion no valida.")
                return

        target_traj = trajs[traj_name]
        raw_waypoints = target_traj.get("waypoints", [])
        if len(raw_waypoints) < 2:
            print("[!] Trayectoria invalida o vacia.")
            return

        envelope = self.storage.get_joint_envelope()
        margin_deg = float(self.storage.data.get("safety_envelope", {}).get("hard_stop_margin_deg", 2.0))

        # --- 1. FASE DE VERIFICACIÓN PRE-VUELO ---
        print(f"\n[*] Verificando trayectoria '{traj_name}' contra limites de seguridad...")
        has_violations = False
        for idx, wp in enumerate(raw_waypoints):
            safe, reason = is_in_safe_joint_envelope(wp["joints_deg"], margin_deg=margin_deg, envelope=envelope)
            if not safe:
                print(f"   [PELIGRO] Waypoint #{idx} (t={wp.get('t', 0.0):.2f}s) viola envolvente: {reason}")
                has_violations = True

            # Verificar límites cartesianos calculando FK
            rads = [math.radians(v) for v in wp["joints_deg"]]
            xyz_m = self.solver.forward_position_m(rads)
            c_safe, c_reason = is_in_safe_cartesian_box(xyz_m[0], xyz_m[1], xyz_m[2])
            if not c_safe:
                print(f"   [AVISO CARTESIANO] Waypoint #{idx} fuera de caja de trabajo: {c_reason}")

        if has_violations:
            print("\n[BLOQUEADO] La trayectoria CONTIENE VIOLACIONES de seguridad articular.")
            print("[HARD-STOP] Reproduccion cancelada automaticamente para proteger el robot.")
            return

        print("[+] Verificacion completada: Todos los waypoints estan dentro del Area Segura!")

        if verify_only:
            print("[*] Modo --verify-only activo. Saltando ejecucion fisica.")
            return

        # --- 2. FASE DE REPRODUCCIÓN FÍSICA ---
        print(f"\n[*] Factor de velocidad de reproduccion: {speed_scale:.2f}x")
        confirm = input(" ¿Desea ejecutar el movimiento fisico del brazo? [s/N]: ").strip().lower()
        if confirm not in ["s", "si", "y", "yes"]:
            print("[*] Reproduccion cancelada por el usuario.")
            return

        print("\n[*] Moviendo brazo suavemente al primer waypoint de la trayectoria...")
        first_wp = raw_waypoints[0]
        self.bridge.send_joint_cmd(first_wp["joints_deg"], duration_s=2.5, gripper_mm=first_wp.get("gripper_mm", 70.0))
        time.sleep(2.6)

        print("[*] Ejecutando reproduccion continua con interpolacion Cosine S-Curve a 50 Hz...")

        control_rate_hz = 50.0
        dt = 1.0 / control_rate_hz

        try:
            for k in range(len(raw_waypoints) - 1):
                wp_curr = raw_waypoints[k]
                wp_next = raw_waypoints[k + 1]

                delta_t_orig = max(0.05, wp_next.get("t", 0.0) - wp_curr.get("t", 0.0))
                segment_duration = max(0.1, delta_t_orig / max(0.1, speed_scale))
                steps = max(2, int(segment_duration * control_rate_hz))

                for step in range(steps):
                    alpha = (step + 1) / steps
                    # Interpolación Cosine S-Curve
                    interp_joints = [
                        cosine_s_curve(wp_curr["joints_deg"][j], wp_next["joints_deg"][j], alpha)
                        for j in range(6)
                    ]
                    interp_grip = cosine_s_curve(
                        wp_curr.get("gripper_mm", 70.0),
                        wp_next.get("gripper_mm", 70.0),
                        alpha
                    )

                    # Interbloqueo Hard Stop en tiempo real
                    safe, reason = is_in_safe_joint_envelope(interp_joints, margin_deg=margin_deg, envelope=envelope)
                    if not safe:
                        print(f"\n[HARD STOP INMEDIATO] Trayectoria abortada a mitad de ejecucion: {reason}")
                        # Detener brazo en su posición actual
                        cur_j, cur_g = self.bridge.read_telemetry_live()
                        self.bridge.send_joint_cmd(cur_j, duration_s=0.2, gripper_mm=cur_g)
                        return

                    self.bridge.send_joint_cmd(interp_joints, duration_s=dt * 1.5, gripper_mm=interp_grip)

                    j_str = ", ".join(f"{v:5.1f}°" for v in interp_joints)
                    print(
                        f"\r   Segmento [{k+1}/{len(raw_waypoints)-1}] {alpha*100:3.0f}% | J:[{j_str}] | Pinza: {interp_grip:4.1f}mm",
                        end="",
                        flush=True,
                    )
                    time.sleep(dt)

            print("\n\n[+] Reproduccion de trayectoria finalizada con EXITO!")

        except KeyboardInterrupt:
            print("\n[!] Movimiento interrumpido por el operador. Deteniendo brazo...")
            cur_j, cur_g = self.bridge.read_telemetry_live()
            self.bridge.send_joint_cmd(cur_j, duration_s=0.2, gripper_mm=cur_g)

    # --------------------------------------------------------------------------
    # MODO 3: CALIBRAR POSES NOMBRADAS ('home', 'cube_stow', etc.)
    # --------------------------------------------------------------------------
    def calibrate_poses_mode(self):
        print("\n==========================================================================")
        print("   MODO 3: CALIBRACION INTERACTIVA DE POSES NOMBRADAS DEL BRAZO          ")
        print("==========================================================================")
        print(" Poses estandar requeridas:")
        print("   1. 'home'             : Reposo estibado seguro junto a la columna")
        print("   2. 'cube_stow'        : Estiba lateral pegada al cubo de montaje")
        print("   3. 'pre_grasp_center' : Frente al espacio de trabajo / mesa central")
        print("   4. 'pre_grasp_table'  : Pose baja de aproximacion sobre la mesa")
        print("--------------------------------------------------------------------------")

        # Permitir liberar motores
        rel = input(" ¿Desea liberar los motores para mover el brazo manualmente? [s/N]: ").strip().lower()
        if rel in ["s", "si", "y", "yes"]:
            self.bridge.set_teaching_mode(free_guide=True)

        try:
            while True:
                joints, grip = self.bridge.read_telemetry_live()
                rads = [math.radians(v) for v in joints]
                xyz_m = self.solver.forward_position_m(rads)

                print("\n--------------------------------------------------------------------------")
                print(" POSICION FISICA ACTUAL:")
                for i, deg in enumerate(joints, 1):
                    print(f"   Joint {i}: {deg:8.2f}° ({math.radians(deg):8.4f} rad)")
                print(f"   Pinza  : {grip:8.2f} mm")
                print(f"   Extremo: X={xyz_m[0]:.3f}m, Y={xyz_m[1]:.3f}m, Z={xyz_m[2]:.3f}m (en arm_base_link)")
                print("--------------------------------------------------------------------------")
                print(" Opciones de guardado:")
                print("   [1] Guardar como 'home'")
                print("   [2] Guardar como 'cube_stow'")
                print("   [3] Guardar como 'pre_grasp_center'")
                print("   [4] Guardar como 'pre_grasp_table'")
                print("   [5] Guardar con nombre personalizado")
                print("   [R] Refrescar lectura actual")
                print("   [Q] Salir de calibracion")
                print("--------------------------------------------------------------------------")

                opt = input(" Seleccione una opcion: ").strip().lower()
                if opt == "1":
                    self.storage.save_named_pose("home", joints, grip, "Pose de reposo seguro estibado junto a columna")
                    print("[+] Pose 'home' calibrada y guardada.")
                elif opt == "2":
                    self.storage.save_named_pose("cube_stow", joints, grip, "Pose compacta de estiba lateral junto al cubo")
                    print("[+] Pose 'cube_stow' calibrada y guardada.")
                elif opt == "3":
                    self.storage.save_named_pose("pre_grasp_center", joints, grip, "Pose frontal pre-agarre hacia centro de mesa")
                    print("[+] Pose 'pre_grasp_center' calibrada y guardada.")
                elif opt == "4":
                    self.storage.save_named_pose("pre_grasp_table", joints, grip, "Pose baja de aproximacion sobre mesa de servicio")
                    print("[+] Pose 'pre_grasp_table' calibrada y guardada.")
                elif opt == "5":
                    custom_name = input(" Ingrese nombre de la pose: ").strip()
                    if custom_name:
                        desc = input(" Ingrese descripcion opcional: ").strip()
                        self.storage.save_named_pose(custom_name, joints, grip, desc)
                        print(f"[+] Pose '{custom_name}' calibrada y guardada.")
                elif opt == "r":
                    continue
                elif opt in ["q", "quit", "exit"]:
                    break
                else:
                    print("[!] Opcion no reconocida.")

        finally:
            self.bridge.set_teaching_mode(free_guide=False)
            print("[*] Salida del modo de calibracion.")


# ==============================================================================
# PUNTO DE ENTRADA CLI
# ==============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Base@Home - Teach Pendant, Registro y Verificacion de Trayectorias para Piper 6-DOF"
    )
    parser.add_argument(
        "--mode",
        choices=["record", "playback", "calibrate", "inspect"],
        default=None,
        help="Modo directo de ejecucion (record, playback, calibrate, inspect)",
    )
    parser.add_argument(
        "--name",
        type=str,
        default=None,
        help="Nombre de la trayectoria o pose para grabacion/reproduccion",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=10.0,
        help="Frecuencia de muestreo en Hz para grabacion (default: 10 Hz)",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=0.5,
        help="Factor de escalamiento de velocidad en reproduccion (0.1 a 1.5, default: 0.5)",
    )
    parser.add_argument(
        "--verify-only",
        action="store_true",
        help="En modo playback, realiza solo la verificacion de limites sin mover el brazo",
    )
    parser.add_argument(
        "--config",
        type=str,
        default=DEFAULT_CONFIG_PATH,
        help="Ruta alternativa al archivo safe_arm_trajectories.yaml",
    )

    args = parser.parse_args()

    controller = PiperTeachRecorder(config_path=args.config)

    try:
        if args.mode == "record":
            controller.record_trajectory_mode(traj_name=args.name, rate_hz=args.rate)
        elif args.mode == "playback":
            controller.playback_trajectory_mode(
                traj_name=args.name,
                speed_scale=args.speed,
                verify_only=args.verify_only,
            )
        elif args.mode == "calibrate":
            controller.calibrate_poses_mode()
        elif args.mode == "inspect":
            print(f"\n[*] Contenido de {args.config}:")
            with open(args.config, "r", encoding="utf-8") as f:
                print(f.read())
        else:
            # Menu interactivo general
            while True:
                print("\n==========================================================================")
                print("   BASE@HOME - PIPER 6-DOF TEACH PENDANT & TRAJECTORY MANAGER            ")
                print("==========================================================================")
                print("   [1] Grabar Trayectoria Manual / Area Segura (Calcular Envolvente)     ")
                print("   [2] Reproducir & Verificar Trayectoria (S-Curve & Hard-Stop)          ")
                print("   [3] Calibrar Poses Nombradas ('home', 'cube_stow', 'pre_grasp', etc.) ")
                print("   [4] Inspeccionar Archivo de Configuracion YAML                        ")
                print("   [Q] Salir                                                             ")
                print("==========================================================================")

                choice = input(" Seleccione una opcion [1-4 / Q]: ").strip().lower()
                if choice == "1":
                    controller.record_trajectory_mode(rate_hz=args.rate)
                elif choice == "2":
                    controller.playback_trajectory_mode(speed_scale=args.speed, verify_only=args.verify_only)
                elif choice == "3":
                    controller.calibrate_poses_mode()
                elif choice == "4":
                    with open(args.config, "r", encoding="utf-8") as f:
                        print(f.read())
                elif choice in ["q", "quit", "exit"]:
                    break
                else:
                    print("[!] Opcion no valida.")

    finally:
        controller.shutdown()


if __name__ == "__main__":
    main()
