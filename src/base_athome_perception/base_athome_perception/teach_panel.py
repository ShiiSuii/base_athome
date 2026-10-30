"""Panel visual OpenCV para la ensenanza de poses del brazo robotico Piper.

Permite visualizar poses guardadas (HOME, PRE_GRASP, POST_GRASP), monitorear
articulaciones actuales y diferencias angulares, entrar en modo ensenanza,
guardar/cancelar nuevas poses y controlar torque con atajos de teclado y clics.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union

import cv2
import numpy as np


@dataclass
class ButtonRect:
    """Rectangulo interactivo de un boton en el panel."""

    name: str
    rect: Tuple[int, int, int, int]  # (x, y, w, h)
    action: Tuple[str, Optional[str]]
    enabled: bool = True
    bg_color: Tuple[int, int, int] = (60, 60, 60)
    fg_color: Tuple[int, int, int] = (255, 255, 255)


class TeachPanel:
    """Panel vertical interactivo para ensenar poses de manipulacion."""

    FONT = cv2.FONT_HERSHEY_SIMPLEX
    FONT_SCALE_BASE = 0.45

    def __init__(
        self,
        width_px: int = 340,
        poses: Sequence[str] = ("home", "pre_grasp", "post_grasp"),
    ) -> None:
        """Inicializa el panel de ensenanza."""
        self.width_px = int(width_px)
        self.poses: Tuple[str, ...] = tuple(poses)
        self.buttons: List[ButtonRect] = []
        self._last_state: Dict[str, Any] = {}
        self._last_origin: Tuple[int, int] = (0, 0)
        self._panel_height: int = 500

    def render(
        self,
        canvas: np.ndarray,
        x0: int,
        y0: int,
        state: Optional[Dict[str, Any]],
    ) -> None:
        """Dibuja el panel en el canvas a partir de (x0, y0).

        Recalcula las posiciones de los botones y guarda su estado habilitado.
        """
        if canvas is None or not isinstance(canvas, np.ndarray):
            return

        state = state or {}
        self._last_state = state
        self._last_origin = (x0, y0)
        self.buttons.clear()

        feedback_ok: bool = bool(state.get("feedback_ok", False))
        torque_on: bool = bool(state.get("torque_on", False))
        teaching_pose: Optional[str] = state.get("teaching")
        saved_poses: Dict[str, Any] = state.get("saved_poses") or {}
        current_joints = state.get("current_joints_deg")
        current_gripper = state.get("current_gripper_mm")

        # Calculo dinamico de altura segun secciones activas
        extra_teaching = 66 if teaching_pose else 0
        poses_count = len(self.poses)
        total_h = 245 + extra_teaching + (poses_count * 74)
        self._panel_height = total_h

        # Fondo del panel (charcoal oscuro) y borde
        x1 = x0 + self.width_px
        y1 = y0 + total_h
        cv2.rectangle(canvas, (x0, y0), (x1, y1), (28, 28, 28), -1)
        cv2.rectangle(canvas, (x0, y0), (x1, y1), (80, 80, 80), 1)

        curr_y = y0 + 6

        # --- 1. Titulo ---
        title_h = 28
        cv2.rectangle(
            canvas,
            (x0 + 4, curr_y),
            (x1 - 4, curr_y + title_h),
            (45, 45, 55),
            -1,
        )
        cv2.putText(
            canvas,
            "ENSENAR POSES",
            (x0 + 12, curr_y + 19),
            self.FONT,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        curr_y += title_h + 6

        # --- 2. Estado de feedback del brazo ---
        status_h = 22
        if not feedback_ok:
            cv2.rectangle(
                canvas,
                (x0 + 6, curr_y),
                (x1 - 6, curr_y + status_h),
                (30, 30, 200),
                -1,
            )
            cv2.putText(
                canvas,
                "SIN DATOS DEL BRAZO",
                (x0 + 14, curr_y + 16),
                self.FONT,
                0.48,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
        else:
            cv2.rectangle(
                canvas,
                (x0 + 6, curr_y),
                (x1 - 6, curr_y + status_h),
                (35, 65, 35),
                -1,
            )
            cv2.putText(
                canvas,
                "BRAZO CONECTADO (FEEDBACK OK)",
                (x0 + 14, curr_y + 15),
                self.FONT,
                0.45,
                (120, 240, 120),
                1,
                cv2.LINE_AA,
            )
        curr_y += status_h + 6

        # --- 3. Botones globales de Torque ---
        btn_w = (self.width_px - 20) // 2
        btn_h = 24

        # Boton LIBERAR BRAZO (torque off)
        self._add_button(
            canvas=canvas,
            name="torque_off",
            label="LIBERAR BRAZO",
            rect=(x0 + 7, curr_y, btn_w, btn_h),
            action=("torque_off", None),
            enabled=True,
            bg_color=(35, 90, 195),
            fg_color=(255, 255, 255),
        )

        # Boton BLOQUEAR BRAZO (torque on)
        self._add_button(
            canvas=canvas,
            name="torque_on",
            label="BLOQUEAR BRAZO",
            rect=(x0 + 13 + btn_w, curr_y, btn_w, btn_h),
            action=("torque_on", None),
            enabled=True,
            bg_color=(175, 110, 40),
            fg_color=(255, 255, 255),
        )
        curr_y += btn_h + 8

        # --- 4. Banner de Ensenanza Activa (si aplica) ---
        if teaching_pose:
            banner_h = 36
            cv2.rectangle(
                canvas,
                (x0 + 6, curr_y),
                (x1 - 6, curr_y + banner_h),
                (20, 110, 220),
                -1,
            )
            cv2.putText(
                canvas,
                "BRAZO LIBRE: MOVELO A MANO",
                (x0 + 12, curr_y + 15),
                self.FONT,
                0.48,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            mode_txt = f"Ensenando: {teaching_pose.upper()}"
            torque_txt = "TORQUE: ON" if torque_on else "TORQUE: OFF"
            cv2.putText(
                canvas,
                f"{mode_txt} | {torque_txt}",
                (x0 + 12, curr_y + 30),
                self.FONT,
                0.45,
                (240, 240, 240),
                1,
                cv2.LINE_AA,
            )
            curr_y += banner_h + 4

            # Botones GUARDAR y CANCELAR
            self._add_button(
                canvas=canvas,
                name="save",
                label="GUARDAR",
                rect=(x0 + 7, curr_y, btn_w, btn_h),
                action=("save", None),
                enabled=feedback_ok,
                bg_color=(40, 160, 40),
                fg_color=(255, 255, 255),
            )
            self._add_button(
                canvas=canvas,
                name="cancel",
                label="TERMINAR",
                rect=(x0 + 13 + btn_w, curr_y, btn_w, btn_h),
                action=("cancel", None),
                enabled=True,
                bg_color=(40, 40, 180),
                fg_color=(255, 255, 255),
            )
            curr_y += btn_h + 8

        # --- 5. Fila de articulaciones ACTUALES y diferencias ---
        act_box_h = 58
        cv2.rectangle(
            canvas,
            (x0 + 6, curr_y),
            (x1 - 6, curr_y + act_box_h),
            (38, 38, 38),
            -1,
        )
        cv2.rectangle(
            canvas,
            (x0 + 6, curr_y),
            (x1 - 6, curr_y + act_box_h),
            (60, 60, 60),
            1,
        )

        grip_txt = (
            f"Pinza: {current_gripper:.1f} mm"
            if isinstance(current_gripper, (int, float))
            else "Pinza: --"
        )
        cv2.putText(
            canvas,
            f"ACTUAL | {grip_txt}",
            (x0 + 12, curr_y + 15),
            self.FONT,
            0.45,
            (220, 220, 140),
            1,
            cv2.LINE_AA,
        )

        # Valores articulares actuales
        if (
            isinstance(current_joints, (list, tuple))
            and len(current_joints) == 6
            and feedback_ok
        ):
            j_str = " ".join(f"{float(v):.1f}" for v in current_joints)
            cv2.putText(
                canvas,
                f"J: {j_str}",
                (x0 + 12, curr_y + 32),
                self.FONT,
                0.45,
                (240, 240, 240),
                1,
                cv2.LINE_AA,
            )
        else:
            cv2.putText(
                canvas,
                "J: -- (sin lectura)",
                (x0 + 12, curr_y + 32),
                self.FONT,
                0.45,
                (140, 140, 140),
                1,
                cv2.LINE_AA,
            )

        # Calculo de diferencia maxima contra cada pose guardada
        diff_x = x0 + 12
        cv2.putText(
            canvas,
            "Diff:",
            (diff_x, curr_y + 49),
            self.FONT,
            0.45,
            (180, 180, 180),
            1,
            cv2.LINE_AA,
        )
        diff_x += 42

        for p_name in self.poses:
            p_data = saved_poses.get(p_name)
            p_label = p_name.replace("_grasp", "").upper()
            if (
                isinstance(p_data, dict)
                and "joints_deg" in p_data
                and isinstance(current_joints, (list, tuple))
                and len(current_joints) == 6
                and feedback_ok
            ):
                s_joints = p_data["joints_deg"]
                try:
                    max_diff = max(
                        abs(float(current_joints[i]) - float(s_joints[i]))
                        for i in range(6)
                    )
                    color = (
                        (60, 240, 60) if max_diff < 2.0 else (200, 200, 200)
                    )
                    txt = f"{p_label}:{max_diff:.1f}d"
                except Exception:
                    color = (130, 130, 130)
                    txt = f"{p_label}:--"
            else:
                color = (130, 130, 130)
                txt = f"{p_label}:--"

            cv2.putText(
                canvas,
                txt,
                (diff_x, curr_y + 49),
                self.FONT,
                0.45,
                color,
                1,
                cv2.LINE_AA,
            )
            (w, _), _ = cv2.getTextSize(txt, self.FONT, 0.45, 1)
            diff_x += w + 8

        curr_y += act_box_h + 8

        # --- 6. Filas por cada Pose ---
        for pose_name in self.poses:
            pose_box_h = 66
            cv2.rectangle(
                canvas,
                (x0 + 6, curr_y),
                (x1 - 6, curr_y + pose_box_h),
                (34, 34, 34),
                -1,
            )
            cv2.rectangle(
                canvas,
                (x0 + 6, curr_y),
                (x1 - 6, curr_y + pose_box_h),
                (55, 55, 55),
                1,
            )

            # Nombre de la pose
            p_data = saved_poses.get(pose_name)
            p_title = pose_name.upper()
            cv2.putText(
                canvas,
                p_title,
                (x0 + 12, curr_y + 16),
                self.FONT,
                0.48,
                (255, 215, 0),
                1,
                cv2.LINE_AA,
            )

            # Botones IR y ENSEÑAR para esta pose
            ir_w = 44
            ens_w = 78
            btn_py = curr_y + 3

            # Boton [IR]
            self._add_button(
                canvas=canvas,
                name=f"go_{pose_name}",
                label="IR",
                rect=(x1 - 14 - ens_w - ir_w, btn_py, ir_w, 20),
                action=("go", pose_name),
                enabled=(feedback_ok and bool(p_data)),
                bg_color=(160, 95, 30),
                fg_color=(255, 255, 255),
            )

            # Boton [ENSEÑAR]
            self._add_button(
                canvas=canvas,
                name=f"teach_{pose_name}",
                label="GRABAR",
                rect=(x1 - 10 - ens_w, btn_py, ens_w, 20),
                action=("teach", pose_name),
                enabled=feedback_ok,
                bg_color=(150, 60, 140),
                fg_color=(255, 255, 255),
            )

            # Informacion de la pose guardada
            if isinstance(p_data, dict) and "joints_deg" in p_data:
                j_vals = p_data["joints_deg"]
                j_txt = " ".join(f"{float(v):.1f}" for v in j_vals)
                cv2.putText(
                    canvas,
                    f"J: {j_txt}",
                    (x0 + 12, curr_y + 36),
                    self.FONT,
                    0.45,
                    (220, 220, 220),
                    1,
                    cv2.LINE_AA,
                )
                g_val = p_data.get("gripper_mm", 0.0)
                ts_val = p_data.get("timestamp", "-")
                cv2.putText(
                    canvas,
                    f"Pinza: {float(g_val):.1f}mm | {ts_val}",
                    (x0 + 12, curr_y + 54),
                    self.FONT,
                    0.45,
                    (170, 170, 170),
                    1,
                    cv2.LINE_AA,
                )
            else:
                cv2.putText(
                    canvas,
                    "(sin pose guardada)",
                    (x0 + 12, curr_y + 40),
                    self.FONT,
                    0.45,
                    (130, 130, 130),
                    1,
                    cv2.LINE_AA,
                )

            curr_y += pose_box_h + 6

        # --- 7. Guia de Atajos / Ayuda ---
        cv2.line(
            canvas,
            (x0 + 8, curr_y + 2),
            (x1 - 8, curr_y + 2),
            (60, 60, 60),
            1,
        )
        curr_y += 16
        cv2.putText(
            canvas,
            "T: libre | 1/2/3: grabar o ir",
            (x0 + 10, curr_y),
            self.FONT,
            0.45,
            (170, 170, 170),
            1,
            cv2.LINE_AA,
        )
        curr_y += 16
        cv2.putText(
            canvas,
            "Enter: fin | I: MoveIt | M: calib",
            (x0 + 10, curr_y),
            self.FONT,
            0.45,
            (170, 170, 170),
            1,
            cv2.LINE_AA,
        )

    def _add_button(
        self,
        canvas: np.ndarray,
        name: str,
        label: str,
        rect: Tuple[int, int, int, int],
        action: Tuple[str, Optional[str]],
        enabled: bool,
        bg_color: Tuple[int, int, int],
        fg_color: Tuple[int, int, int],
    ) -> None:
        """Dibuja un boton y lo registra en la lista de botones interactivos."""
        x, y, w, h = rect
        btn = ButtonRect(
            name=name,
            rect=rect,
            action=action,
            enabled=enabled,
            bg_color=bg_color,
            fg_color=fg_color,
        )
        self.buttons.append(btn)

        # Colores segun estado
        draw_bg = bg_color if enabled else (50, 50, 50)
        draw_fg = fg_color if enabled else (120, 120, 120)
        draw_border = (80, 80, 80) if enabled else (55, 55, 55)

        cv2.rectangle(canvas, (x, y), (x + w, y + h), draw_bg, -1)
        cv2.rectangle(canvas, (x, y), (x + w, y + h), draw_border, 1)

        # Centrar texto en el boton
        (tw, th), _ = cv2.getTextSize(label, self.FONT, self.FONT_SCALE_BASE, 1)
        tx = x + max(2, (w - tw) // 2)
        ty = y + max(th + 2, (h + th) // 2)
        cv2.putText(
            canvas,
            label,
            (tx, ty),
            self.FONT,
            self.FONT_SCALE_BASE,
            draw_fg,
            1,
            cv2.LINE_AA,
        )

    def handle_click(
        self, x: int, y: int
    ) -> Optional[Tuple[str, Optional[str]]]:
        """Procesa un clic del raton en coordenadas (x, y).

        Devuelve la accion correspondiente o None si no se toco ningun boton
        habilitado.
        """
        for btn in self.buttons:
            bx, by, bw, bh = btn.rect
            if bx <= x <= bx + bw and by <= y <= by + bh:
                if btn.enabled:
                    return btn.action
                return None
        return None

    def handle_key(
        self, key: Union[int, str]
    ) -> Optional[Tuple[str, Optional[str]]]:
        """Procesa un atajo de teclado sin ambiguedad.

        - 1/2/3: IR a pose 0, 1, 2
        - !/@/# (Shift+1/2/3): ENSEÑAR pose 0, 1, 2
        - ENTER (13): guardar
        - ESC (27): cancelar
        - 'l'/'L': liberar brazo
        - 'k'/'K': bloquear brazo
        """
        if isinstance(key, int):
            if key in (13, 10):
                return ("save", None)
            if key == 27:
                return ("cancel", None)
            if 0 <= key <= 255:
                char = chr(key)
            else:
                return None
        elif isinstance(key, str):
            if key in ("\r", "\n"):
                return ("save", None)
            if key == "\x1b":
                return ("cancel", None)
            char = key
        else:
            return None

        # Atajos de movimiento IR
        if char == "1" and len(self.poses) > 0:
            return ("go", self.poses[0])
        if char == "2" and len(self.poses) > 1:
            return ("go", self.poses[1])
        if char == "3" and len(self.poses) > 2:
            return ("go", self.poses[2])

        # Atajos de ENSEÑAR (Shift + 1/2/3)
        if char == "!" and len(self.poses) > 0:
            return ("teach", self.poses[0])
        if char == "@" and len(self.poses) > 1:
            return ("teach", self.poses[1])
        if char == "#" and len(self.poses) > 2:
            return ("teach", self.poses[2])

        # Control de torque
        if char in ("l", "L"):
            return ("torque_off", None)
        if char in ("k", "K"):
            return ("torque_on", None)

        return None
