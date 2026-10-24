#!/usr/bin/env python3
# ==============================================================================
# Robot @Home - Cara Interactiva y Expresiva para Pantalla OLED / HDMI
# Diseno Vectorial Antialiased de Alta Calidad con Pygame y ROS 2 Jazzy
# Soporta Emociones: Neutral, Feliz, Escuchando, Pensando, Hablando y Parpadeo
# Sincronizado con Whisper ASR (/voice/status) y Piper TTS (/hri/say_something)
# ==============================================================================

import math
import os
import random
import sys
import threading
import time
from typing import Optional, Tuple

import pygame

# ROS 2 Imports
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


# Colores de la Paleta Cyberpunk / OLED Clean
COLOR_BG = (7, 11, 18)             # Negro OLED azulado profundo
COLOR_CYAN_GLOW = (0, 210, 255)     # Cyan neon brillante
COLOR_CYAN_CORE = (180, 245, 255)   # Blanco-cyan central
COLOR_HALO = (0, 100, 160)          # Halo exterior
COLOR_HAPPY = (0, 240, 180)         # Verde-cyan para felicidad
COLOR_THINKING = (160, 120, 255)    # Violeta neon para pensamiento
COLOR_HUD_TEXT = (140, 170, 200)    # Texto de estado
COLOR_WHITE = (245, 250, 255)


class RobotFaceApp(Node if ROS2_AVAILABLE else object):
    """Aplicacion de rostro robotico animado e interactivo con integracion ROS 2."""

    def __init__(self, width: int = 1280, height: int = 720, fullscreen: bool = False):
        if ROS2_AVAILABLE:
            super().__init__("robot_animated_face_node")
            self.sub_voice_status = self.create_subscription(
                String, "/voice/status", self.voice_status_callback, 10
            )
            self.sub_voice_text = self.create_subscription(
                String, "/voice/transcription", self.voice_text_callback, 10
            )
            self.sub_say = self.create_subscription(
                String, "/hri/say_something", self.say_callback, 10
            )
            self.sub_emotion = self.create_subscription(
                String, "/robot/emotion", self.emotion_callback, 10
            )
            self.get_logger().info("Nodo de Cara Animada ROS 2 inicializado.")

        self.width = width
        self.height = height
        self.fullscreen = fullscreen
        self.running = True

        # Estados de Emocion: "neutral", "happy", "listening", "thinking", "speaking"
        self.current_emotion = "neutral"
        self.target_emotion = "neutral"

        # Variables de Parpadeo y Mirada (Saccades)
        self.blink_progress = 0.0      # 0.0 = totalmente abierto, 1.0 = cerrado
        self.is_blinking = False
        self.next_blink_time = time.time() + random.uniform(3.0, 5.5)
        self.blink_duration = 0.18

        self.look_offset_x = 0.0
        self.look_offset_y = 0.0
        self.target_look_x = 0.0
        self.target_look_y = 0.0
        self.next_look_time = time.time() + 2.0

        # Animacion de Onda de Voz (Mouth Waveform)
        self.wave_phase = 0.0
        self.talking_until = 0.0
        self.subtitle_text = ""
        self.subtitle_until = 0.0
        self.voice_state = "IDLE"

        # Respiracion / Flotacion sutil
        self.breath_phase = 0.0

    # --- Callbacks ROS 2 ---
    def voice_status_callback(self, msg: String):
        status = msg.data.strip().upper()
        self.voice_state = status
        if status == "LISTENING":
            self.target_emotion = "listening"
        elif status == "PROCESSING":
            self.target_emotion = "thinking"
        elif status == "IDLE" and self.current_emotion not in ["speaking", "happy"]:
            self.target_emotion = "neutral"

    def voice_text_callback(self, msg: String):
        text = msg.data.strip()
        if text:
            self.subtitle_text = f'"{text}"'
            self.subtitle_until = time.time() + 4.5

    def say_callback(self, msg: String):
        text = msg.data.strip()
        if text:
            self.subtitle_text = f'Robot: "{text}"'
            # Calcular duracion estimada de locucion (~12 caracteres por segundo)
            dur = max(2.5, len(text) / 11.0)
            self.talking_until = time.time() + dur
            self.subtitle_until = time.time() + dur + 1.0
            self.target_emotion = "speaking"

    def emotion_callback(self, msg: String):
        emo = msg.data.strip().lower()
        if emo == "blink":
            self.trigger_blink()
        elif emo in ["neutral", "happy", "listening", "thinking", "speaking"]:
            self.target_emotion = emo

    def trigger_blink(self):
        if not self.is_blinking:
            self.is_blinking = True
            self.blink_progress = 0.0

    def update_logic(self, dt: float):
        now = time.time()
        self.breath_phase += dt * 1.8
        self.wave_phase += dt * 8.0

        # Actualizar estado de hablar
        if self.current_emotion == "speaking" and now > self.talking_until:
            self.target_emotion = "neutral"

        # Transicion de emocion
        self.current_emotion = self.target_emotion

        # Logica de Parpadeo
        if not self.is_blinking and now > self.next_blink_time:
            self.trigger_blink()
            self.next_blink_time = now + random.uniform(3.0, 6.0)

        if self.is_blinking:
            self.blink_progress += dt / (self.blink_duration / 2.0)
            if self.blink_progress >= 2.0:
                self.is_blinking = False
                self.blink_progress = 0.0

        # Logica de Mirada Natural (Micro-Saccades)
        if now > self.next_look_time:
            if random.random() < 0.45:
                # Ocasionalmente mirar hacia un lado sutilmente
                self.target_look_x = random.choice([-24.0, 24.0, -12.0, 12.0])
                self.target_look_y = random.choice([-10.0, 10.0, 0.0])
            else:
                self.target_look_x = 0.0
                self.target_look_y = 0.0
            self.next_look_time = now + random.uniform(1.8, 4.2)

        # Suavizado de la mirada (EMA)
        self.look_offset_x += (self.target_look_x - self.look_offset_x) * (dt * 8.0)
        self.look_offset_y += (self.target_look_y - self.look_offset_y) * (dt * 8.0)

    def draw_eye(
        self,
        surface: pygame.Surface,
        cx: int,
        cy: int,
        width: int,
        height: int,
        is_left: bool,
    ):
        """Dibuja un ojo estilizado con halos de brillo, parpadeo y expresion."""
        # Factor de parpadeo (1.0 = abierto, 0.0 = cerrado)
        if self.is_blinking:
            # Triangular curve 0 -> 1 -> 0
            open_factor = abs(1.0 - self.blink_progress)
        else:
            open_factor = 1.0

        # Ajuste de color segun emocion
        if self.current_emotion == "happy":
            glow_col = COLOR_HAPPY
            core_col = (200, 255, 230)
        elif self.current_emotion == "thinking":
            glow_col = COLOR_THINKING
            core_col = (220, 210, 255)
        else:
            glow_col = COLOR_CYAN_GLOW
            core_col = COLOR_CYAN_CORE

        cur_h = max(4, int(height * open_factor))

        if self.current_emotion == "happy" and open_factor > 0.4:
            # Ojos cerrados sonrientes (arcos ascendentes estilo anime / Pixar)
            arc_rect = pygame.Rect(cx - width // 2, cy - height // 2, width, height)
            pygame.draw.arc(surface, glow_col, arc_rect, 0.2, math.pi - 0.2, 14)
            pygame.draw.arc(surface, core_col, arc_rect, 0.2, math.pi - 0.2, 6)
            return

        # Capa exterior (Halo suave de neon)
        halo_rect = pygame.Rect(
            cx - width // 2 - 12,
            cy - cur_h // 2 - 8,
            width + 24,
            cur_h + 16,
        )
        halo_surf = pygame.Surface((halo_rect.width, halo_rect.height), pygame.SRCALPHA)
        pygame.draw.rect(
            halo_surf,
            (*glow_col, 45),
            (0, 0, halo_rect.width, halo_rect.height),
            border_radius=int(cur_h * 0.45),
        )
        surface.blit(halo_surf, (halo_rect.x, halo_rect.y))

        # Ojo Principal (Borde exterior brillante)
        eye_rect = pygame.Rect(
            cx - width // 2,
            cy - cur_h // 2,
            width,
            cur_h,
        )
        pygame.draw.rect(
            surface,
            glow_col,
            eye_rect,
            width=0,  # Relleno solido
            border_radius=int(cur_h * 0.38),
        )

        # Pupila interior / Brillo central
        if open_factor > 0.45:
            pupil_w = int(width * 0.42)
            pupil_h = int(cur_h * 0.45)
            # Desplazar pupila segun mirada
            px = cx + int(self.look_offset_x)
            py = cy + int(self.look_offset_y)

            if self.current_emotion == "thinking":
                # En pensando, mirar hacia arriba
                py -= 18

            pygame.draw.ellipse(
                surface,
                core_col,
                (px - pupil_w // 2, py - pupil_h // 2, pupil_w, pupil_h),
            )
            # Destello de luz especular (catchlight)
            pygame.draw.circle(
                surface,
                COLOR_WHITE,
                (px - pupil_w // 4, py - pupil_h // 4),
                max(3, int(pupil_w * 0.16)),
            )

        # Si esta pensando, dibujar anillo orbital giratorio sutil
        if self.current_emotion == "thinking" and is_left:
            orb_r = width // 2 + 18
            angle = self.wave_phase * 1.5
            ox = cx + int(math.cos(angle) * orb_r)
            oy = cy + int(math.sin(angle) * (orb_r * 0.35))
            pygame.draw.circle(surface, COLOR_THINKING, (ox, oy), 6)
            pygame.draw.circle(surface, COLOR_WHITE, (ox, oy), 3)

    def draw_eyebrows(self, surface: pygame.Surface, cx: int, cy: int, is_left: bool):
        """Dibuja cejas expresivas segun la emocion."""
        bw = 85
        by = cy - 110

        if self.current_emotion == "happy":
            tilt = 12 if is_left else -12
        elif self.current_emotion == "listening":
            tilt = -8 if is_left else 8
        elif self.current_emotion == "thinking":
            tilt = -14 if is_left else 6
        else:
            tilt = 0

        p1 = (cx - bw // 2, by + tilt)
        p2 = (cx + bw // 2, by - tilt)
        pygame.draw.line(surface, COLOR_CYAN_GLOW, p1, p2, 6)

    def draw_mouth(self, surface: pygame.Surface, cx: int, cy: int):
        """Dibuja la boca animada (onda de sonido para hablar/escuchar o sonrisa)."""
        now = time.time()

        if self.current_emotion == "speaking":
            # Onda de sonido parlante animada en tiempo real
            points = []
            num_pts = 28
            span = 240
            for i in range(num_pts):
                t = i / (num_pts - 1)
                x = cx - span // 2 + int(t * span)
                # Modulacion de amplitud
                envelope = math.sin(t * math.pi)
                amp = (
                    math.sin(self.wave_phase + t * 9.0) * 0.6
                    + math.sin(self.wave_phase * 1.8 + t * 14.0) * 0.4
                ) * 26.0 * envelope
                points.append((x, int(cy + amp)))

            pygame.draw.lines(surface, COLOR_CYAN_GLOW, False, points, 4)
            pygame.draw.lines(surface, COLOR_CYAN_CORE, False, points, 2)

        elif self.current_emotion == "listening":
            # Ecualizador de barras de frecuencia neon
            num_bars = 21
            span = 220
            bar_w = 6
            step = span // num_bars
            for i in range(num_bars):
                bx = cx - span // 2 + i * step
                freq = math.sin(now * 8.0 + i * 0.7) * 0.5 + 0.5
                bh = int(6 + freq * 28)
                pygame.draw.line(
                    surface,
                    COLOR_CYAN_GLOW,
                    (bx, cy - bh // 2),
                    (bx, cy + bh // 2),
                    bar_w,
                )

        elif self.current_emotion == "happy":
            # Sonrisa amplia y alegre
            rect = pygame.Rect(cx - 70, cy - 25, 140, 50)
            pygame.draw.arc(surface, COLOR_HAPPY, rect, math.pi + 0.3, -0.3, 6)

        else:
            # Neutral: linea sutilmente curvada
            p1 = (cx - 55, cy - 2)
            p_mid = (cx, cy + 4)
            p2 = (cx + 55, cy - 2)
            pygame.draw.lines(surface, COLOR_CYAN_GLOW, False, [p1, p_mid, p2], 4)

    def draw_hud(self, surface: pygame.Surface, font_title, font_sub):
        """Dibuja la barra de estado superior y subtitulos inferiores."""
        # Top-Left: Badge del Robot
        badge_rect = pygame.Rect(30, 25, 175, 42)
        pygame.draw.rect(surface, (18, 28, 45), badge_rect, border_radius=8)
        pygame.draw.rect(surface, COLOR_CYAN_GLOW, badge_rect, width=1, border_radius=8)
        txt_name = font_title.render("ROBOT @HOME", True, COLOR_WHITE)
        surface.blit(txt_name, (badge_rect.x + 14, badge_rect.y + 10))

        # Top-Center: Estado Activo
        state_txt = f"[{self.current_emotion.upper()}]"
        txt_state = font_sub.render(state_txt, True, COLOR_CYAN_GLOW)
        surface.blit(txt_state, (self.width // 2 - txt_state.get_width() // 2, 35))

        # Top-Right: Indicadores (Mic / Wi-Fi / Battery)
        mic_color = (0, 255, 120) if self.voice_state == "LISTENING" else (150, 160, 175)
        mic_txt = font_sub.render(f"Mic: {self.voice_state}", True, mic_color)
        surface.blit(mic_txt, (self.width - mic_txt.get_width() - 35, 35))

        # Bottom: Subtitulos dinamicos
        now = time.time()
        if self.subtitle_text and now < self.subtitle_until:
            sub_rendered = font_title.render(self.subtitle_text, True, COLOR_WHITE)
            sw = sub_rendered.get_width() + 40
            sh = 50
            sx = (self.width - sw) // 2
            sy = self.height - 85

            sub_bg = pygame.Surface((sw, sh), pygame.SRCALPHA)
            sub_bg.fill((12, 18, 28, 215))
            surface.blit(sub_bg, (sx, sy))
            pygame.draw.rect(surface, COLOR_CYAN_GLOW, (sx, sy, sw, sh), width=1, border_radius=8)
            surface.blit(sub_rendered, (sx + 20, sy + 13))

    def run(self):
        pygame.init()
        pygame.font.init()

        flags = pygame.FULLSCREEN if self.fullscreen else pygame.RESIZABLE
        screen = pygame.display.set_mode((self.width, self.height), flags)
        pygame.display.set_caption("Robot @Home - Cara Interactiva")
        clock = pygame.time.Clock()

        # Fuentes tipograficas
        font_title = pygame.font.SysModeFont(["Liberation Sans", "DejaVu Sans", "Arial"], 20, bold=True)
        font_sub = pygame.font.SysModeFont(["Liberation Sans", "DejaVu Sans", "Arial"], 16)

        eye_w, eye_h = 175, 135
        eye_spacing = 380

        print("==================================================")
        print("  ROBOT @HOME - CARA INTERACTIVA INICIADA         ")
        print("--------------------------------------------------")
        print("  [N] : Emocion Neutral                           ")
        print("  [H] : Emocion Feliz (Happy)                     ")
        print("  [L] : Emocion Escuchando (Listening)            ")
        print("  [T] : Emocion Pensando (Thinking)               ")
        print("  [S] : Emocion Hablando (Speaking Test)          ")
        print("  [B] : Forzar Parpadeo (Blink)                   ")
        print("  [F] : Alternar Pantalla Completa                ")
        print("  [Q] : Salir                                     ")
        print("==================================================")

        last_time = time.time()

        while self.running:
            # Procesar eventos ROS 2 si esta activo
            if ROS2_AVAILABLE and rclpy.ok():
                try:
                    rclpy.spin_once(self, timeout_sec=0.001)
                except Exception:
                    break

            now = time.time()
            dt = min(0.05, now - last_time)
            last_time = now

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.VIDEORESIZE and not self.fullscreen:
                    self.width, self.height = event.w, event.h
                    screen = pygame.display.set_mode((self.width, self.height), pygame.RESIZABLE)
                elif event.type == pygame.KEYDOWN:
                    if event.key in [pygame.K_ESCAPE, pygame.K_q]:
                        self.running = False
                    elif event.key == pygame.K_f:
                        self.fullscreen = not self.fullscreen
                        flags = pygame.FULLSCREEN if self.fullscreen else pygame.RESIZABLE
                        screen = pygame.display.set_mode((self.width, self.height), flags)
                    elif event.key == pygame.K_n:
                        self.target_emotion = "neutral"
                    elif event.key == pygame.K_h:
                        self.target_emotion = "happy"
                    elif event.key == pygame.K_l:
                        self.target_emotion = "listening"
                    elif event.key == pygame.K_t:
                        self.target_emotion = "thinking"
                    elif event.key == pygame.K_s:
                        self.say_callback(String(data="Hello! I am Robot. Nice to meet you."))
                    elif event.key == pygame.K_b:
                        self.trigger_blink()

            self.update_logic(dt)

            # Renderizado
            screen.fill(COLOR_BG)

            # Centro del rostro con leve flotacion de respiracion
            breath_offset = math.sin(self.breath_phase) * 4.0
            face_center_x = self.width // 2
            face_center_y = int(self.height // 2 + breath_offset) - 20

            left_eye_x = face_center_x - eye_spacing // 2
            right_eye_x = face_center_x + eye_spacing // 2
            eye_y = face_center_y - 25

            # Dibujar Ojos y Cejas
            self.draw_eyebrows(screen, left_eye_x, eye_y, is_left=True)
            self.draw_eyebrows(screen, right_eye_x, eye_y, is_left=False)

            self.draw_eye(screen, left_eye_x, eye_y, eye_w, eye_h, is_left=True)
            self.draw_eye(screen, right_eye_x, eye_y, eye_w, eye_h, is_left=False)

            # Dibujar Boca
            mouth_y = face_center_y + 130
            self.draw_mouth(screen, face_center_x, mouth_y)

            # Dibujar HUD
            self.draw_hud(screen, font_title, font_sub)

            pygame.display.flip()
            clock.tick(60)

        pygame.quit()


def SysModeFont(names, size, bold=False):
    for name in names:
        try:
            return pygame.font.SysFont(name, size, bold=bold)
        except Exception:
            pass
    return pygame.font.Font(None, size)

pygame.font.SysModeFont = SysModeFont


def main():
    if ROS2_AVAILABLE:
        rclpy.init()

    # Iniciar en 1280x720 para ventana o 1920x1080 para la pantalla del elevador
    app = RobotFaceApp(width=1280, height=720, fullscreen=False)
    try:
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        if ROS2_AVAILABLE and rclpy.ok():
            try:
                app.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
