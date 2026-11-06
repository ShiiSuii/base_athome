#!/usr/bin/env python3
"""Detector de timbre puro (sin dependencias de ROS) para RoboCup@Home.

Arquitectura y justificacion tecnica:
    Basado en el enfoque DSP de energia en banda y umbral adaptativo sobre el piso de
    ruido de fondo (documentado en docs/INVESTIGACION_HRI.txt y probado en la liga OPL
    por RoBorregos @Home).
    
    Un timbre tipico (chime tradicional o ding-dong electronico) se caracteriza por:
      1. Concentracion espectral en banda de interes (500 Hz a 4000 Hz) con alta tonalidad
         (crest factor espectral / relacion pico-a-promedio elevada).
      2. Energia en banda que supera el piso de ruido ambiente adaptativo por un margen
         definido (confirm_margin_db >= 12 dB).
      3. Ausencia de energia dominante en baja frecuencia (< 450 Hz), a diferencia de la voz
         humana que contiene la frecuencia fundamental (F0 en 80-300 Hz) y sus primeros armonicos.
      4. Duracion sostenida minima (min_duration_s >= 120 ms), lo que discrimina golpes
         breves (knocks/clicks de 20-40 ms) y picos impulsivos.
      5. Periodo refractario (refractory_s >= 2.5 s) que evita disparos redundantes o ecos
         secundarios tras la primera confirmacion valida.
      6. Auto-enrollment (huella espectral normalizada) que permite registrar la frecuencia
         del primer timbre detectado para aumentar la robustez de redeteccion.

Cumple estrictamente las reglas de AGENTS.md:
  - Modulo puro sin ROS (sin rclpy ni mensajes ROS).
  - Testeable con senales sinteticas sin necesidad de microfono fisico ni hardware encendido.
"""

from dataclasses import dataclass
from typing import Any, List, Optional, Union

import numpy as np


@dataclass
class DoorbellResult:
    """Resultado detallado de la evaluacion acustica de un bloque de audio."""

    detected: bool = False
    timestamp_s: float = 0.0
    frequency_hz: float = 0.0
    in_band_db: float = -100.0
    noise_floor_db: float = -100.0
    margin_db: float = 0.0
    tonality: float = 0.0
    duration_s: float = 0.0
    in_refractory: bool = False
    is_armed: bool = True
    confidence: float = 0.0
    fingerprint_similarity: float = 0.0

    def __bool__(self) -> bool:
        """Permite evaluar el resultado directamente como booleano: `if result: ...`"""
        return self.detected


class DoorbellDetector:
    """Detector DSP adaptativo de timbre acustico para recepcion en RoboCup@Home."""

    def __init__(
        self,
        sample_rate: int = 16000,
        freq_min: float = 500.0,
        freq_max: float = 4000.0,
        min_duration_s: float = 0.12,
        max_duration_s: float = 4.0,
        refractory_s: float = 2.5,
        confirm_margin_db: float = 12.0,
        min_tonality_crest: float = 16.0,
        min_in_band_ratio: float = 0.45,
        max_low_freq_ratio: float = 0.18,
        frame_size: int = 512,
        hop_size: int = 256,
        initial_floor_db: float = -60.0,
        alpha_floor: float = 0.05,
        auto_enroll: bool = True,
        armed: bool = True,
    ) -> None:
        """Inicializa el detector con parametros acusticos calibrados.

        Args:
            sample_rate: Frecuencia de muestreo PCM en Hz (predeterminado 16000 Hz).
            freq_min: Limite inferior de la banda util del timbre en Hz.
            freq_max: Limite superior de la banda util del timbre en Hz.
            min_duration_s: Duracion sostenida minima requerida (segundos).
            max_duration_s: Duracion maxima permitida para evitar tonos permanentes.
            refractory_s: Tiempo refractario de enfriamiento tras una deteccion (segundos).
            confirm_margin_db: Margen en dB sobre el piso de ruido para confirmar evento.
            min_tonality_crest: Factor de cresta minimo del espectro en banda para considerar tono.
            min_in_band_ratio: Fraccion minima de energia total contenida en la banda util.
            max_low_freq_ratio: Fraccion maxima permitida de energia en bajas frecuencias (< 450 Hz).
            frame_size: Tamano de ventana FFT en muestras (512 muestras = 32 ms a 16 kHz).
            hop_size: Salto entre cuadros en muestras (256 muestras = 16 ms a 16 kHz).
            initial_floor_db: Estimacion inicial del piso de ruido en dBFS.
            alpha_floor: Coeficiente EMA para adaptacion del piso de ruido ambiente.
            auto_enroll: Si es True, registra la huella espectral del primer timbre confirmado.
            armed: Estado inicial de armado (si esta desarmado, no genera eventos de deteccion).
        """
        self.sample_rate = sample_rate
        self.freq_min = freq_min
        self.freq_max = freq_max
        self.min_duration_s = min_duration_s
        self.max_duration_s = max_duration_s
        self.refractory_s = refractory_s
        self.confirm_margin_db = confirm_margin_db
        self.min_tonality_crest = min_tonality_crest
        self.min_in_band_ratio = min_in_band_ratio
        self.max_low_freq_ratio = max_low_freq_ratio
        self.frame_size = frame_size
        self.hop_size = hop_size
        self.hop_s = hop_size / float(sample_rate)
        self.initial_floor = 10.0 ** (initial_floor_db / 10.0)
        self.alpha_floor = alpha_floor
        self.auto_enroll = auto_enroll
        self.armed = armed

        # Ventana de Hann y factor de correccion coherente
        self.window = np.hanning(frame_size).astype(np.float32)
        win_sum = float(np.sum(self.window))
        self.scale = 2.0 / max(win_sum, 1e-6)

        # Mascaras de frecuencia
        freqs = np.fft.rfftfreq(frame_size, 1.0 / sample_rate)
        self.freqs = freqs
        self.in_band_mask = (freqs >= freq_min) & (freqs <= freq_max)
        self.low_freq_mask = freqs < max(freq_min - 50.0, 300.0)

        # Huella de auto-enrollment
        self.enrolled_fingerprint: Optional[np.ndarray] = None
        self.enrolled_freq: Optional[float] = None

        self.reset(reset_floor=True)

    def reset(self, reset_floor: bool = False) -> None:
        """Reinicia el estado temporal y buffers del detector.

        Args:
            reset_floor: Si es True, reinicia tambien el estimado del piso de ruido.
        """
        self._buffer = np.zeros(0, dtype=np.float32)
        self._active_duration_s = 0.0
        self._refractory_remaining_s = 0.0
        self._dropout_frames = 0
        self._elapsed_time_s = 0.0
        if reset_floor:
            self._noise_floor = float(self.initial_floor)

    def arm(self) -> None:
        """Arma el detector para permitir confirmaciones de timbre."""
        self.armed = True

    def disarm(self) -> None:
        """Desarma el detector (e.g. mientras el robot habla o se desplaza)."""
        self.armed = False
        self._active_duration_s = 0.0
        self._dropout_frames = 0

    @property
    def is_armed(self) -> bool:
        """Indica si el detector esta armado."""
        return self.armed

    @property
    def is_refractory(self) -> bool:
        """Indica si el detector se encuentra en periodo de enfriamiento refractario."""
        return self._refractory_remaining_s > 0.0

    @property
    def refractory_remaining_s(self) -> float:
        """Retorna los segundos restantes del periodo refractario."""
        return max(0.0, self._refractory_remaining_s)

    @property
    def noise_floor_db(self) -> float:
        """Retorna el nivel actual del piso de ruido adaptativo en dBFS."""
        return 10.0 * np.log10(max(self._noise_floor, 1e-12))

    def _normalize_input(self, chunk: Any) -> np.ndarray:
        """Convierte diferentes formatos de entrada PCM a un arreglo 1D float32 en [-1.0, 1.0]."""
        if isinstance(chunk, (bytes, bytearray)):
            if len(chunk) == 0:
                return np.zeros(0, dtype=np.float32)
            # Asume int16 Little Endian estandar
            return np.frombuffer(chunk, dtype=np.int16).astype(np.float32) / 32768.0

        samples = np.asarray(chunk, dtype=np.float32)
        if samples.size == 0:
            return np.zeros(0, dtype=np.float32)

        # Si es multicanal (e.g. estereo), promediar a mono
        if samples.ndim == 2:
            if samples.shape[1] == 1:
                samples = samples[:, 0]
            elif samples.shape[0] == 1:
                samples = samples[0, :]
            else:
                samples = np.mean(samples, axis=-1)
        elif samples.ndim > 2:
            samples = samples.flatten()

        # Si viene en escala int16 dentro de float (> 2.0 en valor absoluto)
        max_abs = float(np.max(np.abs(samples))) if samples.size > 0 else 0.0
        if max_abs > 2.0:
            samples = samples / 32768.0

        return samples

    def process(self, chunk: Union[np.ndarray, bytes, bytearray, List[float]]) -> DoorbellResult:
        """Procesa un bloque de audio PCM y retorna el diagnostico acustico y estado de deteccion.

        Args:
            chunk: Muestras de audio PCM en numpy array, bytes int16 o lista float.

        Returns:
            DoorbellResult con el veredicto de deteccion y metricas espectrales.
        """
        samples = self._normalize_input(chunk)
        if samples.size > 0:
            self._buffer = np.concatenate([self._buffer, samples])

        last_result = DoorbellResult(
            detected=False,
            timestamp_s=self._elapsed_time_s,
            noise_floor_db=self.noise_floor_db,
            in_refractory=self.is_refractory,
            is_armed=self.armed,
        )

        event_occurred = False

        while len(self._buffer) >= self.frame_size:
            frame = self._buffer[: self.frame_size]
            self._buffer = self._buffer[self.hop_size :]
            self._elapsed_time_s += self.hop_s

            # Decrementar temporizador refractario
            if self._refractory_remaining_s > 0.0:
                self._refractory_remaining_s = max(0.0, self._refractory_remaining_s - self.hop_s)

            # Analisis espectral FFT
            X = np.fft.rfft(frame * self.window)
            amps = np.abs(X) * self.scale
            P = amps**2

            E_total = float(np.sum(P)) + 1e-12
            P_band = P[self.in_band_mask]
            E_band = float(np.sum(P_band))
            E_low = float(np.sum(P[self.low_freq_mask]))

            in_band_ratio = E_band / E_total
            low_freq_ratio = E_low / E_total

            in_band_db = 10.0 * np.log10(max(E_band, 1e-12))
            margin_db = 10.0 * np.log10(max(E_band, 1e-12) / max(self._noise_floor, 1e-12))

            # Factor de cresta espectral (tonalidad)
            mean_band_power = float(np.mean(P_band)) if len(P_band) > 0 else 1e-12
            max_band_power = float(np.max(P_band)) if len(P_band) > 0 else 0.0
            crest = max_band_power / max(mean_band_power, 1e-12)

            # Frecuencia pico en la banda util
            if len(P_band) > 0:
                peak_idx = int(np.argmax(P_band))
                peak_freq = float(self.freqs[self.in_band_mask][peak_idx])
            else:
                peak_freq = 0.0

            # Evaluacion de similitud con huella enrolada
            fp_similarity = 0.0
            if self.enrolled_fingerprint is not None and len(P_band) > 0:
                v = P_band / (float(np.linalg.norm(P_band)) + 1e-12)
                fp_similarity = float(np.dot(self.enrolled_fingerprint, v))

            # Umbral de confirmacion adaptado si coincide con huella enrolada
            required_margin = self.confirm_margin_db
            if fp_similarity >= 0.85:
                # Si la firma espectral es identica, tolera 3 dB menos de margen
                required_margin = max(6.0, self.confirm_margin_db - 3.0)

            # Criterios discriminativos de timbre:
            # 1) Tonalidad elevada (energia concentrada en tono o dos tonos)
            is_tonal = crest >= self.min_tonality_crest
            # 2) Mayoria de la energia en la banda del timbre
            is_in_band = in_band_ratio >= self.min_in_band_ratio
            # 3) Baja presencia de contenido en baja frecuencia (rechaza voz humana F0)
            not_voice_low = low_freq_ratio <= self.max_low_freq_ratio
            # 4) Margen suficiente sobre el ruido de fondo adaptativo
            has_margin = margin_db >= required_margin

            is_candidate = is_tonal and is_in_band and not_voice_low and has_margin

            if is_candidate:
                self._dropout_frames = 0
                self._active_duration_s += self.hop_s

                # Confianza basada en cresta, margen y relacion en banda
                confidence = float(
                    np.clip(
                        0.4 * (crest / 40.0)
                        + 0.4 * (margin_db / 24.0)
                        + 0.2 * in_band_ratio,
                        0.0,
                        1.0,
                    )
                )

                if (
                    self.min_duration_s
                    <= self._active_duration_s
                    <= self.max_duration_s
                ):
                    if not self.is_refractory and self.armed:
                        event_occurred = True
                        self._refractory_remaining_s = self.refractory_s

                        # Auto-enrollment en la primera deteccion
                        if self.auto_enroll and self.enrolled_fingerprint is None and len(P_band) > 0:
                            v = P_band / (float(np.linalg.norm(P_band)) + 1e-12)
                            self.enrolled_fingerprint = v
                            self.enrolled_freq = peak_freq

                        last_result = DoorbellResult(
                            detected=True,
                            timestamp_s=self._elapsed_time_s,
                            frequency_hz=peak_freq,
                            in_band_db=in_band_db,
                            noise_floor_db=self.noise_floor_db,
                            margin_db=margin_db,
                            tonality=crest,
                            duration_s=self._active_duration_s,
                            in_refractory=True,
                            is_armed=self.armed,
                            confidence=confidence,
                            fingerprint_similarity=fp_similarity,
                        )
                        self._active_duration_s = 0.0
            else:
                # Tolerancia a 1 cuadro de transicion/parpadeo (e.g. entre ding y dong)
                if self._active_duration_s > 0.0 and self._dropout_frames < 1:
                    self._dropout_frames += 1
                else:
                    self._active_duration_s = 0.0
                    self._dropout_frames = 0

                # Actualizacion del piso de ruido adaptativo:
                # Solo se adapta cuando NO es un tono candidato (evita que el timbre contamine el piso)
                if not is_tonal and not is_candidate:
                    if E_band < self._noise_floor:
                        # Si bajo el ruido ambiente, baja con mayor velocidad (alpha=0.10)
                        self._noise_floor = 0.90 * self._noise_floor + 0.10 * E_band
                    else:
                        # Si subio el ruido ambiente, sube suavemente (alpha predeterminado=0.05)
                        self._noise_floor = (1.0 - self.alpha_floor) * self._noise_floor + self.alpha_floor * E_band

            if not event_occurred:
                last_result = DoorbellResult(
                    detected=False,
                    timestamp_s=self._elapsed_time_s,
                    frequency_hz=peak_freq,
                    in_band_db=in_band_db,
                    noise_floor_db=self.noise_floor_db,
                    margin_db=margin_db,
                    tonality=crest,
                    duration_s=self._active_duration_s,
                    in_refractory=self.is_refractory,
                    is_armed=self.armed,
                    fingerprint_similarity=fp_similarity,
                )

        if event_occurred:
            last_result.detected = True

        return last_result

    def process_chunk(self, chunk: Union[np.ndarray, bytes, bytearray, List[float]]) -> bool:
        """Metodo de conveniencia que retorna directamente bool si se detecto un timbre."""
        return bool(self.process(chunk))
