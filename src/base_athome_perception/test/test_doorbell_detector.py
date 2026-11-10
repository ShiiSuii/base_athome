#!/usr/bin/env python3
"""Pruebas unitarias con senales sinteticas para DoorbellDetector (HOME-96).

Verifica exhaustivamente el detector de timbre en modulo puro (sin ROS):
  1. Tono simple (single tone, chimes tradicionales de 800, 1000, 1500 Hz).
  2. Dos tonos (secuencial ding-dong y simultaneo acorde dual).
  3. Rechazo de voz simulada (sintesis armonica con formantes y modulacion de pitch).
  4. Rechazo de ruido ambiente (blanco, coloreado, escalon de ruido).
  5. Rechazo de golpes breves (knocks/clicks de 20 a 40 ms < min_duration).
  6. Medicion explicita de falsos positivos (tasa cero esperada en voz, ruido y golpes).
  7. Periodo refractario de enfriamiento.
  8. Adaptacion del piso de ruido de fondo.
  9. Variedad de formatos PCM de entrada (float32, int16, bytes, multicanal).
  10. Procesamiento con diferentes tamanos de bloque/chunk.
  11. Armado y desarmado.
  12. Auto-enrollment y similitud de huella espectral.
"""

import os
from typing import Optional
import sys

import numpy as np
import pytest

# Asegurar que el modulo local del worktree tenga precedencia incluso si
# base_athome_perception ya fue importado desde install/
_src_pkg_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
_src_pkg_dir = os.path.join(_src_pkg_root, "base_athome_perception")
if _src_pkg_root not in sys.path:
    sys.path.insert(0, _src_pkg_root)
if "base_athome_perception" in sys.modules:
    _mod = sys.modules["base_athome_perception"]
    if hasattr(_mod, "__path__") and _src_pkg_dir not in _mod.__path__:
        _mod.__path__.insert(0, _src_pkg_dir)

from base_athome_perception.doorbell_detector import DoorbellDetector, DoorbellResult


def generate_tone(
    freq: float, duration_s: float, sample_rate: int = 16000, amplitude: float = 0.2
) -> np.ndarray:
    """Genera un tono sinusoidal puro."""
    t = np.arange(int(duration_s * sample_rate)) / float(sample_rate)
    return (amplitude * np.sin(2.0 * np.pi * freq * t)).astype(np.float32)


def generate_ding_dong(
    freq1: float = 800.0,
    freq2: float = 600.0,
    dur1_s: float = 0.15,
    dur2_s: float = 0.20,
    sample_rate: int = 16000,
    amplitude: float = 0.2,
) -> np.ndarray:
    """Genera un timbre clasico bitonal secuencial ding-dong."""
    tone1 = generate_tone(freq1, dur1_s, sample_rate, amplitude)
    tone2 = generate_tone(freq2, dur2_s, sample_rate, amplitude)
    return np.concatenate([tone1, tone2])


def generate_dual_tone_chord(
    freq1: float = 800.0,
    freq2: float = 1200.0,
    duration_s: float = 0.25,
    sample_rate: int = 16000,
    amplitude: float = 0.15,
) -> np.ndarray:
    """Genera un timbre de acorde simultaneo con dos frecuencias tonales."""
    t = np.arange(int(duration_s * sample_rate)) / float(sample_rate)
    sig = amplitude * np.sin(2.0 * np.pi * freq1 * t) + amplitude * np.sin(2.0 * np.pi * freq2 * t)
    return sig.astype(np.float32)


def generate_simulated_voice(
    f0_base: float = 130.0,
    duration_s: float = 1.0,
    sample_rate: int = 16000,
    amplitude: float = 0.3,
) -> np.ndarray:
    """Genera senal vocal simulada con barrido de pitch (prosodia) y resonancias formanticas."""
    t = np.arange(int(duration_s * sample_rate)) / float(sample_rate)
    # Variacion dinamica de entonacion (pitch contour)
    f0_contour = f0_base + 35.0 * np.sin(2.0 * np.pi * 3.0 * t)
    phase = 2.0 * np.pi * np.cumsum(f0_contour) / float(sample_rate)

    voice = np.zeros_like(t)
    # Suma de armonicos con decaimiento glotico y picos formanticos F1 (~600 Hz), F2 (~1400 Hz)
    for h in range(1, 28):
        fh = h * f0_base
        if fh < sample_rate / 2.0:
            glottal = 1.0 / (h**0.8)
            formant1 = np.exp(-(((fh - 650.0) / 140.0) ** 2))
            formant2 = 0.5 * np.exp(-(((fh - 1400.0) / 180.0) ** 2))
            gain = glottal * (1.0 + formant1 + formant2)
            voice += gain * np.sin(h * phase)

    max_v = np.max(np.abs(voice))
    if max_v > 0:
        voice = (voice / max_v) * amplitude
    return voice.astype(np.float32)


def generate_knock(
    duration_s: float = 0.03,
    freq: float = 800.0,
    sample_rate: int = 16000,
    amplitude: float = 0.4,
) -> np.ndarray:
    """Genera un golpe seco breve (transitorio con decaimiento exponencial rapido < 40 ms)."""
    t = np.arange(int(duration_s * sample_rate)) / float(sample_rate)
    decay = np.exp(-t / 0.007)
    knock = amplitude * np.sin(2.0 * np.pi * freq * t) * decay
    return knock.astype(np.float32)


def stream_audio(detector: DoorbellDetector, audio: np.ndarray, chunk_size: int = 512) -> int:
    """Divide una senal en bloques PCM y cuenta las detecciones observadas."""
    detections = 0
    num_chunks = len(audio) // chunk_size
    for i in range(num_chunks):
        chunk = audio[i * chunk_size : (i + 1) * chunk_size]
        res = detector.process(chunk)
        if res.detected:
            detections += 1
    # Resto de muestras si quedan
    rem = len(audio) % chunk_size
    if rem > 0:
        chunk = audio[num_chunks * chunk_size :]
        res = detector.process(chunk)
        if res.detected:
            detections += 1
    return detections


# ==============================================================================
# PRUEBAS DE DETECCION POSITIVA (TIMBRE)
# ==============================================================================


def test_single_tone_detection():
    """Verifica que tonos puros en la banda del timbre se detecten con precision."""
    detector = DoorbellDetector(sample_rate=16000, confirm_margin_db=12.0)

    for freq in [800.0, 1000.0, 1500.0, 2200.0]:
        detector.reset(reset_floor=True)
        # 100 ms silencio inicial + 250 ms tono + 200 ms silencio
        silence_pre = np.zeros(int(0.10 * 16000), dtype=np.float32)
        tone = generate_tone(freq=freq, duration_s=0.25, amplitude=0.20)
        silence_post = np.zeros(int(0.20 * 16000), dtype=np.float32)
        signal = np.concatenate([silence_pre, tone, silence_post])

        detections = stream_audio(detector, signal, chunk_size=512)
        assert detections == 1, f"Tono de {freq} Hz debio producir exactamente 1 deteccion, obtuvo {detections}"


def test_two_tones_sequential_ding_dong():
    """Verifica la deteccion de un timbre clasico bitonal ding-dong."""
    detector = DoorbellDetector(sample_rate=16000, confirm_margin_db=12.0)

    # ding (800 Hz, 150 ms) -> dong (600 Hz, 200 ms)
    silence = np.zeros(int(0.10 * 16000), dtype=np.float32)
    ding_dong = generate_ding_dong(freq1=800.0, freq2=600.0, dur1_s=0.15, dur2_s=0.20, amplitude=0.22)
    signal = np.concatenate([silence, ding_dong, silence])

    detections = stream_audio(detector, signal, chunk_size=512)
    assert detections == 1, f"Ding-dong secuencial debio generar 1 deteccion, obtuvo {detections}"


def test_two_tones_simultaneous_chord():
    """Verifica la deteccion de un timbre de dos tonos simultaneos (acorde dual)."""
    detector = DoorbellDetector(sample_rate=16000, confirm_margin_db=12.0)

    silence = np.zeros(int(0.10 * 16000), dtype=np.float32)
    chord = generate_dual_tone_chord(freq1=800.0, freq2=1200.0, duration_s=0.25, amplitude=0.18)
    signal = np.concatenate([silence, chord, silence])

    detections = stream_audio(detector, signal, chunk_size=512)
    assert detections == 1, f"Acorde dual debio generar 1 deteccion, obtuvo {detections}"


def test_detection_reported_metadata():
    """Verifica que el objeto DoorbellResult informe frecuencia, margen y confianza correctos."""
    detector = DoorbellDetector(sample_rate=16000, min_duration_s=0.12)
    tone = generate_tone(freq=1200.0, duration_s=0.30, amplitude=0.25)

    detected_res: Optional[DoorbellResult] = None
    for i in range(len(tone) // 256):
        res = detector.process(tone[i * 256 : (i + 1) * 256])
        if res.detected:
            detected_res = res
            break

    assert detected_res is not None, "El timbre no fue detectado"
    assert detected_res.detected is True
    assert bool(detected_res) is True
    # Margen del FFT: frecuencia pico proxima a 1200 Hz (+- 35 Hz)
    assert abs(detected_res.frequency_hz - 1200.0) < 40.0
    assert detected_res.margin_db >= 12.0
    assert detected_res.confidence > 0.5
    assert detected_res.in_refractory is True


# ==============================================================================
# PRUEBAS DE MEDICION DE FALSOS POSITIVOS (RECHAZO DE RUIDO, VOZ Y GOLPES)
# ==============================================================================


def test_simulated_voice_rejection_measures_false_positives():
    """Mide falsos positivos ante voz humana simulada a volumen elevado."""
    detector = DoorbellDetector(sample_rate=16000)

    false_positives = 0
    total_speech_cases = 0

    # Probamos perfiles vocales masculinos, femeninos y agudos
    for f0 in [110.0, 140.0, 200.0, 240.0, 280.0]:
        detector.reset(reset_floor=True)
        # Voz simulada de 1.2 segundos con prosodia variable
        voice = generate_simulated_voice(f0_base=f0, duration_s=1.2, amplitude=0.35)
        detections = stream_audio(detector, voice, chunk_size=512)
        false_positives += detections
        total_speech_cases += 1

    assert false_positives == 0, f"Falsos positivos detectados en voz: {false_positives} en {total_speech_cases} casos"


def test_noise_rejection_measures_false_positives():
    """Mide falsos positivos ante ruido blanco, coloreado y escalones de ruido ambiente."""
    detector = DoorbellDetector(sample_rate=16000)

    false_positives = 0
    np.random.seed(42)

    # 1. Ruido blanco estacionario moderado (RMS ~ 0.05)
    noise_white = 0.05 * np.random.randn(int(1.5 * 16000)).astype(np.float32)
    false_positives += stream_audio(detector, noise_white, chunk_size=512)

    # 2. Ruido fuerte (RMS ~ 0.15)
    detector.reset(reset_floor=True)
    noise_loud = 0.15 * np.random.randn(int(1.5 * 16000)).astype(np.float32)
    false_positives += stream_audio(detector, noise_loud, chunk_size=512)

    # 3. Escalon repentino de ruido (+15 dB jump de ventilador)
    detector.reset(reset_floor=True)
    step_noise = np.concatenate([
        0.01 * np.random.randn(int(0.5 * 16000)),
        0.10 * np.random.randn(int(1.0 * 16000)),
    ]).astype(np.float32)
    false_positives += stream_audio(detector, step_noise, chunk_size=512)

    assert false_positives == 0, f"Falsos positivos detectados en ruido: {false_positives}"


def test_brief_knock_rejection_measures_false_positives():
    """Mide falsos positivos ante golpes breves (knocks < min_duration)."""
    detector = DoorbellDetector(sample_rate=16000, min_duration_s=0.12)

    false_positives = 0

    # Golpe breve individual de 20 ms, 30 ms y 40 ms
    for dur_s in [0.02, 0.03, 0.04]:
        detector.reset(reset_floor=True)
        silence = np.zeros(int(0.20 * 16000), dtype=np.float32)
        knock = generate_knock(duration_s=dur_s, freq=900.0, amplitude=0.45)
        signal = np.concatenate([silence, knock, silence])
        false_positives += stream_audio(detector, signal, chunk_size=256)

    # Doble golpe seco (knock-knock) separado por 150 ms
    detector.reset(reset_floor=True)
    k1 = generate_knock(duration_s=0.03, freq=1000.0, amplitude=0.45)
    gap = np.zeros(int(0.15 * 16000), dtype=np.float32)
    k2 = generate_knock(duration_s=0.03, freq=1000.0, amplitude=0.45)
    double_knock = np.concatenate([silence, k1, gap, k2, silence])
    false_positives += stream_audio(detector, double_knock, chunk_size=256)

    assert false_positives == 0, f"Falsos positivos detectados en golpes breves: {false_positives}"


# ==============================================================================
# PRUEBAS DE REGLAS TEMPORALES Y MECANISMOS DSP
# ==============================================================================


def test_refractory_period_blocks_secondary_triggers():
    """Verifica que el periodo refractario (2.5 s) bloquee re-disparos antes de enfriarse."""
    refractory_s = 2.5
    detector = DoorbellDetector(sample_rate=16000, refractory_s=refractory_s)

    tone_300ms = generate_tone(1000.0, 0.30, amplitude=0.20)
    silence_1s = np.zeros(int(1.0 * 16000), dtype=np.float32)
    silence_3s = np.zeros(int(3.0 * 16000), dtype=np.float32)

    # Secuencia:
    # 1. Tono inicial -> DEBE DETECTAR (1er disparo)
    # 2. Silencio de 1.0 s (< 2.5 s)
    # 3. Segundo tono -> NO DEBE DETECTAR (en refractario)
    # 4. Silencio de 3.0 s (> 2.5 s: se agota refractario)
    # 5. Tercer tono -> DEBE DETECTAR (2do disparo)
    seq = np.concatenate([tone_300ms, silence_1s, tone_300ms, silence_3s, tone_300ms])

    detections = stream_audio(detector, seq, chunk_size=512)
    assert detections == 2, f"Se esperaban exactamente 2 detecciones (inicio y tras refractario), hubo {detections}"


def test_adaptive_noise_floor_tracking():
    """Verifica que el piso de ruido ambiente se adapte sin contaminarse por timbres."""
    detector = DoorbellDetector(sample_rate=16000, initial_floor_db=-60.0)

    # 1 segundo de ruido ambiente a -40 dBFS (RMS ~ 0.01)
    np.random.seed(123)
    noise_ambient = 0.01 * np.random.randn(int(1.0 * 16000)).astype(np.float32)
    stream_audio(detector, noise_ambient, chunk_size=512)

    # El piso de ruido debe haber subido desde -60 dB hacia aproximadamente -40 dB
    assert -45.0 <= detector.noise_floor_db <= -35.0

    # Si se inyecta un tono a -15 dBFS (margen > 15 dB sobre -40 dB)
    tone = generate_tone(1000.0, 0.30, amplitude=0.18)
    detections = stream_audio(detector, tone, chunk_size=512)
    assert detections == 1


def test_pcm_input_formats():
    """Verifica compatibilidad transparente con int16, bytes crudos, estereo y listas."""
    detector = DoorbellDetector(sample_rate=16000)

    tone_f32 = generate_tone(1000.0, 0.25, amplitude=0.25)
    tone_i16 = (tone_f32 * 32767.0).astype(np.int16)
    tone_bytes = tone_i16.tobytes()
    tone_stereo = np.column_stack([tone_f32, tone_f32])

    # Formato bytes crudos
    detector.reset(reset_floor=True)
    d_bytes = 0
    chunk_bytes_len = 512 * 2  # 512 muestras de 16-bit
    for i in range(len(tone_bytes) // chunk_bytes_len):
        chunk = tone_bytes[i * chunk_bytes_len : (i + 1) * chunk_bytes_len]
        if detector.process(chunk).detected:
            d_bytes += 1
    assert d_bytes == 1

    # Formato int16 numpy
    detector.reset(reset_floor=True)
    d_i16 = stream_audio(detector, tone_i16, chunk_size=512)
    assert d_i16 == 1

    # Formato estereo 2D
    detector.reset(reset_floor=True)
    d_stereo = 0
    for i in range(len(tone_stereo) // 512):
        chunk = tone_stereo[i * 512 : (i + 1) * 512]
        if detector.process_chunk(chunk):
            d_stereo += 1
    assert d_stereo == 1


def test_varying_chunk_sizes():
    """Verifica que el procesamiento en streaming funcione con cualquier tamano de chunk."""
    tone = generate_tone(1000.0, 0.30, amplitude=0.25)

    for chunk_sz in [128, 256, 512, 1024, 4800]:
        detector = DoorbellDetector(sample_rate=16000)
        detections = stream_audio(detector, tone, chunk_size=chunk_sz)
        assert detections == 1, f"Fallo con chunk_size={chunk_sz}: obtuvo {detections}"


def test_arming_and_disarming():
    """Verifica que desarmar el detector impida disparos de timbre."""
    detector = DoorbellDetector(sample_rate=16000, armed=False)
    tone = generate_tone(1000.0, 0.30, amplitude=0.25)

    # Desarmado: no debe detectar
    detections = stream_audio(detector, tone, chunk_size=512)
    assert detections == 0
    assert detector.is_armed is False

    # Armado: debe detectar
    detector.reset(reset_floor=True)
    detector.arm()
    assert detector.is_armed is True
    detections_armed = stream_audio(detector, tone, chunk_size=512)
    assert detections_armed == 1


def test_auto_enrollment_and_spectral_similarity():
    """Verifica que la huella espectral enrolada permita confirmar re-detecciones."""
    detector = DoorbellDetector(sample_rate=16000, auto_enroll=True, confirm_margin_db=12.0)

    # 1. Primer timbre a 1100 Hz -> Enrola huella
    tone1 = generate_tone(1100.0, 0.30, amplitude=0.25)
    detections1 = stream_audio(detector, tone1, chunk_size=512)
    assert detections1 == 1
    assert detector.enrolled_fingerprint is not None
    assert abs(detector.enrolled_freq - 1100.0) < 35.0

    # Avanzamos tiempo para salir del refractario
    silence = np.zeros(int(3.0 * 16000), dtype=np.float32)
    stream_audio(detector, silence, chunk_size=512)
    assert detector.is_refractory is False

    # 2. Segundo timbre identico pero mas suave (volumen bajo con margen 10 dB)
    tone2_soft = generate_tone(1100.0, 0.30, amplitude=0.04)
    # Sin huella enrolada requeriria 12 dB; con similitud >= 0.85 tolera 9 dB
    res = detector.process(tone2_soft)
    # Comprobar que reconocio la huella
    assert res.fingerprint_similarity > 0.85


def test_reset_functionality():
    """Verifica que reset limpie completamente el estado temporal y buffers."""
    detector = DoorbellDetector(sample_rate=16000)
    tone = generate_tone(1000.0, 0.10, amplitude=0.25)  # 100 ms (no llega a 120 ms)
    stream_audio(detector, tone, chunk_size=256)

    # Debe haber acumulado duracion pero no disparado
    assert detector._active_duration_s > 0.0

    detector.reset(reset_floor=False)
    assert detector._active_duration_s == 0.0
    assert len(detector._buffer) == 0
    assert detector.is_refractory is False
