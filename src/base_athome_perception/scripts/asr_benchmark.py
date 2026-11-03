#!/usr/bin/env python3
"""Banco de prueba del microfono REAL (camino acustico completo).

Reproduce frases por el parlante del robot (paplay en la NUC), el RØDE de la Jetson las escucha con
el ASR en produccion y se mide: texto, nombre/bebida/si-no entendidos y demora desde que termina la
frase hasta que llega el resultado. Sirve para comparar cambios con datos, no con impresiones.

  ros2 run base_athome_perception asr_benchmark                 # frases sinteticas (Piper)
  ros2 run base_athome_perception asr_benchmark --wav-dir DIR   # grabaciones propias: DIR/index.txt
                                                                 # con lineas "archivo.wav|texto esperado"
  ros2 run base_athome_perception asr_benchmark --inject --gain 0.3
      # sin parlante: la Jetson mezcla cada WAV con el microfono en vivo (/voice/debug_inject).
      # --gain baja el volumen de la voz (0.3 ~ persona a 2-3 m). Reproducible y sin depender del parlante.
  ros2 run base_athome_perception asr_benchmark --once [--n 5]
      # voz humana real: el robot hace beep, escucha y muestra lo entendido (nombre/bebida/si-no) y la demora.
Opciones: --delay S (silencio antes de hablar), --repeat N, --late (habla a los 5 s), --csv salida.csv
Usar ROS_DOMAIN_ID del robot (10). No mueve actuadores.
"""
import argparse
import csv
import os
import subprocess
import sys
import tempfile
import threading
import json
import time
import wave

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from base_athome_perception.guest_dialog import extract_name_and_drink, parse_yes_no
from base_athome_perception.guest_vocabulary import GuestVocabulary
from base_athome_perception.voice_client import VoiceClient

PIPER = "/home/athome/piper_bin/piper/piper"
VOICE = "/home/athome/piper_bin/voices/en/en_US-lessac-medium.onnx"
PHRASES = [
    "My name is Adel and my favorite drink is orange juice.",
    "My name is Angel and my favorite drink is cola.",
    "My name is Axel and my favorite drink is milk.",
    "My name is Charlie and my favorite drink is tropical juice.",
    "My name is Jane and my favorite drink is red wine.",
    "My name is Jules and my favorite drink is iced tea.",
    "My name is Morgan and my favorite drink is juice pack.",
    "My name is Paris and my favorite drink is milk.",
    "My name is Robin and my favorite drink is iced tea.",
    "My name is Simone and my favorite drink is cola.",
    "Hi, I am Jules. I would like a cola, please.",
    "My name is Robin, and, um, my favorite drink is... orange juice.",
    "Yes.", "No.", "Yes, that is correct.", "No, that is wrong.",
]


def wav_seconds(path):
    with wave.open(path) as w:
        return w.getnframes() / float(w.getframerate())


def synth(text, path, length_scale="1.0"):
    p = subprocess.run([PIPER, "--model", VOICE, "--length_scale", length_scale, "--output_file", path],
                       input=text.encode(), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return p.returncode == 0 and os.path.exists(path)


def check(expected, heard, vocab):
    yn = parse_yes_no(expected) if len(expected.split()) <= 5 else None
    if yn is not None:
        return parse_yes_no(heard) == yn, f"si/no={parse_yes_no(heard)}"
    en, ed = extract_name_and_drink(expected, vocab)
    hn, hd = extract_name_and_drink(heard or "", vocab)
    return (en, ed) == (hn, hd), f"{hn}/{hd}"


def listen_live(args):
    """Prueba con una persona: beep, escucha, y muestra que entendio el robot."""
    rclpy.init()
    node = Node("asr_escuchar")
    voice = VoiceClient(node)
    vocab = GuestVocabulary()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    t = time.time()
    while not voice.online() and time.time() - t < 5.0:
        time.sleep(0.1)
    if not voice.online():
        sys.exit("El ASR no publica /voice/status (Jetson/whisper_node apagados o ROS_DOMAIN_ID distinto)")
    beep = os.path.expanduser("~/base_athome_ws/src/base_athome_perception/resources/beep.wav")
    for i in range(args.n):
        print(f"\n[{i + 1}/{args.n}] Hable despues del beep (ej: 'My name is Robin and my favorite drink is iced tea')")
        if os.path.exists(beep):
            subprocess.run(["paplay", beep])
        t0 = time.time()
        rid = voice.request(prompt=vocab.whisper_prompt())
        res = None
        while res is None:
            res = voice.poll(rid)
            time.sleep(0.02)
        n, d = extract_name_and_drink(res.text, vocab) if res.text else (None, None)
        print(f"  estado: {res.status} {res.reason}\n  texto : {res.text!r}\n  nombre/bebida: {n}/{d}   si/no: "
              f"{parse_yes_no(res.text)}\n  tiempos: total {time.time() - t0:.1f} s, {res.info}")
    node.destroy_node()
    rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wav-dir")
    ap.add_argument("--delay", type=float, default=0.6)
    ap.add_argument("--late", action="store_true", help="hablar 5 s despues del pedido")
    ap.add_argument("--repeat", type=int, default=1)
    ap.add_argument("--csv")
    ap.add_argument("--sink", default="", help="sink de pulseaudio (por defecto el actual)")
    ap.add_argument("--inject", action="store_true", help="mezclar los WAV en la Jetson en vez de usar el parlante")
    ap.add_argument("--gain", type=float, default=1.0, help="volumen de la voz inyectada")
    ap.add_argument("--jetson", default="ubuntu@10.42.0.12")
    ap.add_argument("--once", action="store_true", help="escuchar a una persona real (beep + escucha)")
    ap.add_argument("--n", type=int, default=1, help="con --once: cuantas veces")
    args = ap.parse_args()

    if args.once:
        return listen_live(args)
    items = []
    tmp = tempfile.mkdtemp(prefix="asr_bench_")
    if args.wav_dir:
        for line in open(os.path.join(args.wav_dir, "index.txt"), encoding="utf-8"):
            if "|" in line:
                f, text = line.strip().split("|", 1)
                items.append((os.path.join(args.wav_dir, f if f.endswith(".wav") else f + ".wav"), text))
    else:
        for i, text in enumerate(PHRASES):
            path = os.path.join(tmp, f"p{i}.wav")
            if not synth(text, path):
                sys.exit(f"No pude sintetizar con Piper ({PIPER})")
            items.append((path, text))

    if args.inject:
        remote = "/tmp/asr_bench"
        subprocess.run(["ssh", args.jetson, f"mkdir -p {remote}"], check=True)
        subprocess.run(["scp", "-q"] + [p for p, _ in items] + [f"{args.jetson}:{remote}/"], check=True)
        items = [(p, t, f"{remote}/{os.path.basename(p)}") for p, t in items]
    else:
        items = [(p, t, None) for p, t in items]
        # el sink HDMI se suspende y al despertar se come el comienzo de la frase: mantenerlo activo
        keepalive = subprocess.Popen(["pacat", "--raw", "--format", "s16le", "--rate", "22050", "--channels", "1",
                                      "/dev/zero"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    rclpy.init()
    node = Node("asr_benchmark")
    pub_inject = node.create_publisher(String, "/voice/debug_inject", 10)
    voice = VoiceClient(node)
    vocab = GuestVocabulary()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    t = time.time()
    while not voice.online() and time.time() - t < 5.0:
        time.sleep(0.1)
    if not voice.online():
        sys.exit("El ASR no publica /voice/status (Jetson/whisper_node apagados o ROS_DOMAIN_ID distinto)")
    time.sleep(0.5)

    rows, ok_count = [], 0
    sink = ["-d", args.sink] if args.sink else []
    print(f"{'#':>2} {'estado':9} {'ok':3} {'demora':>6}  {'entendido':22} texto")
    for rep in range(args.repeat):
        for i, (path, expected, remote_path) in enumerate(items):
            rid = voice.request(prompt=vocab.whisper_prompt())
            time.sleep(5.0 if args.late else args.delay)
            if remote_path:
                pub_inject.publish(String(data=json.dumps({"path": remote_path, "gain": args.gain})))
                time.sleep(wav_seconds(path))
            else:
                subprocess.run(["paplay"] + sink + [path])
            t_end = time.time()
            res = None
            while res is None and time.time() - t_end < 30:
                res = voice.poll(rid)
                time.sleep(0.02)
            latency = time.time() - t_end
            status = res.status if res else "sin_resp"
            good, got = check(expected, res.text if res else "", vocab)
            ok_count += good
            rows.append(dict(rep=rep, idx=i, expected=expected, status=status, text=res.text if res else "",
                             understood=got, correct=int(good), latency=round(latency, 2),
                             asr_sec=(res.info.get("asr_sec") if res else None),
                             speech_sec=(res.info.get("speech_sec") if res else None)))
            print(f"{i:2d} {status:9} {'SI' if good else 'NO':3} {latency:5.1f}s  {got:22} {res.text if res else ''}")
            time.sleep(0.8)
    n = len(rows)
    lat = sorted(r["latency"] for r in rows)
    print(f"\nEntendidas {ok_count}/{n} ({100.0 * ok_count / max(n, 1):.0f} %). Demora tras terminar de hablar: "
          f"mediana {lat[n // 2]:.1f} s, peor {lat[-1]:.1f} s")
    if args.csv:
        with open(args.csv, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
        print(f"CSV: {args.csv}")
    if not args.inject:
        keepalive.terminate()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
