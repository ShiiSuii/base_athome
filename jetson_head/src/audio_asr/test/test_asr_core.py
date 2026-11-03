import numpy as np

from audio_asr.asr_core import CHUNK_SEC, VAD_CHUNK, Endpointer, clean_transcript, collapse_repeats

Z = np.zeros(VAD_CHUNK, np.float32)
V = np.full(VAD_CHUNK, 0.1, np.float32)


def run(ep, pattern, floor=None, loud=None):
    """pattern: lista de (segundos, prob, muted)."""
    for sec, prob, muted in pattern:
        for _ in range(int(round(sec / CHUNK_SEC))):
            sig = loud if (loud is not None and prob > 0.1) else (V if prob > 0.4 else Z)
            out = ep.feed(sig, prob, muted, floor=floor)
            if out is not None:
                return out
    return None


def test_ends_after_silence_not_fixed_window():
    ep = Endpointer(start_timeout=8, end_silence=1.0, max_speech=15)
    out = run(ep, [(0.5, 0.0, False), (2.0, 0.9, False), (3.0, 0.0, False)])
    assert out['status'] == 'ok' and out['reason'] == 'end_silence'
    assert 1.9 <= out['speech_sec'] <= 2.2
    # audio = voz + ~0.3 s de cola (no los 6 s de antes)
    assert len(out['audio']) / 16000 < 2.8


def test_late_speaker_is_not_lost():
    ep = Endpointer(start_timeout=8, end_silence=1.0)
    out = run(ep, [(6.5, 0.0, False), (2.0, 0.9, False), (2.0, 0.0, False)])
    assert out['status'] == 'ok'


def test_no_speech_reports_quickly():
    ep = Endpointer(start_timeout=3, end_silence=1.0)
    out = run(ep, [(10.0, 0.1, False)])
    assert out['status'] == 'no_speech' and abs(out['wait_sec'] - 3.0) < 0.1


def test_pause_inside_sentence_does_not_cut():
    ep = Endpointer(start_timeout=8, end_silence=1.1)
    out = run(ep, [(1.5, 0.9, False), (0.7, 0.1, False), (1.5, 0.9, False), (2.0, 0.0, False)])
    assert out['status'] == 'ok' and out['speech_sec'] > 2.9


def test_robot_speaking_pauses_timeout_and_is_zeroed():
    ep = Endpointer(start_timeout=2, end_silence=1.0)
    out = run(ep, [(5.0, 0.9, True), (1.0, 0.0, False), (1.5, 0.9, False), (2.0, 0.0, False)])
    assert out['status'] == 'ok'   # los 5 s de mute no consumieron el start_timeout
    assert out['wait_sec'] < 1.5


def test_click_is_not_speech():
    ep = Endpointer(start_timeout=2, end_silence=1.0)
    out = run(ep, [(0.05, 0.9, False), (3.0, 0.0, False)])
    assert out['status'] == 'no_speech'


def test_max_speech_cuts_long_monologue():
    ep = Endpointer(start_timeout=8, end_silence=1.0, max_speech=4.0)
    out = run(ep, [(10.0, 0.9, False)])
    assert out['status'] == 'ok' and out['reason'] == 'max_speech'


def test_preroll_keeps_sentence_start():
    pre = [V] * 10
    ep = Endpointer(start_timeout=8, end_silence=1.0, pre_roll=pre)
    out = run(ep, [(1.0, 0.9, False), (2.0, 0.0, False)])
    assert len(out['audio']) >= (10 + 30) * VAD_CHUNK


def test_text_filters():
    assert clean_transcript('Thank you.')[1] == 'hallucination'
    assert clean_transcript('')[1] == 'empty'
    assert clean_transcript('[MUSIC]')[1] == 'non_speech_tag'
    assert clean_transcript('No.')[1] is None
    assert clean_transcript('Yes')[1] is None
    prompt = 'Names: Adel, Angel, Axel, Charlie, Jane, Jules, Morgan. Drinks: cola, milk, iced tea.'
    assert clean_transcript('Adel, Angel, Axel, Charlie, Jane, Jules.', prompt)[1] == 'prompt_echo'
    assert clean_transcript('My name is Jules and my favorite drink is milk.', prompt)[1] is None


def test_collapse_repeats():
    t = 'My name is Robin, and my favorite drink is Ice Tea, Robin, Robin, Robin, Robin, Robin, Robin'
    assert collapse_repeats(t).count('Robin') <= 3
    t2 = 'My name is Morgan and I like milk. My name is Morgan and I like milk. My name is Morgan and I like milk.'
    assert collapse_repeats(t2).count('Morgan') <= 2
    assert collapse_repeats('no no') == 'no no'


def test_weak_far_voice_is_accepted_when_clearly_above_noise():
    ep = Endpointer(start_timeout=4, end_silence=1.0)
    loud = np.full(VAD_CHUNK, 0.05, np.float32)      # 5x el ruido de fondo, Silero solo 0.25
    out = run(ep, [(0.5, 0.0, False), (2.0, 0.25, False), (2.0, 0.0, False)], floor=0.01, loud=loud)
    assert out['status'] == 'ok' and out['weak']


def test_weak_prob_without_energy_is_not_speech():
    ep = Endpointer(start_timeout=3, end_silence=1.0)
    quiet = np.full(VAD_CHUNK, 0.015, np.float32)    # 1.5x el ruido: no alcanza
    out = run(ep, [(4.0, 0.25, False)], floor=0.01, loud=quiet)
    assert out['status'] == 'no_speech'
