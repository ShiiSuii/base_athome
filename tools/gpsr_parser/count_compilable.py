#!/usr/bin/env python3
"""Cuenta compile_gpsr_actions sobre los 500 comandos, sin ROS ni red.

Ejemplo: python3 tools/gpsr_parser/count_compilable.py --baseline-ref 4e56930
Los lugares oficiales usan coordenadas ficticias: solo mide compilacion.
La referencia base cambia unicamente command_parser.py; los demas modulos
y los datos son los del worktree para comparar la misma entrada.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import types

ROOT = Path(__file__).resolve().parents[2]
COMPILER_PATH = 'src/base_athome_skills/base_athome_skills/mission/command_parser.py'
sys.path.insert(0, str(ROOT / 'src/base_athome_skills'))

from base_athome_skills.mission.command_parser import compile_gpsr_actions
from base_athome_skills.mission.gpsr_plan import gpsr_actions


def baseline_compiler(ref):
    revision = subprocess.check_output(
        ['git', 'rev-parse', '--verify', ref + '^{commit}'], cwd=ROOT, text=True).strip()
    source = subprocess.check_output(
        ['git', 'show', revision + ':' + COMPILER_PATH], cwd=ROOT, text=True)
    module = types.ModuleType('base_athome_skills.mission._coverage_baseline')
    module.__package__ = 'base_athome_skills.mission'
    sys.modules[module.__name__] = module
    exec(compile(source, revision + ':' + COMPILER_PATH, 'exec'), module.__dict__)
    return revision, module.compile_gpsr_actions


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline-ref', help='commit local del compilador anterior (sin checkout)')
    args = parser.parse_args()
    rows = [json.loads(line) for line in
            (ROOT / 'tools/gpsr_parser/data/commands.jsonl').read_text().splitlines() if line.strip()]
    actions = [gpsr_actions(row['command']) for row in rows]
    places = {str(a['params'][key]).replace(' ', '_').replace('-', '_'): (0., 0., 0.)
              for command in actions for a in command for key in ('location', 'destination')
              if a.get('params', {}).get(key)}
    compilers = []
    if args.baseline_ref:
        revision, compiler = baseline_compiler(args.baseline_ref)
        compilers.append(('baseline ' + revision[:7], compiler))
    compilers.append(('worktree', compile_gpsr_actions))
    for label, compiler in compilers:
        count = sum(compiler(command, places).ok for command in actions)
        print(f'{label}: {count}/{len(rows)} ({100. * count / len(rows):.1f}%)')


if __name__ == '__main__':
    main()
