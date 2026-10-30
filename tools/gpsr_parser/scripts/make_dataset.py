#!/usr/bin/env python3
"""Generate a dataset of GPSR commands using the official RoboCup@Home CommandGenerator.

Outputs data/commands.jsonl with >= 300 entries.
Each line contains:
{
  "id": int,
  "category": str,
  "command": str
}
"""

import argparse
import json
import os
import random
import sys
from pathlib import Path

from robocupathome_generator.generator import createGPSRGenerator


def generate_dataset(data_dir: str, output_path: str, count: int = 500, seed: int = 42):
    random.seed(seed)
    generator = createGPSRGenerator(data_dir)

    out_file = Path(output_path)
    out_file.parent.mkdir(parents=True, exist_ok=True)

    categories = ["", "people", "objects"]
    records = []

    print(f"Generating {count} commands using data from {data_dir}...")
    for idx in range(count):
        cat = categories[idx % len(categories)]
        raw_cmd = generator.generate_command_start(cmd_category=cat)
        # Normalise clean text
        cmd_text = " ".join(raw_cmd.split())
        record = {
            "id": idx + 1,
            "category": cat if cat else "mixed",
            "command": cmd_text,
        }
        records.append(record)

    with open(out_file, "w", encoding="utf-8") as f:
        for r in records:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")

    print(f"Successfully wrote {len(records)} commands to {output_path}")


def main():
    default_base = Path(__file__).resolve().parent.parent
    default_data_dir = default_base / "data"
    default_output = default_base / "data" / "commands.jsonl"

    parser = argparse.ArgumentParser(description="Generate GPSR command dataset")
    parser.add_argument(
        "-d", "--data-dir",
        default=str(default_data_dir),
        help="Path to competition data directory (maps, names, objects)",
    )
    parser.add_argument(
        "-o", "--output",
        default=str(default_output),
        help="Path to output JSONL file",
    )
    parser.add_argument(
        "-n", "--count",
        type=int,
        default=500,
        help="Number of commands to generate (minimum 300)",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Random seed for reproducibility",
    )
    args = parser.parse_args()

    if not os.path.isdir(args.data_dir):
        print(f"Error: Data directory not found: {args.data_dir}", file=sys.stderr)
        sys.exit(1)

    generate_dataset(args.data_dir, args.output, count=args.count, seed=args.seed)


if __name__ == "__main__":
    main()
