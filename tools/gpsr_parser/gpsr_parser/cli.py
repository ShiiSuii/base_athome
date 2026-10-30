"""CLI entrypoint for GPSR parser.

Usage:
    python -m gpsr_parser "bring me a coke from the kitchen"
    python -m gpsr_parser.cli "meet Angel in the bedroom and follow them"
"""

import argparse
import json
import sys

from gpsr_parser.parser import GPSRParser


def main():
    arg_parser = argparse.ArgumentParser(
        description="Offline GPSR natural language command parser (closed-vocabulary action planner)",
    )
    arg_parser.add_argument(
        "command",
        nargs="*",
        help="Command text to parse (e.g. 'bring me a coke from the kitchen')",
    )
    arg_parser.add_argument(
        "-d", "--data-dir",
        default=None,
        help="Path to competition data directory",
    )
    arg_parser.add_argument(
        "--json-only",
        action="store_true",
        help="Output raw JSON plan only",
    )

    args = arg_parser.parse_args()

    if not args.command:
        arg_parser.print_help()
        sys.exit(1)

    full_cmd = " ".join(args.command).strip()
    parser = GPSRParser(data_dir=args.data_dir)
    result = parser.parse(full_cmd)

    if args.json_only:
        print(json.dumps(result.to_dict(), indent=2, ensure_ascii=False))
        sys.exit(0 if result.success else 1)

    print("=== GPSR PLAN ===")
    print(f"Command: \"{result.command}\"")
    print(f"Success: {result.success}")
    if result.error:
        print(f"Error: {result.error}")
    print("\nPlan JSON:")
    print(json.dumps(result.actions, indent=2, ensure_ascii=False))
    print("\nRobot Speech Demonstration (HRI):")
    print(f"\"{result.explanation}\"")

    sys.exit(0 if result.success else 1)


if __name__ == "__main__":
    main()
