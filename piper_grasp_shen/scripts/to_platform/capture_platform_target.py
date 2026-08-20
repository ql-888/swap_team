#!/usr/bin/env python3
"""Capture a stable Gemini Standard52h13 platform-tag pose."""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys


PROJECT_DIR = Path(__file__).resolve().parents[2]
CAPTURE = PROJECT_DIR / "scripts/capture_gemini_standard52_target.py"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--timeout-s", type=float, default=12.0)
    parser.add_argument("--maximum-rms-mm", type=float, default=10.0)
    args = parser.parse_args()
    if args.samples < 5:
        raise ValueError("--samples must be at least 5")
    command = [
        sys.executable,
        str(CAPTURE),
        "--base-frame", "piper_x/base_link",
        "--tag-frame", "gemini_standard52h13_0",
        "--samples", str(args.samples),
        "--timeout-s", str(args.timeout_s),
        "--maximum-rms-mm", str(args.maximum_rms_mm),
        "--output", str(args.output),
    ]
    return subprocess.run(command, cwd=PROJECT_DIR, check=False).returncode


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
