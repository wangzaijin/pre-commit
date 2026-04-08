#!/usr/bin/env python3
"""Fix absolute imports in protoc-generated *_pb2_grpc.py files.

protoc generates:
    import robot_state_pb2 as robot__state__pb2
We need:
    from . import robot_state_pb2 as robot__state__pb2

This is required for the generated files to work as part of the
tongrobot.proto package.
"""
import re
import sys
from pathlib import Path

PATTERN = re.compile(r"^import (\w+_pb2(?:_grpc)?) as (.+)$", re.MULTILINE)


def fix_file(path: Path) -> None:
    text = path.read_text()
    fixed = PATTERN.sub(r"from . import \1 as \2", text)
    if fixed != text:
        path.write_text(fixed)
        print(f"  Fixed: {path.name}")
    else:
        print(f"  OK (no change): {path.name}")


def main() -> None:
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <output_dir>")
        sys.exit(1)
    out_dir = Path(sys.argv[1])
    for pb2_file in sorted(out_dir.glob("*_pb2*.py")):
        fix_file(pb2_file)


if __name__ == "__main__":
    main()
