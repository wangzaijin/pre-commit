#!/usr/bin/env bash
# proto/build.sh — compile all .proto files into tongrobot/proto/
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
PROTO_DIR="$SCRIPT_DIR"
OUT_DIR="$REPO_ROOT/tongrobot/proto"

mkdir -p "$OUT_DIR"

echo "Compiling .proto files → $OUT_DIR"

python3 -m grpc_tools.protoc \
  -I "$PROTO_DIR" \
  --python_out="$OUT_DIR" \
  --grpc_python_out="$OUT_DIR" \
  "$PROTO_DIR/robot_state.proto" \
  "$PROTO_DIR/sensor_data.proto" \
  "$PROTO_DIR/commands.proto" \
  "$PROTO_DIR/bridge_service.proto"

echo "Fixing imports in generated *_pb2_grpc.py files..."
python3 "$SCRIPT_DIR/fix_imports.py" "$OUT_DIR"

# Ensure package __init__.py exists
touch "$OUT_DIR/__init__.py"

echo "Done. Generated files:"
ls "$OUT_DIR"/*.py
