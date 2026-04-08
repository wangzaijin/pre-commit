#!/usr/bin/env bash
# Build the TongRobot dashboard frontend.
# Output: tongrobot_dashboard/frontend/dist/
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FRONTEND_DIR="$SCRIPT_DIR/frontend"

echo "[tongrobot-dashboard] Installing npm dependencies…"
cd "$FRONTEND_DIR"
npm install

echo "[tongrobot-dashboard] Building…"
npm run build

echo "[tongrobot-dashboard] Done. Output: $FRONTEND_DIR/dist/"
