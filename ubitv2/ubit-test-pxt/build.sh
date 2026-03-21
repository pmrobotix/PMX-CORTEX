#!/bin/bash
# Build du projet MakeCode micro:bit V2
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo "=== Build MakeCode micro:bit V2 ==="
pxt target microbit
pxt install
pxt build

echo "=== Build OK: built/binary.hex ==="
