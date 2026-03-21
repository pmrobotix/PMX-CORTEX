#!/bin/bash
# Build et flash du projet ubit-test-pxt sur micro:bit V2
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MICROBIT_DRIVE="/media/$USER/MICROBIT"

echo "=== Build ==="
cd "$SCRIPT_DIR"
./build.sh

echo ""
echo "=== Flash ==="
if [ -d "$MICROBIT_DRIVE" ]; then
    cp "$SCRIPT_DIR/built/binary.hex" "$MICROBIT_DRIVE/"
    echo "Copié sur $MICROBIT_DRIVE — le micro:bit redémarre..."
else
    echo "ERREUR : micro:bit non détecté sur $MICROBIT_DRIVE"
    echo "Branche le micro:bit en USB et relance le script."
    exit 1
fi
