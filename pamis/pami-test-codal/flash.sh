#!/bin/bash
# Build et flash du projet pami-test-codal sur micro:bit V2
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MICROBIT_DRIVE="/media/$USER/MICROBIT"

echo "=== Build ==="
cd "$SCRIPT_DIR"
python3 build.py

echo ""
echo "=== Flash ==="
if [ -d "$MICROBIT_DRIVE" ]; then
    cp MICROBIT.hex "$MICROBIT_DRIVE/"
    echo "Copié sur $MICROBIT_DRIVE — le micro:bit redémarre..."
else
    echo "ERREUR : micro:bit non détecté sur $MICROBIT_DRIVE"
    echo "Branche le micro:bit en USB et relance le script."
    exit 1
fi
