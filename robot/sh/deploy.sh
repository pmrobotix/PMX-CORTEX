#!/bin/bash
# ============================================================
# deploy.sh — Build, strip, deploy et run sur l'OPOS6UL
#
# Usage:
#   ./deploy.sh [ip] <target> [run]
#
# Si l'IP n'est pas specifiee, utilise 192.168.3.103 (5GHz) par defaut.
#
# Exemples:
#   ./deploy.sh driver-test                       # deploy sur 5g (defaut)
#   ./deploy.sh driver-test run                   # deploy + execute sur 5g
#   ./deploy.sh 192.168.2.105 driver-test run     # deploy + execute sur eth0
#
# Targets disponibles : opos6ul, driver-test, common-test, manual-test, bench
# ============================================================

set -e

ROBOT_IP="${1:-192.168.3.103}"
TARGET="${2:?Usage: $0 [ip] <target> [run]}"
RUN="${3:-}"

ROBOT_USER="root"
ROBOT_PASS="pmx"
ROBOT_DIR="/root/pmx"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROBOT_DIR_SRC="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$ROBOT_DIR_SRC/build-arm-release"
STRIP="/install/opos6ul-git/buildroot/output/host/opt/ext-toolchain/arm-none-linux-gnueabihf/bin/strip"

# --- Couleurs ---
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

info()  { echo -e "${GREEN}[deploy]${NC} $*"; }
warn()  { echo -e "${YELLOW}[deploy]${NC} $*"; }
error() { echo -e "${RED}[deploy]${NC} $*" >&2; }

# --- Verification sshpass ---
if ! command -v sshpass &>/dev/null; then
    error "sshpass non installe. Installer avec : sudo apt install sshpass"
    exit 1
fi

# Multiplexing SSH : une seule connexion reutilisee pour mkdir, scp, kill, run
CTRL_SOCK="/tmp/ssh-pmx-$ROBOT_IP"
SSH_OPTS="-o StrictHostKeyChecking=no -o ControlMaster=auto -o ControlPath=$CTRL_SOCK -o ControlPersist=30"
SSH_CMD="sshpass -p $ROBOT_PASS ssh $SSH_OPTS $ROBOT_USER@$ROBOT_IP"
SCP_CMD="sshpass -p $ROBOT_PASS scp $SSH_OPTS"

# --- Build ARM release ---
info "Build $TARGET (arm-release)..."
cd "$ROBOT_DIR_SRC"
cmake --preset arm-release 2>&1 | tail -1
cmake --build --preset arm-release --target "$TARGET" -j$(nproc) 2>&1 | tail -3

BUILD_DIR="$ROBOT_DIR_SRC/build-arm-release"
BINARY="$BUILD_DIR/$TARGET"

if [ ! -f "$BINARY" ]; then
    error "Binaire non trouve : $BINARY"
    exit 1
fi

# --- Strip ---
info "Strip $TARGET..."
$STRIP -s "$BINARY"
SIZE=$(du -h "$BINARY" | cut -f1)
info "Taille : $SIZE"

# --- Deploy ---
info "Deploy $TARGET vers $ROBOT_USER@$ROBOT_IP:$ROBOT_DIR/"
$SSH_CMD "mkdir -p $ROBOT_DIR" 2>/dev/null || true
$SCP_CMD "$BINARY" "$ROBOT_USER@$ROBOT_IP:$ROBOT_DIR/"

# --- Run (optionnel) ---
if [ "$RUN" = "run" ]; then
    # Kill l'ancien process s'il tourne (uniquement bot-opos6ul)
    if [ "$TARGET" = "bot-opos6ul" ]; then
        info "Kill $TARGET (si en cours)..."
        $SSH_CMD "killall $TARGET 2>/dev/null" || true
    fi
    info "Execution $TARGET sur la carte..."
    echo "----------------------------------------"
    $SSH_CMD "cd $ROBOT_DIR && ./$TARGET"
    echo "----------------------------------------"
    info "Termine."
else
    info "Deploy OK. Pour executer :"
    info "  ssh $ROBOT_USER@$ROBOT_IP \"cd $ROBOT_DIR && ./$TARGET\""
fi
