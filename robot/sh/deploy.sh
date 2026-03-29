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
#   ./sh/deploy.sh 1 bench                # deploy bench sur carte 1
#   ./sh/deploy.sh 2 bot-opos6ul          # deploy bot sur carte 2
#   ./sh/deploy.sh 3 driver-test          # deploy driver-test sur carte 3
#   ./sh/deploy.sh 1 bench run            # deploy + execute bench sur carte 1
#   ./sh/deploy.sh 3 manual-test run      # deploy + execute manual-test sur carte 3
#   ./sh/deploy.sh 192.168.3.103 bench    # IP directe aussi acceptee
#
# Cartes :
#   carte 1 = 192.168.3.103
#   carte 2 = 192.168.3.104
#   carte 3 = 192.168.3.105
#
# Targets disponibles : bot-opos6ul, driver-test, common-test, manual-test, bench
# ============================================================

set -e

# Resolution IP : accepte 1, 2, 3 ou une IP directe
resolve_ip() {
    case "$1" in
        1) echo "192.168.3.103" ;;
        2) echo "192.168.3.104" ;;
        3) echo "192.168.3.105" ;;
        *) echo "$1" ;;
    esac
}
ROBOT_IP=$(resolve_ip "${1:-1}")
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
SSH_OPTS="-o StrictHostKeyChecking=no -o ControlMaster=auto -o ControlPath=$CTRL_SOCK -o ControlPersist=yes"
SSH_CMD="sshpass -p $ROBOT_PASS ssh $SSH_OPTS $ROBOT_USER@$ROBOT_IP"
SCP_CMD="sshpass -p $ROBOT_PASS scp $SSH_OPTS"

# --- Build ARM release ---
info "Build $TARGET (arm-release)..."
cd "$ROBOT_DIR_SRC"
cmake --preset arm-release 2>&1 | tail -1
cmake --build --preset arm-release --target "$TARGET" -j6 2>&1 | tail -3

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
