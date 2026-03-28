#!/bin/bash
# ============================================================
# recup-svg.sh — Rapatrie les fichiers SVG depuis l'OPOS6UL
#
# Usage:
#   ./sh/recup-svg.sh <ip>
#
# Exemples:
#   ./sh/recup-svg.sh 1    # carte 1 (192.168.3.103)
#   ./sh/recup-svg.sh 2    # carte 2 (192.168.3.104)
#   ./sh/recup-svg.sh 3    # carte 3 (192.168.3.105)
#   ./sh/recup-svg.sh 192.168.3.103   # IP directe aussi acceptee
# ============================================================

resolve_ip() {
    case "$1" in
        1) echo "192.168.3.103" ;;
        2) echo "192.168.3.104" ;;
        3) echo "192.168.3.105" ;;
        *) echo "$1" ;;
    esac
}
ROBOT_IP=$(resolve_ip "${1:?Usage: $0 <1|2|3|ip>}")
ROBOT_USER="root"
ROBOT_PASS="pmx"
CTRL_SOCK="/tmp/ssh-pmx-$ROBOT_IP"
SSH_OPTS="-o StrictHostKeyChecking=no -o ControlMaster=auto -o ControlPath=$CTRL_SOCK -o ControlPersist=yes"
DEST_DIR="/home/pmx/recup"

mkdir -p "$DEST_DIR"

sshpass -p $ROBOT_PASS scp $SSH_OPTS \
    $ROBOT_USER@$ROBOT_IP:/root/pmx/*.svg "$DEST_DIR/"

echo "SVG rapatries dans $DEST_DIR/"
ls -la "$DEST_DIR/"*.svg 2>/dev/null
