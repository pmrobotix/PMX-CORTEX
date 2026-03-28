#!/bin/bash
# ============================================================
# ssh-connect.sh — Connexion SSH interactive sur l'OPOS6UL
#
# Usage:
#   ./sh/ssh-connect.sh <ip>
#
# Exemples:
#   ./sh/ssh-connect.sh 1    # carte 1 (192.168.3.103)
#   ./sh/ssh-connect.sh 2    # carte 2 (192.168.3.104)
#   ./sh/ssh-connect.sh 3    # carte 3 (192.168.3.105)
#   ./sh/ssh-connect.sh 192.168.3.103   # IP directe aussi acceptee
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

sshpass -p $ROBOT_PASS ssh -t $SSH_OPTS $ROBOT_USER@$ROBOT_IP \
    "date -s '$(date -u '+%Y-%m-%d %H:%M:%S')' > /dev/null 2>&1; cd /root/pmx; exec /bin/sh -l"
