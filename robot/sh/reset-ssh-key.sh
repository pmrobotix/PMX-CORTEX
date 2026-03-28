#!/bin/bash
#
# reset-ssh-key.sh
# Supprime l'ancienne cle SSH et accepte la nouvelle apres un reflash de la carte.
#
# Usage : ./reset-ssh-key.sh <numero_carte>
#   1 = carte1 (192.168.3.103)
#   2 = carte2 (192.168.3.104)
#   3 = carte3 (192.168.3.105)

case "$1" in
    1) CARTE_IP="192.168.3.103"; CARTE_NAME="carte1" ;;
    2) CARTE_IP="192.168.3.104"; CARTE_NAME="carte2" ;;
    3) CARTE_IP="192.168.3.105"; CARTE_NAME="carte3" ;;
    *)
        echo "Usage : $0 <1|2|3>"
        echo "  1 = carte1 (192.168.3.103)"
        echo "  2 = carte2 (192.168.3.104)"
        echo "  3 = carte3 (192.168.3.105)"
        exit 1
        ;;
esac

echo "Reset SSH pour ${CARTE_NAME} @ ${CARTE_IP}..."
ssh-keygen -f "$HOME/.ssh/known_hosts" -R "${CARTE_IP}" 2>/dev/null

echo "Connexion et enregistrement de la nouvelle cle..."
ssh -o StrictHostKeyChecking=accept-new root@${CARTE_IP} "echo 'SSH OK pour ${CARTE_NAME} @ ${CARTE_IP}'"
