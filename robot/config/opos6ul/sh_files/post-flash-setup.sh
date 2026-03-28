#!/bin/bash
#
# post-flash-setup.sh
# Configuration d'une carte OPOS6UL apres un flash rootfs complet.
# A executer depuis le PC (VM Kubuntu), la carte doit etre accessible en RJ45.
#
# Usage : ./post-flash-setup.sh [IP_CARTE]
#   Par defaut : 192.168.0.44

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CARTE_IP="${1:-192.168.0.44}"
CARTE_USER="root"
SSH_OPTS="-o StrictHostKeyChecking=no -o LogLevel=ERROR"

echo "=== Post-flash setup pour OPOS6UL @ ${CARTE_IP} ==="

# --- 1. Changer le mot de passe root ---
echo ""
echo "[1/3] Changement du mot de passe root en 'pmx'..."
ssh ${SSH_OPTS} ${CARTE_USER}@${CARTE_IP} "echo 'root:pmx' | chpasswd"
echo "  -> OK"

# --- 2. Copier les fichiers de configuration ---
echo ""
echo "[2/3] Upload des fichiers de configuration..."

# S90opos6ulstart.sh
scp ${SSH_OPTS} "${SCRIPT_DIR}/S90opos6ulstart.sh" ${CARTE_USER}@${CARTE_IP}:/root/
ssh ${SSH_OPTS} ${CARTE_USER}@${CARTE_IP} "chmod +x /root/S90opos6ulstart.sh && ln -sf /root/S90opos6ulstart.sh /etc/init.d/S90opos6ulstart.sh"
echo "  -> S90opos6ulstart.sh installe"

# wpa_supplicant.conf
scp ${SSH_OPTS} "${SCRIPT_DIR}/wpa_supplicant" ${CARTE_USER}@${CARTE_IP}:/etc/wpa_supplicant.conf
echo "  -> wpa_supplicant.conf installe"

# --- 3. Synchro de l'heure ---
echo ""
echo "[3/3] Synchro de l'heure..."
ssh ${SSH_OPTS} ${CARTE_USER}@${CARTE_IP} "date -s '$(date -u '+%Y-%m-%d %H:%M:%S')'"
echo "  -> OK"

# --- 4. Partition 3 en F2FS pour les donnees robot ---
# Decommenter ce bloc pour formater et configurer la partition data en F2FS.
# ATTENTION : mkfs.f2fs efface toutes les donnees de la partition 3 !
#
# echo ""
# echo "[4/x] Configuration partition F2FS (/dev/mmcblk0p3 -> /root/pmx)..."
# ssh ${SSH_OPTS} ${CARTE_USER}@${CARTE_IP} bash -s <<'REMOTE'
#   # Demonter si montee
#   umount /dev/mmcblk0p3 2>/dev/null || true
#   # Commenter l'ancienne entree fstab si presente
#   sed -i 's|^/dev/mmcblk0p3|#/dev/mmcblk0p3|' /etc/fstab
#   # Formater en F2FS
#   mkfs.f2fs -f /dev/mmcblk0p3
#   # Creer le point de montage
#   mkdir -p /root/pmx
#   # Ajouter dans fstab
#   echo '/dev/mmcblk0p3   /root/pmx   f2fs    rw,acl,active_logs=6,background_gc=on,user_xattr   0 1' >> /etc/fstab
#   # Monter
#   mount /root/pmx
#   echo "Partition F2FS montee :"
#   df -h /root/pmx
# REMOTE
# echo "  -> OK"

echo ""
echo "=== Setup termine ! ==="
echo "Redemarrer la carte pour appliquer S90opos6ulstart.sh :"
echo "  ssh ${CARTE_USER}@${CARTE_IP} reboot"
