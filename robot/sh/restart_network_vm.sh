#!/bin/bash
# net-restart.sh — Restart réseau pour VM Kubuntu (NAT ou Bridge selon VMware)

set -u

echo "=== Network restart ==="

# 1. Tue les éventuels dhclient zombies (au cas où l'ancien script en a laissé)
sudo pkill -9 dhclient 2>/dev/null && echo "Killed stale dhclient processes"

# 2. Restart NetworkManager
echo "Restarting NetworkManager..."
sudo systemctl restart NetworkManager
sleep 3

# 3. Détecte l'interface avec carrier (câble virtuel branché côté VMware)
ACTIVE_IFACE=""
for iface in ens33 ens38; do
    if [ -f "/sys/class/net/$iface/carrier" ] && [ "$(cat /sys/class/net/$iface/carrier 2>/dev/null)" = "1" ]; then
        ACTIVE_IFACE=$iface
        break
    fi
done

if [ -z "$ACTIVE_IFACE" ]; then
    echo "⚠️  Aucune interface avec carrier détectée."
    echo "    Vérifie dans VMware: VM > Settings > Network Adapter > Connected"
    nmcli device status
    exit 1
fi

echo "Active interface detected: $ACTIVE_IFACE"

# 4. Active la connexion correspondante
case "$ACTIVE_IFACE" in
    ens33) CONN="nat-ens33" ;;
    ens38) CONN="bridge-ens38" ;;
esac

sudo nmcli connection up "$CONN" || {
    echo "⚠️  Échec activation $CONN, tentative via device connect..."
    sudo nmcli device connect "$ACTIVE_IFACE"
}

sleep 2

# 5. Résumé
echo ""
echo "=== Status ==="
nmcli device status
echo ""
ip -brief addr show "$ACTIVE_IFACE"
echo ""
echo "=== Routes ==="
ip route
echo ""

# 6. Test connectivité (avec timeout pour pas bloquer)
echo "=== Connectivity test ==="
if timeout 3 ping -c 1 -W 2 8.8.8.8 >/dev/null 2>&1; then
    echo "✅ IP connectivity OK (8.8.8.8 reachable)"
else
    echo "❌ Pas de connectivité IP — vérifie le portail captif côté host Windows"
    exit 1
fi

if timeout 3 ping -c 1 -W 2 google.com >/dev/null 2>&1; then
    echo "✅ DNS OK"
else
    echo "⚠️  IP OK mais DNS KO — tente: sudo resolvectl dns $ACTIVE_IFACE 8.8.8.8 1.1.1.1"
fi