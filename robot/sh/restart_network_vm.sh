#!/bin/bash
# Libérer l'ancien bail DHCP et en demander un nouveau
sudo dhclient -r
sudo dhclient

# Restart network on Kubuntu VM
echo "Restarting NetworkManager..."
sudo systemctl restart NetworkManager
echo "Network restarted."

sleep 2
#ip a
ifconfig
