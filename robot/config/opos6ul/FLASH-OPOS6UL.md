# Flasher une carte OPOS6UL

Guide pour recompiler et flasher le firmware (kernel + rootfs) sur les cartes OPOS6UL (Armadeus 7.0).

## Contexte

La carte OPOS6UL utilise un BSP Armadeus basé sur Buildroot. Pour ajouter/modifier des paquets, il faut :
1. Modifier la configuration Buildroot
2. Recompiler
3. Flasher les images sur la carte

---

## Pre-requis

- BSP Armadeus 7.0 installe dans `/install/opos6ul-git/`
- Connexion serie a la carte (voir [Connexion serie OPOS6UL](https://github.com/pmrobotix/PMX-CORTEX/wiki/Connexion-s%C3%A9rie-OPOS6UL))
- Cable USB serie + minicom (`minicom -D /dev/ttyUSB0`)
- `sshpass` installe (`sudo apt install sshpass`)

---

## 1. Modifier la configuration Buildroot

```bash
cd /install/opos6ul-git/buildroot
```

### Paquets modifies (2026-03-27)

| Paquet | Ancienne version | Nouvelle version | Fichiers modifies |
|---|---|---|---|
| Dropbear (SSH) | 2020.81 | 2024.86 | `package/dropbear/dropbear.mk`, `dropbear.hash` |
| dhcpcd (DHCP) | 9.4.1 (non installe) | 10.3.1 | `package/dhcpcd/dhcpcd.mk`, `dhcpcd.hash` |

**Dropbear 2020.81** avait un probleme de blocage au key exchange sur ARM (lié a `/dev/random` bloquant). La 2024.86 corrige ce probleme.

**dhcpcd** : l'ancienne source (roy.marples.name) n'est plus disponible. La nouvelle source est GitHub.

### Ajouter/modifier des paquets

```bash
make menuconfig
```

Naviguer vers :
```
Target packages
  -> Networking applications
     [*] dhcpcd          # client DHCP (ajoute en 2026)
```

### Alternative sans reflash : utiliser udhcpc (BusyBox)

`udhcpc` est deja inclus via BusyBox. On peut l'utiliser pour le WiFi dans le script de demarrage :

```bash
# Dans S90opos6ulstart.sh, remplacer :
#   ifconfig wlan0 192.168.3.103 netmask 255.255.255.0 up
# Par :
udhcpc -i wlan0 -n    # -n = ne pas bloquer si pas de reponse
```

Si `udhcpc` fonctionne pour wlan0, **pas besoin de reflasher**.

### Mettre a jour la version d'un paquet Buildroot

Pour mettre a jour un paquet (ex: dropbear) :

1. Modifier la version dans `package/<paquet>/<paquet>.mk`
2. Mettre a jour le hash SHA256 dans `package/<paquet>/<paquet>.hash`
3. Mettre a jour l'URL source si necessaire
4. Nettoyer et recompiler :

```bash
make <paquet>-dirclean
make
```

---

## 2. Fix appliques au script de demarrage (S90opos6ulstart.sh)

### Fix entropie pour Dropbear SSH

Dropbear lit `/dev/random` (bloquant) pour le key exchange. Sur un systeme embarque avec peu de sources d'entropie, ca bloque la connexion SSH apres quelques tentatives.

**Solution** : rediriger `/dev/random` vers `/dev/urandom` au boot. Sur Linux 5.10+, `/dev/urandom` est cryptographiquement sur apres le boot initial.

```bash
# Ajoute dans S90opos6ulstart.sh
if [ -c /dev/random ] && [ ! -L /dev/random ]; then
    rm /dev/random
    ln -s /dev/urandom /dev/random
fi
```

### Synchro de l'heure via SSH

La carte n'a pas de pile RTC, l'heure est perdue a chaque reboot. La synchro se fait automatiquement a chaque connexion SSH depuis la VM via la tache VSCode "SSH" :

```bash
ssh root@<IP> "date -s '$(date -u '+%Y-%m-%d %H:%M:%S')'"
```

---

## 3. Recompiler le systeme

```bash
cd /install/opos6ul-git/buildroot

# Recompilation complete (si modification kernel + rootfs)
make

# Ou recompiler seulement le rootfs (si ajout de paquets uniquement)
make rootfs-rebuild
make
```

Les images sont generees dans :
```
buildroot/output/images/
```

Fichiers generes :
| Fichier | Contenu |
|---|---|
| `rootfs.tar` | Systeme de fichiers racine |
| `rootfs.ext2` | Image ext2 du rootfs |
| `zImage` | Kernel Linux compresse |
| `imx6ul-opos6uldev.dtb` | Device Tree blob |
| `u-boot.imx` | Bootloader U-Boot |

---

## 4. Flasher la carte OPOS6UL

### Methode 1 : Flash via U-Boot + TFTP (recommande)

#### Configuration reseau

Le PC et la carte doivent etre sur le meme reseau Ethernet.

| | Valeur par defaut |
|---|---|
| IP carte (U-Boot) | Voir `bios_oposulX.txt` (ex: `192.168.0.220`) |
| IP serveur TFTP | Voir `bios_oposulX.txt` (ex: `192.168.0.218`) |
| Interface | eth0 (cable RJ45) |

#### Installer et demarrer le serveur TFTP sur le PC

```bash
sudo apt install tftpd-hpa

# Verifier que le service tourne
sudo systemctl status tftpd-hpa

# Demarrer le service (session courante uniquement)
sudo systemctl start tftpd-hpa

# Activer au demarrage (persistant apres reboot)
sudo systemctl enable tftpd-hpa

# Ou les deux en une seule commande
sudo systemctl enable --now tftpd-hpa
```

#### Desactiver le firewall (necessaire pour TFTP)

Le TFTP utilise UDP port 69, souvent bloque par le firewall Ubuntu (ufw).

```bash
# Verifier l'etat du firewall
sudo ufw status

# Option 1 : desactiver completement le firewall
sudo ufw disable

# Option 2 : autoriser uniquement le TFTP (UDP 69) depuis le reseau local
sudo ufw allow from 192.168.0.0/24 to any port 69 proto udp
```

**Note** : si le ping fonctionne mais que le TFTP reste bloque en attente, c'est probablement le firewall.

Le repertoire TFTP est configure dans `/etc/default/tftpd-hpa` :

```
TFTP_USERNAME="tftp"
TFTP_DIRECTORY="/tftpboot"
TFTP_ADDRESS=":69"
TFTP_OPTIONS="--secure"
```

Pour changer le repertoire, modifier `TFTP_DIRECTORY` puis `sudo systemctl restart tftpd-hpa`.

#### Entrer dans U-Boot

1. Se connecter en serie : `minicom -D /dev/ttyUSB0`
2. Redemarrer la carte (reset ou power cycle)
3. Appuyer sur une touche pendant le countdown pour interrompre le boot
4. Le prompt `BIOS>` apparait

#### Verifier la configuration reseau

```
BIOS> printenv ipaddr serverip
```

Valeurs attendues (carte 1 robot) :

| Variable | Valeur |
|---|---|
| `ipaddr` | `192.168.0.220` (IP de la carte) |
| `serverip` | `192.168.0.218` (IP du PC/VM avec le serveur TFTP) |

Si besoin de modifier :

```
BIOS> setenv ipaddr 192.168.0.220
BIOS> setenv serverip 192.168.0.218
BIOS> saveenv
```

**Note** : le PC (VM Kubuntu) doit etre connecte en **RJ45** sur le meme reseau (192.168.0.x).

Configurer l'interface Ethernet de la VM avec l'IP `serverip` :

```bash
# Sur la VM Kubuntu, configurer l'IP du serveur TFTP
sudo ifconfig eth0 192.168.0.218 netmask 255.255.255.0 up
```

Pour tester la connexion depuis U-Boot :

```
BIOS> dhcp
```

#### Fichiers dans /tftpboot/

Les images buildroot sont copiees dans `/tftpboot/` avec des **liens symboliques** vers les noms attendus par U-Boot :

| Fichier attendu par U-Boot | → | Fichier reel (buildroot) | Lien |
|---|---|---|---|
| `opos6ul-linux.bin` | → | `zImage` | symlink |
| `imx6ull-opos6uldev.dtb` | → | `imx6ul-opos6uldev.dtb` | copie directe |
| `opos6ul-rootfs.ext4` | → | `rootfs.ext2` | symlink (via rootfs.ext4) |
| `opos6ul-u-boot.spl` | → | `SPL` | symlink |
| `opos6ul-u-boot.img` | → | `u-boot-dtb.img` | symlink |

**Note** : le DTB est `imx6ull-opos6uldev.dtb` (cpu_type=`imx6ull`, fdt_name=`opos6uldev`), pas `imx6ul-`.

#### Copier les nouvelles images apres recompilation

```bash
# Copier les fichiers reels (les liens symboliques restent en place)
sudo cp /install/opos6ul-git/buildroot/output/images/zImage /tftpboot/
sudo cp /install/opos6ul-git/buildroot/output/images/imx6ul-opos6uldev.dtb /tftpboot/
sudo cp /install/opos6ul-git/buildroot/output/images/rootfs.ext2 /tftpboot/
sudo cp /install/opos6ul-git/buildroot/output/images/rootfs.tar /tftpboot/
```

Les modules kernel sont aussi disponibles :
```bash
# Si les modules ont change
sudo cp /install/opos6ul-git/buildroot/output/images/opos6ul-modules.tar.gz /tftpboot/
```

#### Creer les liens symboliques (premiere fois uniquement)

```bash
cd /tftpboot
ln -sf zImage opos6ul-linux.bin
ln -sf imx6ul-opos6uldev.dtb opos6ul.dtb
ln -sf rootfs.ext2 rootfs.ext4
ln -sf rootfs.ext4 opos6ul-rootfs.ext4
ln -sf rootfs.tar opos6ul-rootfs.tar
ln -sf SPL opos6ul-u-boot.spl
ln -sf u-boot-dtb.img opos6ul-u-boot.img
```

#### Flash du kernel + DTB

```
BIOS> run update_kernel
BIOS> run update_dtb
```

#### Flash du rootfs (systeme de fichiers complet)

```
BIOS> run update_rootfs
```

**Attention** : le flash du rootfs ecrase tout le systeme de fichiers (partition 2). Les fichiers dans `/root/` seront perdus (scripts, binaires). Sauvegarder avant si necessaire.

#### Flash complete (rootfs + U-Boot)

```
BIOS> run update_all
```

#### Redemarrer apres le flash

```
BIOS> reset
```

#### Resume des commandes U-Boot disponibles

| Commande | Action |
|---|---|
| `run update_kernel` | Flash le kernel seul |
| `run update_dtb` | Flash le device tree seul |
| `run update_rootfs` | Flash le rootfs (partition 2) |
| `run update_uboot` | Flash le bootloader U-Boot |
| `run update_all` | Flash rootfs + U-Boot |
| `run update_userdata` | Flash la partition user data (partition 3) |
| `printenv` | Afficher toutes les variables |
| `saveenv` | Sauvegarder les variables modifiees |
| `reset` | Redemarrer la carte |

### Methode 2 : Copie directe via SCP (rootfs seulement)

Si on veut juste ajouter des binaires sans reflasher tout le rootfs :

```bash
# Compiler le paquet voulu
cd ~/armadeus-7.0/buildroot
make dhcpcd-rebuild

# Trouver le binaire
find output/target -name "dhcpcd"

# Copier sur la carte
scp output/target/usr/sbin/dhcpcd root@<IP_CARTE>:/usr/sbin/
```

### Methode 3 : Flash complete via carte SD

Voir la documentation Armadeus :
- https://www.armadeus.org/wiki/index.php?title=Flashing

---

## 5. Apres le flash

### Verifier le bon fonctionnement

```bash
# Se connecter en serie
minicom -D /dev/ttyUSB0

# Apres boot, verifier les paquets
which dhcpcd     # ou which dhclient
dhcpcd --version
```

### Remettre la configuration WiFi

Copier les fichiers de configuration sur la carte :

```bash
# Depuis le PC (adapter l'IP)
scp config/opos6ul/sh_files/S90opos6ulstart.sh root@<IP_CARTE>:/root/
scp config/opos6ul/sh_files/wpa_supplicant root@<IP_CARTE>:/etc/wpa_supplicant.conf

# Sur la carte : creer le lien de demarrage auto
ln -sf /root/S90opos6ulstart.sh /etc/init.d/S90opos6ulstart.sh
```

### Configuration WiFi (wpa_supplicant.conf)

Le fichier est sauvegarde dans `config/opos6ul/sh_files/wpa_supplicant` :

```
ctrl_interface=/var/run/wpa_supplicant
ap_scan=1

network={
    ssid="PMX2_AP_5G"
    psk="LLLLLLLL"
}
```

**Remplacer `LLLLLLLL` par le vrai mot de passe WiFi !**

### Adresses IP WiFi des cartes

| Carte | IP WiFi statique | Commentaire |
|---|---|---|
| Carte 1 (robot) | `192.168.3.102` | Dans le robot |
| Carte 2 | `192.168.3.103` | Avec LCD |
| Carte 3 | *(non configuree)* | Spare |

---

## 6. Diagnostic reseau WiFi

### Probleme : la carte n'apparait pas dans la liste DHCP du routeur TP-Link

Causes possibles :
1. **Le module WiFi RTL8812AU n'est pas charge** → verifier `lsmod | grep 8812`
2. **wpa_supplicant n'a pas reussi l'association** → verifier `iwconfig wlan0`
3. **IP statique configuree** → la carte n'envoie pas de requete DHCP au routeur, donc elle n'apparait pas dans la liste DHCP du TP-Link (c'est normal avec une IP statique)
4. **Le SSID ne correspond pas** → verifier `/etc/wpa_supplicant.conf`

### Commandes de diagnostic (sur la carte via serie)

```bash
# Verifier que le module WiFi est charge
lsmod | grep 8812

# Verifier l'association WiFi
iwconfig wlan0
# Doit montrer : ESSID:"PMX2_AP_5G" et Access Point: <MAC du routeur>

# Verifier l'IP
ifconfig wlan0

# Scanner les reseaux disponibles
iwlist wlan0 scan | grep ESSID

# Relancer manuellement le WiFi
ifconfig wlan0 up
wpa_supplicant -i wlan0 -c /etc/wpa_supplicant.conf -B
# Attendre quelques secondes
iwconfig wlan0
ifconfig wlan0 192.168.3.103 netmask 255.255.255.0 up

# Tester la connectivite
ping 192.168.3.1       # routeur TP-Link
ping 192.168.3.100     # PC Windows
```

### Probleme : le PC ne ping pas la carte

Si `iwconfig wlan0` montre `Access Point: Not-Associated` :
- Le WiFi n'est pas connecte au routeur TP-Link
- Verifier le SSID et le mot de passe dans `wpa_supplicant.conf`
- Verifier que le routeur TP-Link est en mode **AP** (pas en mode client/repeater)
- Essayer de rapprocher la carte du routeur (signal 5GHz porte moins loin)

---

## Inventaire des cartes

Voir [CONFIG-STATUS.md](CONFIG-STATUS.md) pour les details de chaque carte (MAC, IP, modules actifs, etc.).

---

## Liens utiles

- [Wiki Armadeus - Flashing](https://www.armadeus.org/wiki/index.php?title=Flashing)
- [Wiki Armadeus - Toolchain](https://www.armadeus.org/wiki/index.php?title=Toolchain)
- [Wiki PMX - DevEnv OPOS6UL](https://github.com/pmrobotix/PMX/wiki/DevEnv-OPOS6UL-Compilation)
- [Wiki PMX-CORTEX - Connexion serie](https://github.com/pmrobotix/PMX-CORTEX/wiki/Connexion-s%C3%A9rie-OPOS6UL)
