# Configuration OPOS6UL - Sauvegarde du 2026-03-27

Source : `/install/opos6ul-git/` (Armadeus 7.0)
Reference wiki PMX : https://github.com/pmrobotix/PMX/wiki/DevEnv-OPOS6UL-Compilation

## Fichiers sauvegardes et chemins d'origine

### Buildroot
| Fichier local | Chemin d'origine |
|---|---|
| `buildroot/buildroot.config` | `buildroot/.config` (2 mai 2023) |
| `buildroot/opos6ul_defconfig` | `buildroot/configs/opos6ul_defconfig` |

### Kernel Linux 5.10.167
| Fichier local | Chemin d'origine |
|---|---|
| `kernel/linux-5.10.167.config` | `buildroot/output/build/linux-5.10.167/.config` (29 avril 2023) |
| `kernel/opos6ul-linux-5.10.config` | `buildroot/target/device/armadeus/opos6ul/opos6ul-linux-5.10.config` (29 avril 2023) |

### Device Tree Linux
| Fichier local | Chemin d'origine |
|---|---|
| `dts-linux/imx6ul-opos6uldev.dts` | `buildroot/output/build/linux-5.10.167/arch/arm/boot/dts/` |
| `dts-linux/imx6ul-imx6ull-opos6uldev.dtsi` | idem |
| `dts-linux/imx6ul-opos6ul.dtsi` | idem |
| `dts-linux/imx6ul-imx6ull-opos6ul.dtsi` | idem |

### Device Tree U-Boot
| Fichier local | Chemin d'origine |
|---|---|
| `dts-uboot/imx6ul-opos6uldev.dts` | `buildroot/output/build/uboot-5f63b6d5ae18fb7dde758b425a19cd5bc4f9e6a2/arch/arm/dts/` |

---

## Statut des modifications (vs wiki PMX)

### Appliquees (menuconfig buildroot — `make menuconfig`)
- [x] F2FS : `BR2_PACKAGE_F2FS_TOOLS=y`
  `Target packages  ---> Filesystem and flash utilities  ---> [*] f2fs-tools`
- [x] RTL8821AU WiFi 5GHz : `BR2_PACKAGE_RTL8821AU=y`
  `Target packages  ---> Hardware handling  ---> [*] rtl8821au`
- [x] wireless-tools : `BR2_PACKAGE_WIRELESS_TOOLS=y`
  `Target packages  ---> Networking applications  ---> [*] wireless tools`
- [x] wpa_supplicant + CLI + passphrase : `BR2_PACKAGE_WPA_SUPPLICANT=y`
  `Target packages  ---> Networking applications  ---> [*] wpa_supplicant`
- [x] hostapd : `BR2_PACKAGE_HOSTAPD=y`
  `Target packages  ---> Networking applications  ---> [*] hostapd`
- [x] as_devices + cpp : `BR2_PACKAGE_AS_DEVICES=y`
  `Target packages  ---> Hardware handling  ---> [*] as_devices`

### Appliquees (linux-menuconfig kernel — `make linux-menuconfig`)
- [x] F2FS en module + XATTR, ACL, CHECK_FS (`CONFIG_F2FS_FS=m`)
  `File systems  ---> [M] F2FS filesystem support`
- [x] LED GPIO + triggers timer, heartbeat, backlight, gpio, default_on
  `Device Drivers  ---> LED Support  --->`
- [x] CPU freq scaling : ondemand, powersave, conservative, userspace, i.MX CPUfreq
  `CPU Power Management  ---> CPU Frequency scaling  --->`

### A appliquer (linux-menuconfig kernel) — Amelioration precision timers

Bench actuel (CONFIG_HZ=100, PREEMPT_VOLUNTARY) :
- Timer 100ms : erreur +24%, jitter max 177ms
- Timer 10ms  : erreur +129%, jitter max 159ms
- Timer 1ms   : erreur +164%, jitter max 152ms

**Modification 1 : Preemption**
```
General setup  --->
    Preemption Model  --->
        ( ) No Forced Preemption (Server)
        ( ) Voluntary Kernel Preemption (Desktop)    ← actuel
        (X) Preemptible Kernel (Low-Latency Desktop) ← a selectionner
```
Config : `CONFIG_PREEMPT_VOLUNTARY=y` → `CONFIG_PREEMPT=y`

**Modification 2 : Timer frequency**
```
Kernel Features  --->
    Timer frequency  --->
        ( ) 100 HZ     ← actuel
        (X) 1000 HZ    ← a selectionner
```
Config : `CONFIG_HZ=100` → `CONFIG_HZ=1000`

Impact attendu : jitter ~150ms → ~1-5ms (avec tweaks userspace SCHED_FIFO + mlockall)

### NON appliquees (device tree Linux + U-Boot)
- [ ] Desactivation LCD (backlight, pwm3, lcdif, pinctrl) - Linux ET U-Boot
- [ ] Reaffectation pins LCD en GPIO
- [ ] Ajout I2C3 sur LCD_DATA00/01
- [ ] Ajout I2C4 sur LCD_DATA02/03

---

## Inventaire des 3 cartes OPOS6UL

| | Carte 1 (robot) | Carte 2 | Carte 3 |
|---|---|---|---|
| **Role** | Dans le robot, avec dongle 5GHz | Avec LCD connecte | Spare |
| **MAC Ethernet** | `00:1e:ac:f8:9a:2a` | `00:1e:ac:f8:99:f0` | `00:1e:ac:f8:99:f1` |
| **IP** | `192.168.0.220` | `192.168.0.99` | `192.168.0.99` |
| **Server IP** | `192.168.0.218` | `192.168.0.121` | `192.168.0.121` |
| **Boot delay** | 3s | 5s | 5s |
| **Dongle USB RTL8821AU** | oui (wlan0, 5GHz) | oui (detecte) | non |
| **WiFi Broadcom SDIO** | actif (mmc1, 2.4GHz) | actif (mmc1, 2.4GHz) | actif (mmc1, 2.4GHz) |
| **LCD (lcdif)** | actif (800x480x18) | actif (800x480x18) | actif (800x480x18) |
| **I2C bus** | i2c-0, i2c-1 | i2c-0, i2c-1 | i2c-0, i2c-1 |
| **I2C devices (bus 0)** | 0x00, 0x2D | - | - |
| **I2C devices (bus 1)** | 0x00, 0x08, 0x20, 0x24 | - | - |
| **F2FS (mmcblk0p3)** | oui | ext4 | oui |
| **CPU freq** | performance, 900MHz | - | - |
| **Fichiers** | `bios_oposul1.txt`, `boot_oposul1.txt` | `bios_oposul2.txt`, `boot_oposul2.txt` | `bios_oposul3.txt`, `boot_oposul3.txt` |

---

## Ecran LCD : Santek ST0700-Adapt (L070O5-005)

### Specs

| Caracteristique | Valeur |
|---|---|
| Fabricant | Santek (San Technology) |
| Reference panneau | ST0700I5Y-RBSLW |
| Reference kit Armadeus | ST0700-Adapt |
| Taille | 7.0 pouces |
| Resolution | 800 x 480 (WVGA) |
| Profondeur couleur | 18 bits (RGB666) |
| Interface | RGB parallele (nappe TFT_J21 → X5 sur LCD_Adapt) |
| Backlight | PWM (pwm3) |
| **Tactile** | **Oui, resistif 4 fils (single touch)** |
| Controleur tactile | TSC integre au i.MX6ULL (bloc `2040000.tsc`) |
| Driver Linux | `mxsfb-drm` sur `/dev/fb0`, touchscreen sur `/dev/input/event0` |

### Commandes utiles

```bash
# Afficher une image sur l'ecran
fbv image.png

# Capture d'ecran framebuffer
fbgrab /tmp/screenshot.png

# Test framebuffer
fbtest

# Veille ecran
echo 1 > /sys/class/graphics/fb0/blank   # eteindre
echo 0 > /sys/class/graphics/fb0/blank   # rallumer

# Calibration tactile
export TSLIB_TSDEVICE=/dev/input/event0
ts_calibrate

# Test tactile
ts_test
```

### Logs de boot confirmant le fonctionnement

```
[drm] Initialized mxsfb-drm 1.0.0 for 21c8000.lcdif on minor 0
mxsfb 21c8000.lcdif: [drm] fb0: mxsfb-drmdrmfb frame buffer device
input: iMX6UL Touchscreen Controller as .../2040000.tsc/input0
```

### Test manuel

Programme de test : `brain/test/manual/test_screen.cpp` (cible CMake `test-screen`)

```bash
# Compilation ARM
cmake --preset arm-release && cmake --build build-arm-release --target test-screen

# Copier sur la carte
scp build-arm-release/test-screen root@<IP_CARTE>:/tmp/

# Test framebuffer (rouge, vert, bleu, blanc, mire 8 couleurs)
/tmp/test-screen

# Test framebuffer + tactile (dessine des croix jaunes au toucher)
/tmp/test-screen --touch
```

### Note importante

Si le LCD est desactive (option GPIO ci-dessous), l'affichage ET le tactile sont perdus.
Le LCD peut etre utile en dev pour afficher des infos de debug (position, etat IA, capteurs).

---

## Compilation WiFi RTL8821AU

Modification necessaire dans le fichier buildroot :
- Fichier : `buildroot/package/rtl8821au/rtl8821au.mk`
- Changer le hash du commit GitHub : `9f09c7627bda0ab5a577c1ad0337666ca2951132`
- Source : https://github.com/abperiasamy/rtl8812AU_8821AU_linux

---

## Commandes de compilation

```bash
# Config initiale
make opos6ul_defconfig           # ou opos6ul-preempt-rt_defconfig pour RT

# Menuconfig buildroot
make menuconfig

# Menuconfig kernel
make linux-menuconfig

# Compilation
make linux                       # recompiler le kernel seul
make                             # compilation complete

# Images generees dans :
# buildroot/output/images/
```

---

## Informations systeme (2024)

```
U-Boot SPL 2018.05 (Apr 29 2023)
CPU: Freescale i.MX6ULL rev1.1 900 MHz
DRAM: 256 MiB
Linux 5.10.167
Toolchain: arm-none-linux-gnueabihf-gcc 10.3.1 (A-profile 10.3-2021.07)
Buildroot: 2022.02 (armadeus-7.0-374-g8768da3e)
```

---

## OPTION : Suppression LCD pour GPIO supplementaires (NON appliquee)

La suppression du LCD est **optionnelle** et permet de recuperer jusqu'a 22 GPIO supplementaires
sur le bornier LCD de la carte OPOS6ULDev.

### Carte des pins du connecteur LCD (bornier J3)

**ATTENTION** : les pins `LCD_DATA18-23` sont partagees avec le **WiFi Broadcom SDIO** integre au SoM (usdhc2).
Ce WiFi SDIO est le module Broadcom (brcmfmac) soude sur le SoM OPOS6UL, ce n'est PAS le dongle USB RTL8821AU.
Ces 6 pins ne peuvent PAS etre utilisees comme GPIO si le WiFi integre est actif.

| Pin LCD | Fonction stock | Si LCD desactive | Conflit |
|---|---|---|---|
| `LCD_CLK` | LCDIF_CLK | **GPIO3_IO00** | - |
| `LCD_ENABLE` | LCDIF_ENABLE | **GPIO3_IO01** | - |
| `LCD_HSYNC` | LCDIF_HSYNC | **GPIO3_IO02** | - |
| `LCD_VSYNC` | LCDIF_VSYNC | **GPIO3_IO03** | - |
| `LCD_DATA00` | LCDIF_DATA00 | **I2C3_SDA** | - |
| `LCD_DATA01` | LCDIF_DATA01 | **I2C3_SCL** | - |
| `LCD_DATA02` | LCDIF_DATA02 | **I2C4_SDA** | - |
| `LCD_DATA03` | LCDIF_DATA03 | **I2C4_SCL** | - |
| `LCD_DATA04` | LCDIF_DATA04 | **GPIO3_IO09** | - |
| `LCD_DATA05` | LCDIF_DATA05 | **GPIO3_IO10** | - |
| `LCD_DATA06` | LCDIF_DATA06 | **GPIO3_IO11** | - |
| `LCD_DATA07` | LCDIF_DATA07 | **GPIO3_IO12** | - |
| `LCD_DATA08` | LCDIF_DATA08 | **GPIO3_IO13** | - |
| `LCD_DATA09` | LCDIF_DATA09 | **GPIO3_IO14** | - |
| `LCD_DATA10` | LCDIF_DATA10 | **GPIO3_IO15** | - |
| `LCD_DATA11` | LCDIF_DATA11 | **GPIO3_IO16** | - |
| `LCD_DATA12` | LCDIF_DATA12 | **GPIO3_IO17** | - |
| `LCD_DATA13` | LCDIF_DATA13 | **GPIO3_IO18** | - |
| `LCD_DATA14` | LCDIF_DATA14 | **GPIO3_IO19** | - |
| `LCD_DATA15` | LCDIF_DATA15 | **GPIO3_IO20** | - |
| `LCD_DATA16` | LCDIF_DATA16 | **GPIO3_IO21** | - |
| `LCD_DATA17` | LCDIF_DATA17 | **GPIO3_IO22** | - |
| `LCD_DATA18` | LCDIF_DATA18 | ~~GPIO~~ | **WiFi Broadcom SDIO CMD** (usdhc2) |
| `LCD_DATA19` | LCDIF_DATA19 | ~~GPIO~~ | **WiFi Broadcom SDIO CLK** (usdhc2) |
| `LCD_DATA20` | LCDIF_DATA20 | ~~GPIO~~ | **WiFi Broadcom SDIO DATA0** (usdhc2) |
| `LCD_DATA21` | LCDIF_DATA21 | ~~GPIO~~ | **WiFi Broadcom SDIO DATA1** (usdhc2) |
| `LCD_DATA22` | LCDIF_DATA22 | ~~GPIO~~ | **WiFi Broadcom SDIO DATA2** (usdhc2) |
| `LCD_DATA23` | LCDIF_DATA23 | ~~GPIO~~ | **WiFi Broadcom SDIO DATA3** (usdhc2) |
| `LCD_RESET` | GPIO3_IO04 | **LED User** (heartbeat) | - |

### Resume si LCD desactive

- **4 pins** → I2C3 + I2C4 (LCD_DATA00-03)
- **18 pins** → GPIO libres (LCD_DATA04-17 + CLK/ENABLE/HSYNC/VSYNC)
- **6 pins NON disponibles** → reservees WiFi Broadcom SDIO integre au SoM (LCD_DATA18-23)
- **1 pin** → LED User heartbeat (LCD_RESET)

### Modifications device tree a appliquer (depuis wiki PMX)

#### 1. Desactivation LCD dans linux-menuconfig

```
Device Drivers
  -> Graphics support
     [ ] Backlight & LCD device support  ----
     [ ] Bootup logo  ----
```

#### 2. Supprimer LCD et PWM (Linux DTS)

Fichier : `imx6ul-imx6ull-opos6uldev.dtsi`

Commenter les sections suivantes :

```c
/* --- COMMENTER lcd_backlight ---
	backlight: backlight {
		compatible = "pwm-backlight";
		pwms = <&pwm3 0 191000>;
		brightness-levels = <0 4 8 16 32 64 128 255>;
		default-brightness-level = <7>;
		power-supply = <&reg_5v>;
		status = "okay";
	};
*/
```

```c
/* --- COMMENTER panel ---
	panel: panel {
		compatible = "armadeus,st0700-adapt";
		power-supply = <&reg_3v3>;
		backlight = <&backlight>;
		port {
			panel_in: endpoint {
				remote-endpoint = <&lcdif_out>;
			};
		};
	};
*/
```

```c
/* --- COMMENTER &lcdif ---
&lcdif {
	pinctrl-names = "default";
	pinctrl-0 = <&pinctrl_lcdif>;
	status = "okay";
	port {
		lcdif_out: endpoint {
			remote-endpoint = <&panel_in>;
		};
	};
};
*/
```

```c
/* --- COMMENTER &pwm3 ---
&pwm3 {
	#pwm-cells = <2>;
	pinctrl-names = "default";
	pinctrl-0 = <&pinctrl_pwm3>;
	status = "okay";
};
*/
```

```c
/* --- COMMENTER pinctrl_lcdif ---
	pinctrl_lcdif: lcdifgrp {
		fsl,pins = <
			MX6UL_PAD_LCD_CLK__LCDIF_CLK        0x100b1
			MX6UL_PAD_LCD_ENABLE__LCDIF_ENABLE   0x100b1
			MX6UL_PAD_LCD_HSYNC__LCDIF_HSYNC     0x100b1
			MX6UL_PAD_LCD_VSYNC__LCDIF_VSYNC     0x100b1
			MX6UL_PAD_LCD_DATA00__LCDIF_DATA00   0x100b1
			...
			MX6UL_PAD_LCD_DATA17__LCDIF_DATA17   0x100b1
		>;
	};
*/
```

```c
/* --- COMMENTER pinctrl_pwm3 ---
	pinctrl_pwm3: pwm3grp {
		fsl,pins = <
			MX6UL_PAD_NAND_ALE__PWM3_OUT         0x1b0b0
		>;
	};
*/
```

#### 3. Ajouter I2C3 et I2C4 (Linux DTS)

Fichier : `imx6ul-opos6uldev.dts` (ou `imx6ul-imx6ull-opos6uldev.dtsi`)

```c
&i2c3 {
	pinctrl-names = "default";
	pinctrl-0 = <&pinctrl_i2c3>;
	clock_frequency = <400000>;
	status = "okay";
};

&i2c4 {
	pinctrl-names = "default";
	pinctrl-0 = <&pinctrl_i2c4>;
	clock_frequency = <400000>;
	status = "okay";
};
```

Pinctrl (dans `&iomuxc`) :

```c
	pinctrl_i2c3: i2c3grp {
		fsl,pins = <
			MX6UL_PAD_LCD_DATA00__I2C3_SDA	0x4001b8b0
			MX6UL_PAD_LCD_DATA01__I2C3_SCL	0x4001b8b0
		>;
	};

	pinctrl_i2c4: i2c4grp {
		fsl,pins = <
			MX6UL_PAD_LCD_DATA02__I2C4_SDA	0x4001b8b0
			MX6UL_PAD_LCD_DATA03__I2C4_SCL	0x4001b8b0
		>;
	};
```

#### 4. Reaffecter pins LCD en GPIO (Linux DTS)

Remplacer `pinctrl_gpios` par :

```c
	pinctrl_gpios: gpiosgrp {
		fsl,pins = <
			MX6UL_PAD_GPIO1_IO09__GPIO1_IO09	0x0b0b0
			MX6UL_PAD_UART3_RX_DATA__GPIO1_IO25	0x0b0b0
			MX6UL_PAD_UART3_TX_DATA__GPIO1_IO24	0x0b0b0
			MX6UL_PAD_NAND_RE_B__GPIO4_IO00		0x0b0b0
			MX6UL_PAD_GPIO1_IO08__GPIO1_IO08	0x0b0b0
			MX6UL_PAD_UART1_CTS_B__GPIO1_IO18	0x0b0b0
			MX6UL_PAD_UART1_RTS_B__GPIO1_IO19	0x0b0b0
			MX6UL_PAD_NAND_WE_B__GPIO4_IO01		0x0b0b0
			MX6UL_PAD_SNVS_TAMPER0__GPIO5_IO00	0x0b0b0
			MX6UL_PAD_SNVS_TAMPER2__GPIO5_IO02	0x0b0b0
			MX6UL_PAD_SNVS_TAMPER3__GPIO5_IO03	0x0b0b0
			MX6UL_PAD_SNVS_TAMPER4__GPIO5_IO04	0x0b0b0
			MX6UL_PAD_SNVS_TAMPER5__GPIO5_IO05	0x0b0b0
			MX6UL_PAD_SNVS_TAMPER6__GPIO5_IO06	0x0b0b0
			MX6UL_PAD_SNVS_TAMPER7__GPIO5_IO07	0x0b0b0
			MX6UL_PAD_SNVS_TAMPER8__GPIO5_IO08	0x0b0b0
			MX6UL_PAD_LCD_DATA04__GPIO3_IO09	0x0b0b0
			MX6UL_PAD_LCD_DATA05__GPIO3_IO10	0x0b0b0
			MX6UL_PAD_LCD_DATA06__GPIO3_IO11	0x0b0b0
			MX6UL_PAD_LCD_DATA07__GPIO3_IO12	0x0b0b0
			MX6UL_PAD_LCD_DATA08__GPIO3_IO13	0x0b0b0
			MX6UL_PAD_LCD_DATA09__GPIO3_IO14	0x0b0b0
			MX6UL_PAD_LCD_DATA10__GPIO3_IO15	0x0b0b0
			MX6UL_PAD_LCD_DATA11__GPIO3_IO16	0x0b0b0
			MX6UL_PAD_LCD_DATA12__GPIO3_IO17	0x0b0b0
			MX6UL_PAD_LCD_DATA13__GPIO3_IO18	0x0b0b0
			MX6UL_PAD_LCD_DATA14__GPIO3_IO19	0x0b0b0
			MX6UL_PAD_LCD_DATA15__GPIO3_IO20	0x0b0b0
			MX6UL_PAD_LCD_DATA16__GPIO3_IO21	0x0b0b0
			MX6UL_PAD_LCD_DATA17__GPIO3_IO22	0x0b0b0
			MX6UL_PAD_LCD_CLK__GPIO3_IO00		0x0b0b0
			MX6UL_PAD_LCD_ENABLE__GPIO3_IO01	0x0b0b0
			MX6UL_PAD_LCD_HSYNC__GPIO3_IO02		0x0b0b0
			MX6UL_PAD_LCD_VSYNC__GPIO3_IO03		0x0b0b0
		>;
	};
```

Note : les pins `LCD_DATA00-03` sont utilisees par I2C3/I2C4, les pins `LCD_DATA04-17` + `LCD_CLK/ENABLE/HSYNC/VSYNC` deviennent GPIO.

#### 5. Memes modifications dans U-Boot DTS

Fichier U-Boot : `imx6ul-opos6uldev.dts`

Appliquer les memes commentaires LCD/PWM et ajout I2C3/I2C4 que pour le DTS Linux.
