# Architecture logicielle - Balise Beacon de Detection Adverse

Balise de detection 360° des robots adverses par capteurs Time-of-Flight (ToF), embarquee sur **Teensy 4.1**.

## Vue d'ensemble

La balise est montee sur le robot principal et detecte les robots adverses sur 360° grace a **18 capteurs VL53L1X** (ToF laser) disposes en couronne. Elle calcule la position (angle, distance, coordonnees x/y) de chaque robot adverse detecte et expose ces donnees au cerveau principal (OPOS6UL) via un bus **I2C esclave**.

```
                        +-----------------------+
                        |     OPOS6UL (brain)   |
                        |     I2C Master         |
                        +-----------+-----------+
                                    |
                              I2C (0x2D)
                                    |
+-------------------------------------------------------------------+
|                         Teensy 4.1                                 |
|                                                                   |
|  +------------------+  +------------------+  +------------------+ |
|  |   TofSensors     |  |    LedPanels     |  |    LCDScreen     | |
|  |  (detection 360) |  | (affichage LED)  |  | (ecran ILI9341)  | |
|  +------------------+  +------------------+  +------------------+ |
|                                                                   |
|  +------------------+                                             |
|  |   HystFilter     |                                             |
|  | (filtre mesures) |                                             |
|  +------------------+                                             |
+-------------------------------------------------------------------+
        |                          |                    |
   18x VL53L1X              LED Matrix WS2812     ILI9341 + XPT2046
   (I2C Wire/Wire1)         (serial pin 8)        (SPI)
```

## Modules

### INO_ToF_DetectionBeacon (main)

Point d'entree Arduino (`setup()` / `loop()`).

- **setup()** : initialise les bus I2C (Wire et Wire1) en fast mode (1 MHz), les LEDs, les panneaux LED, les capteurs ToF et l'ecran LCD.
- **loop()** : boucle principale qui enchaine sequentiellement :
  1. Clignotement des LEDs de vie (builtin)
  2. Rafraichissement ecran LCD (`screen_loop`)
  3. Mise a jour affichage LED (`ledPanels_loop`)
  4. Acquisition et calcul ToF (`tof_loop`)

### TofSensors - Detection 360°

Module principal de detection. Gere 18 capteurs VL53L1X repartis en 2 groupes :
- **Front** (9 capteurs) : bus I2C Wire1 (pins 17 SDA1 / 16 SCL1)
- **Back** (9 capteurs) : bus I2C Wire (pins 18 SDA / 19 SCL)

Chaque capteur utilise des **zones SPAD configurables** pour augmenter la resolution angulaire :
- 16 SPADs de largeur totale
- 4 SPADs par zone, decales de 4
- **4 zones par capteur** → 18 x 4 = **72 zones** sur 360°
- Resolution angulaire : **~5° par zone**

**Architecture multi-thread** (TeensyThreads) :
- **Thread loopvl1** : acquisition des 9 capteurs Front (+ capteurs collision si actives)
- **Thread loopvl2** : acquisition des 9 capteurs Back (+ capteurs collision si actives)
- **Loop principale** : synchronisation, filtrage, calcul de positions

**Pipeline de traitement** :

```
Acquisition parallele (2 threads)
        |
        v
Synchronisation (attente fin des 2 threads)
        |
        v
Filtrage des donnees brutes
  - Correction des zeros (remplacement par valeur precedente)
  - Filtrage des faux positifs via SigPerSPAD et NumSPADs
  - Calcul greenHandDistance (detection main proche)
        |
        v
Calcul de position (calculPosition)
  - Identification des balises (groupes de zones contigues)
  - Calcul de l'angle moyen par balise
  - Calcul de la distance moyenne par balise
  - Conversion en coordonnees cartesiennes (x, y)
  - Gestion du chevauchement circulaire (zone 71 → zone 0)
        |
        v
Ecriture registres I2C (protegee par mutex)
```

**Capteurs de collision** (optionnels, `SENSORS_VL_CLOSED_COLLISION_ACTIVATED`) :
- 4 capteurs supplementaires Front + 4 Back pour detection rapprochee

**Detection de mode video** : si plus de 26 zones detectent simultanement, passage en mode video (main proche couvrant tout le champ).

### Communication I2C Esclave — Registres VL53L1X

Les capteurs VL53L1X sont lus par la Teensy en I2C master (Wire/Wire1), puis les donnees traitees sont exposees via une interface **I2C esclave** (adresse `0x2D`, lib `i2c_register_slave`). Le cerveau OPOS6UL (master) lit ces registres pour obtenir les positions des robots adverses.

**Registres Settings** (lecture/ecriture par le master) :

| Registre | Type | Description |
|----------|------|-------------|
| 0 | int8 | Nombre de robots a detecter (defaut: 3) |
| 1 | int8 | Mode affichage LED (0=OFF, 50=moitie, 100=plein) |
| 2 | uint8 | Points a afficher |
| 3 | int8 | Reserve |

**Registres Donnees** (lecture seule) :

| Registre | Type | Description |
|----------|------|-------------|
| 4 | uint8 | Flags (bit0 = new data, bit7 = alive) |
| 5 | uint8 | Nombre de robots detectes |
| 6-22 | int16 | Distances collision (c1 a c8) + reserve |
| 24-55 | int16/float | Positions (x, y, angle) pour 4 robots |
| 56-63 | int16 | Distances centre-a-centre (d1 a d4) |
| 64-127 | uint8/uint16 | Donnees brutes des zones (z1 a z4) |

Le flag `new data` est remis a zero par l'ISR `on_read_isr` apres lecture par le master.

### HystFilter - Filtre a hysteresis

Classe de filtrage a hysteresis (type trigger de Schmitt logiciel). Stabilise les valeurs de sortie en ajoutant une "adherence" aux transitions entre niveaux.

- Parametres : nombre de valeurs d'entree, nombre de niveaux de sortie, marge d'hysteresis
- Evite les oscillations rapides entre deux etats adjacents (anti-flicker)

### LedPanels - Affichage LED matriciel

Affichage sur une **matrice LED WS2812** (NeoPixel) :
- Configuration : 9 matrices 4x4 sur 2 rangees = **36x8 pixels** (288 LEDs)
- Bus : WS2812Serial sur pin 8
- Pilotee par FastLED + Adafruit GFX + FastLED_NeoMatrix

Fonctionnalites d'affichage :
- Visualisation en temps reel des distances mesurees par les VL53L1X
- Indicateur de capteurs hors-ligne (`DEBUG_VL_SUR_LEDMATRIX`)
- Affichage de texte defilant, bitmaps, score, animations
- Thread dedie pour l'affichage (`thread_display`)

### LCDScreen - Ecran tactile ILI9341

Ecran TFT 320x240 avec tactile XPT2046, pilote par :
- **ILI9341_T4** : driver optimise Teensy avec framebuffer interne (150 KB en DMAMEM), double diff buffer (2x8 KB), V-Sync
- **LVGL** : framework UI (grille, boutons, labels)
- Bus : SPI (pins 9-13), tactile sur pin CS 7

Possede son propre bus I2C esclave (adresse `0x2F`, actuellement desactive) avec registres `SettingsLCD` / `RegistersLCD`.

## Configuration hardware

### Pins I2C capteurs ToF

Les capteurs sont adresses individuellement grace a des **pins XSHUT** (shutdown) :
- Front (Wire1) : pins 23, 22, 21, 20, 0, 1, 2, 3, 4
- Back (Wire) : pins 32, 31, 30, 29, 28, 27, 26, 6, 5

Au demarrage, chaque capteur est active un par un et readdresse de `0x15` a `0x26`.

### Configuration des capteurs VL53L1X

- Mode : **Short distance** (plus rapide et precis a courte portee)
- Timing budget : **15 ms**
- Periode inter-mesures : **16 ms**
- ROI : configurable par zone (4 SPADs de large x 8 SPADs de haut)

## Concurrence et synchronisation

- **2 threads** d'acquisition (loopvl1, loopvl2) via TeensyThreads
- **Mutex** :
  - `registers_new_data_lock` : protege l'ecriture des registres I2C
  - `filteredResultWorkingCopy_mutex` : protege la copie de travail des donnees filtrees
  - `l1_mutex`, `l2_mutex` : synchronisation des threads
- **Variables volatiles** : `shared_endloop1`, `shared_endloop2` pour la synchronisation loop/threads
- Les threads utilisent `threads.yield()` pour le partage cooperatif du CPU

## Dependances

| Librairie | Usage |
|-----------|-------|
| SparkFun_VL53L1X | Driver capteurs ToF |
| TeensyThreads | Multi-threading sur Teensy |
| i2c_register_slave | Esclave I2C avec registres |
| i2c_driver_wire | Driver I2C optimise Teensy |
| WS2812Serial | LEDs NeoPixel via serial |
| FastLED / FastLED_NeoMatrix | Controle matrice LED |
| Adafruit GFX | Primitives graphiques |
| ILI9341_T4 | Driver ecran TFT optimise |
| LVGL | Framework UI pour ecran |

## Build

Projet PlatformIO independant :
- Board : `teensy41`
- Framework : Arduino
- Upload : `teensy-cli`
- Moniteur serie : 921600 baud
