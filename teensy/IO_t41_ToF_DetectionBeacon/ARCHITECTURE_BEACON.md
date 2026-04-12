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

**Capteurs de collision / proximite** (optionnels, `SENSORS_VL_CLOSED_COLLISION_ACTIVATED`) :

8 capteurs VL53L1X supplementaires dedies a la detection rapprochee (obstacles proches du robot) :
- **4 Front** : bus I2C Wire (meme bus que les capteurs Back beacon)
- **4 Back** : bus I2C Wire1 (meme bus que les capteurs Front beacon)
- Pins XSHUT : Front = `{40, 39, 38, 37}`, Back = `{36, 35, 34, 33}`
- Adresses I2C : readdressees a `0x30`..`0x37` au demarrage
- Disposition physique : 2 capteurs par cote (BAS / HAUT), avant et arriere
  - Front : c1 (AV GAUCHE BAS), c2 (AV GAUCHE HAUT), c3 (AV DROIT BAS), c4 (AV DROIT HAUT)
  - Back : c5 (AR GAUCHE BAS), c6 (AR GAUCHE HAUT), c7 (AR DROIT BAS), c8 (AR DROIT HAUT)
- ROI : pleine matrice 16x16 SPADs (199 = centre optique)
- Mode : Short distance, meme timing budget que les capteurs beacon (15ms)
- Acquisition : integree dans les threads loopvl1/loopvl2 (zone 0 de chaque cycle), si le define est actif

Registres I2C (Registers) : distances en mm remontees dans `c1_mm`..`c8_mm` (Reg 11-27 absolus, soit offset 2-18 dans la struct Registers). Valeur 0 = pas de mesure. Lues par l'OPOS6UL via `BeaconSensors::getData()`.

Pour activer : decommenter `#define SENSORS_VL_CLOSED_COLLISION_ACTIVATED` dans `INO_ToF_DetectionBeacon.h` et brancher les 8 capteurs sur les pins XSHUT correspondants.

**Etat actuel** : define commente, capteurs non branches. A activer quand le hardware sera pret.

**Detection de mode video** : si plus de 26 zones detectent simultanement, passage en mode video (main proche couvrant tout le champ).

### Communication I2C Esclave — Registres VL53L1X

Les capteurs VL53L1X sont lus par la Teensy en I2C master (Wire/Wire1), puis les donnees traitees sont exposees via une interface **I2C esclave** (adresse `0x2D`, lib `i2c_register_slave`). Le cerveau OPOS6UL (master) lit ces registres pour obtenir les positions des robots adverses.

**Registres Settings** (9 bytes, voir section "Menu pre-match" pour le detail complet) :

| Registre | Type | Description |
|----------|------|-------------|
| 0 | int8 | `numOfBots` : nombre max d'adversaires a detecter (defaut: 3) |
| 1 | int8 | `ledLuminosity` : luminosite LED matrix (0..100, pas de 10) |
| 2 | uint8 | `matchPoints` : score de match a afficher |
| 3 | uint8 | `matchState` : 0=prepa, 1=match, 2=fini |
| 4 | uint8 | `lcdBacklight` : 0=off, 1=on |
| 5 | uint8 | `matchColor` : 0=bleu, 1=jaune |
| 6 | uint8 | `strategy` : 1..3 |
| 7 | uint8 | `testMode` : 0=aucun, 1..5 |
| 8 | uint8 | `advDiameter` : diametre adversaire en cm (defaut 40) |

**Registres Donnees** (lecture seule) :

**IMPORTANT** : les offsets I2C absolus = `sizeof(Settings)` + offset dans la struct `Registers`. Avec `Settings` = 9 bytes, **tous les offsets sont decales de +5** par rapport aux originaux (Settings = 4 bytes). Les anciens offsets sont notes entre parentheses.

| Registre | Type | Description |
|----------|------|-------------|
| 9 (ex-4) | uint8 | Flags (bit0 = new data, bit7 = alive) |
| 10 (ex-5) | uint8 | Nombre de robots detectes |
| 11-27 (ex-6-22) | int16 | Distances collision (c1 a c8) + reserve |
| 29-60 (ex-24-55) | int16/float | Positions (x, y, angle) pour 4 robots |
| 61-68 (ex-56-63) | int16 | Distances centre-a-centre (d1 a d4) |
| 69-132 (ex-64-127) | uint8/uint16 | Donnees brutes des zones (z1 a z4) |
| 133-140 (ex-128-135) | uint16 | Delta temps moyen de mesure par robot (t1-t4_us, en microsecondes depuis debut cycle) |
| 141-144 (ex-136-139) | uint32 | Numero de sequence (incremente chaque cycle) |

Le flag `new data` est remis a zero par l'ISR `on_read_isr` apres lecture par le master.

### Timing de mesure (t1-t4_us, seq)

Pour permettre a l'OPOS6UL de synchroniser la projection beacon→table avec la bonne
position du robot, chaque mesure est horodatee :

- `t_start_loop_us` : sauve `micros()` au debut de chaque cycle dans `tof_loop()`
- `zone_timestamp_us[zone]` : sauve `micros() - t_start_loop_us` apres chaque lecture ToF
- Pour chaque robot detecte, `calculPosition()` calcule la moyenne des timestamps des zones qui le composent
- Le numero de sequence (`seq`) est incremente a chaque cycle pour detecter les doublons

Exemple : si un adversaire est detecte sur les zones 2 et 3 (mesures a t=32ms et t=48ms
depuis le debut du cycle), `t1_us = 40000` (40ms en microsecondes).

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

## Menu pre-match (LCD tactile)

Le but de cette section est d'exposer a l'operateur un menu tactile en debut de match (phase preparation) pour configurer le robot (couleur, strategie, n° de match, tests materiel), et pendant le match d'afficher le score et l'etat. Les choix faits a l'ecran doivent etre lisibles par le cerveau OPOS6UL a sa cadence normale de polling I2C, **sans mecanisme de notification ni slave supplementaire**.

### Principe : un seul slave I2C partage (0x2D)

**Pas de nouveau slave I2C.** Le menu LCD ecrit directement dans la struct `Settings` du slave existant `TofSensors` (adresse `0x2D`), celui que l'OPOS6UL lit deja pour recuperer les positions ToF des robots adverses.

```
                    Teensy 4.1
    +------------------------------------------+
    |                                          |
    |   [LVGL callbacks]                       |
    |   touch --> ecriture directe dans        |
    |             Settings (champs LCD)        |
    |                     |                    |
    |                     v                    |
    |   +--------------------------------+     |
    |   |  Settings  +  Registers        |     |
    |   |  (slave I2C 0x2D)              |<----+------ I2C master
    |   +--------------------------------+     |       OPOS6UL
    |           ^                              |       lit a sa cadence
    |           |                              |       normale de polling
    |   TofSensors ecrit Registers             |
    |   (positions adverses, sequence, etc.)   |
    +------------------------------------------+
```

**Consequences** :
- Aucun nouveau driver cote OPOS6UL, la lecture deja existante recupere "au passage" les choix de l'operateur.
- Aucun handshake, aucune synchro active : c'est la **prochaine lecture** du master qui recupere les valeurs.
- Le code dormant `registerSlaveLCD` / `SettingsLCD` / `RegistersLCD` dans `LCDScreen.cpp` devient inutile (voir section dediee plus bas).

### Champs et proprietaires

Regle stricte : **un seul ecrivain par byte**. Puisque un acces `uint8_t`/`int8_t` est atomique sur Cortex-M7, cela suffit a eliminer toute race condition **sans mutex** sur ces champs. Les reads cote opposes peuvent etre faits a n'importe quel instant.

Les champs sont **regroupes par sens de communication** pour permettre au master I2C (OPOS6UL) de faire des **lectures/ecritures multi-bytes contigues** (block read, plus efficace qu'un acces byte par byte) sur chacun des deux blocs :

- **Bloc 1 (Reg 0-4)** : OPOS6UL -> Teensy. Le master ecrit ce bloc pour piloter l'affichage, la detection et signaler l'etat du match.
- **Bloc 2 (Reg 5-8)** : Teensy (LCD) -> OPOS6UL. Le master lit ce bloc pour recuperer la configuration operateur.

**Important** : `numOfBots` est une **consigne** (nombre max d'adversaires a detecter, ecrite par OPOS6UL) a ne pas confondre avec `nbDetectedBots` (mesure temps-reel du nombre d'adversaires effectivement detectes, deja presente dans `Registers` reg 5, ecrite par TofSensors). Les deux champs coexistent et sont tous les deux possedes par des ecrivains differents.

| Registre | Champ | Ecrivain | Lecteur | Role |
|----------|-------|----------|---------|------|
| **Bloc 1 : OPOS6UL -> Teensy** |||||
| 0 | `numOfBots` | OPOS6UL | Teensy (TofSensors) | Nombre max d'adversaires a detecter (1..4, defaut 3). **EXISTANT, INCHANGE** |
| 1 | `ledLuminosity` | OPOS6UL + LCD | LCD / LedPanels | Luminosite LED matrix (0..100, pas de 10). LCD ecrit en phase prepa, OPOS6UL peut reecrire en match. **EXISTANT, renomme (ex-`ledDisplay`)** |
| 2 | `matchPoints` | OPOS6UL | LCD / LedPanels | Score de match a afficher (texte defilant LED matrix et ecran LCD). **EXISTANT, renomme (ex-`tempNumber`)** |
| 3 | `matchState` | OPOS6UL | LCD | 0=preparation, 1=match en cours, 2=fini (ex-`reserved`) |
| 4 | `lcdBacklight` | OPOS6UL | LCD | 0=ecran eteint (eco batterie post-tirette), 1=allume |
| **Bloc 2 : Teensy (LCD) -> OPOS6UL** |||||
| 5 | `matchColor` | LCD (user) | OPOS6UL | Couleur equipe : 0=bleu, 1=jaune (bouton toggle) |
| 6 | `strategy` | LCD (user) | OPOS6UL | N° de strategie IA (1..3, boutons radio) |
| 7 | `testMode` | LCD (user) | OPOS6UL | Test materiel : 0=aucun, 1..5 (boutons radio, re-clic deselectionne) |
| 8 | `advDiameter` | LCD (user) | OPOS6UL | Diametre adversaire en cm (defaut 40, boutons -5/+5) |

**Offsets I2C preserves** : les registres 0, 1 et 2 gardent leur **offset et leur role fonctionnel**. Seuls les noms ont ete clarifies (`ledDisplay` -> `ledLuminosity`, `tempNumber` -> `matchPoints`). Le champ historique `reserved` (reg 3) accueille le nouveau `matchState`. `matchNumber` et `lcdCommitFlag` ont ete supprimes (non pertinents). `advDiameter` prend le reg 8.

**Exception atomicite** : `ledLuminosity` (reg 1) peut etre ecrit par **deux sources** (LCD en phase prepa, OPOS6UL en phase match). Pas de conflit en pratique car les phases sont sequentielles (dernier ecrivain gagne). Tous les autres champs respectent la regle "un seul ecrivain par byte".

### Struct Settings

```cpp
struct Settings {
    // === Bloc 1 : OPOS6UL -> Teensy (5 bytes) ===
    int8_t  numOfBots     = 3;   // Reg 0. Nb max adv a detecter (W: OPOS6UL)
    int8_t  ledLuminosity = 50;  // Reg 1. Luminosite LED matrix 0..100 (W: OPOS6UL + LCD en prepa)
    uint8_t matchPoints   = 0;   // Reg 2. Score LED matrix + LCD (W: OPOS6UL)
    uint8_t matchState    = 0;   // Reg 3. 0=prepa, 1=match, 2=fini (W: OPOS6UL)
    uint8_t lcdBacklight  = 1;   // Reg 4. 0=off, 1=on (W: OPOS6UL)

    // === Bloc 2 : Teensy (LCD) -> OPOS6UL (4 bytes) ===
    uint8_t matchColor    = 0;   // Reg 5. 0=bleu, 1=jaune (W: LCD)
    uint8_t strategy      = 0;   // Reg 6. 1..3 (W: LCD)
    uint8_t testMode      = 0;   // Reg 7. 0=aucun, 1..5 (W: LCD)
    uint8_t advDiameter   = 40;  // Reg 8. Diametre adversaire en cm (W: LCD)
};
// Total: 9 bytes, 2 blocs contigus
```

Note : `matchPoints` (reg 2) est partage entre deux consommateurs cote Teensy : la matrice LED (`LedPanels` qui affiche deja le score en texte defilant via `add_display_PointsNumber()`) et le LCD (affichage optionnel en phase match). Un seul ecrivain (OPOS6UL), deux lecteurs -> pas de conflit.

Note : `ledLuminosity` (reg 1) est **cable en temps reel** cote Teensy. `LedPanels.cpp` appelle `matrix->setBrightness(settings.ledLuminosity)` a chaque cycle d'affichage. Le changement via le tactile ou via l'OPOS6UL est donc visible immediatement sur la matrice LED.

### Concurrence et atomicite

- **Pas de mutex** necessaire sur les champs ci-dessus : un seul ecrivain par byte + atomicite des acces byte sur Cortex-M7.
- Le mutex existant `registers_new_data_lock` (dans `TofSensors.cpp`) continue de proteger les **champs multi-bytes** de `Registers` (positions x/y/a, sequence, timestamps, etc.) qui doivent rester coherents entre eux.
- Si plus tard un nouveau champ `Settings` devient multi-bytes (ex: un `uint16_t strategy_hash`), il faudra soit l'encadrer avec le meme mutex, soit le decomposer en acces byte.

### Cycle de vie LCD

**Point cle : ordre de mise sous tension Teensy / OPOS6UL indifferent.** La balise Teensy et le cerveau OPOS6UL peuvent etre allumes dans n'importe quel ordre. Des que la balise est sous tension, le menu est actif et l'operateur peut deja configurer les settings, meme si l'OPOS6UL n'est pas encore demarre. Quand l'OPOS6UL demarre (avant ou apres), il commence a lire les registres I2C a sa cadence normale et recupere les valeurs actuelles.

```
[boot Teensy] (avant OU apres OPOS6UL)
      |
      v
[phase preparation - LCD actif]
  Menu tactile operationnel des la mise sous tension Teensy.
  User modifie en direct : matchColor, strategy, testMode, advDiameter, ledLuminosity
  Chaque clic LVGL -> ecriture directe dans Settings (mode live)

  [boot OPOS6UL] peut arriver ici, avant ou apres la mise sous tension Teensy.
  A chaque lecture I2C de la balise, OPOS6UL recupere :
    - les positions ToF des adversaires (Registers, usage normal)
    - les Settings courants (numOfBots, matchColor, strategy, ...)
  L'init OPOS6UL (O_State_Init) relit les Settings a chaque cycle
  tant qu'il est en phase preparation : l'operateur peut donc continuer
  a modifier les settings jusqu'au dernier moment avant le depart du match.
      |
      v
[debut du match]
  OPOS6UL fige les settings finaux en interne et ecrit matchState = 1.
  Ce flag sert de signal multi-usage cote Teensy :
    - LCD desactive le traitement tactile (plus de modif possible)
    - LCD peut passer en ecran d'etat (ou ecran minimal / eteint)
    - LCD passe lcdBacklight = 0 si OPOS6UL l'a demande -> eco batterie
    - LVGL peut reduire sa frequence de rafraichissement -> eco CPU
      |
      v
[match en cours]
  OPOS6UL ecrit matchPoints au fur et a mesure (score temps reel).
  matchState reste a 1.
  Le LCD reste dans son mode "match" (eteint ou minimal).
  Le LCD peut etre rallume temporairement pour afficher le score.
      |
      v
[fin match]
  OPOS6UL ecrit matchState = 2 et le score final dans matchPoints.
  LCD affiche l'ecran final (score + etat).
```

Le `matchState` est donc le **point de bascule unique** entre la phase configurable (live) et la phase figee (eco batterie + CPU). Tant que `matchState == 0`, tout est modifiable ; des que `matchState >= 1`, le menu est en lecture seule de fait.

### Menu LCD implemente (etape 3)

Layout ecran 320x240 :

```
+--------------------------------------------------+
| LED [-] 50 [+]          Bots: 3  Pts: 0          |  luminosite + labels OPOS6UL
|        +========================+                 |
|        |        BLEU            |                 |  bouton toggle couleur
|        +========================+                 |
|                Strategie                          |
|     [    1    ] [    2    ] [    3    ]            |  boutons radio strategie
|                   Test                            |
|      [ T1 ] [ T2 ] [ T3 ] [ T4 ] [ T5 ]         |  boutons radio testMode
|                                                   |
| Diam. adv:  [ -5 ]  40 cm  [ +5 ]                |  diametre adversaire
+--------------------------------------------------+
```

Widgets et comportement :
- **Couleur** : bouton toggle 200x40, fond bleu/jaune, texte "BLEU"/"JAUNE" -> `settings.matchColor`
- **Strategie** : 3 boutons radio 90x40 (vert si selectionne, gris sinon) -> `settings.strategy` (1..3)
- **Test** : 5 boutons radio 54x28 (re-clic deselectionne -> testMode=0) -> `settings.testMode` (0..5)
- **Diam. adv** : boutons [-5]/[+5], affiche valeur en cm -> `settings.advDiameter` (defaut 40, range 5..250)
- **LED** : boutons [-]/[+], affiche valeur 0..100 par pas de 10 -> `settings.ledLuminosity` (applique en temps reel sur matrice LED)
- **Bots / Pts** : labels read-only, rafraichis a 5 Hz depuis `screen_loop()` (valeurs ecrites par OPOS6UL)
- Tous les choix sont en **mode live** : ecriture directe dans Settings a chaque clic, pas de bouton "Valider". La validation est la tirette de debut de match.

### Evolutions prevues

1. **Extinction backlight post-tirette** : `lcdBacklight = 0` apres depart du match pour economiser la batterie. L'OPOS6UL peut rallumer temporairement pour afficher un feedback.
2. **Persistence EEPROM** : sauvegarde des derniers choix operateur (`EEPROM.put/get` Teensy) pour les rappeler au reboot en phase de preparation (utile si reset accidentel juste avant le match).
3. **Ecran de monitoring match** : une fois `matchState = 1`, basculer l'UI LVGL sur un ecran qui affiche score, temps restant, nb robots detectes, etat ToF.
4. **testMode** : coder cote OPOS6UL les routines numerotees (1=test moteurs, 2=test capteurs ToF, 3=test AX12, 4=test servos, ...). Chaque valeur est une routine independante.

### Code dormant : slave 0x2F dans LCDScreen.cpp

`LCDScreen.cpp` contient actuellement un `registerSlaveLCD` (adresse `0x2F`) avec des structs `SettingsLCD` / `RegistersLCD` qui dupliquent celles de `TofSensors`. Ce code etait une **tentative d'architecture abandonnee** qui voulait un deuxieme slave I2C dedie au LCD. Il n'est pas actif (`listen(0x2F)` est commente).

Avec l'architecture actuelle (slave unique 0x2D partage), ce code devient inutile et sera **supprime a l'etape 7 ci-dessous**. Il est conserve temporairement pour reference.

### Etapes de migration

Checklist ordonnee pour suivre l'avancement. Cocher `[x]` au fur et a mesure. Chaque etape doit etre **compilable, flashable et testable** avant de passer a la suivante. Apres chaque etape : verifier que la detection ToF et la lecture OPOS6UL fonctionnent toujours.

#### Etape 1 : rename cosmetique (struct Settings, 0 nouveau champ) [FAIT]

Objectif : aligner les noms code/doc sans changer le comportement. **ABI preservee** (offsets inchanges).

- [x] `TofSensors.h` : rename `ledDisplay` -> `ledLuminosity` (reg 1)
- [x] `TofSensors.h` : rename `tempNumber` -> `matchPoints` (reg 2)
- [x] `LedPanels.cpp` : 3 occurrences `settings.tempNumber` -> `settings.matchPoints` (lignes 181, 466, 469)
- [x] Compiler + flasher + verifier : la detection ToF fonctionne toujours, OPOS6UL lit toujours les positions
- [x] Aucun changement cote OPOS6UL (meme offsets)

Note : les occurrences `ledDisplay` et `tempNumber` dans `LCDScreen.h` (struct `SettingsLCD` code dormant) sont volontairement laissees intactes, elles seront supprimees a l'etape 7 avec le reste du code dormant du slave 0x2F.

#### Etape 2 : extension struct Settings (nouveaux champs) [FAIT]

Objectif : ajouter les nouveaux champs avec leurs defauts.

- [x] `TofSensors.h` : supprimer `reserved`, ajouter `matchState`, `lcdBacklight`, `matchColor`, `strategy`, `testMode`, `advDiameter`
- [x] Initialiser les defauts dans la definition struct (defaults in-class)
- [x] `static_assert(sizeof(Settings) == 9)` pour verrouiller la taille I2C
- [x] `TofSensors.cpp` : simplifier init settings (defaults in-class)
- [x] Fix OPOS6UL : `BeaconSensors.hpp` struct Settings miroir + `SETTINGS_SIZE_BeaconSensors = 9`
- [x] Fix OPOS6UL : `BeaconSensors.cpp` offsets `getData()` via `DATA_BeaconSensors + offset`
- [x] Compiler + flasher Teensy + recompiler OPOS6UL + verifier

#### Etape 3 : menu LCD complet (ecriture LCD -> Settings) [FAIT]

Objectif : valider le pattern touch -> callback -> ecriture directe dans `settings`.

- [x] Bouton toggle couleur bleu/jaune -> `settings.matchColor`
- [x] 3 boutons radio strategie (1/2/3) -> `settings.strategy`
- [x] 5 boutons radio testMode (T1..T5, re-clic deselectionne) -> `settings.testMode`
- [x] Boutons [-5]/[+5] diametre adversaire -> `settings.advDiameter` (defaut 40 cm)
- [x] Boutons [-]/[+] luminosite LED -> `settings.ledLuminosity` (pas de 10, 0..100)
- [x] Labels read-only Bots + Pts (rafraichis a 5 Hz)
- [x] Luminosite cablee en temps reel (`matrix->setBrightness` dans LedPanels.cpp)
- [x] Test physique : bouton couleur, strategie, testMode, advDiameter, ledLuminosity fonctionnent
- [ ] Test I2C : lire les registres cote OPOS6UL et verifier les valeurs modifiees

#### Etape 4 : cote OPOS6UL, lire les nouveaux champs [ ]

Objectif : l'init OPOS6UL recupere les settings LCD au demarrage (en phase preparation).

- [ ] Driver `BeaconSensors` : ajouter une methode `readBeaconSettings()` qui lit les reg 5-8 (Bloc 2)
- [ ] `O_State_Init` : appeler `readBeaconSettings()` pour recuperer `matchColor`, `strategy`, `advDiameter`
- [ ] Mapping `matchColor` -> couleur attendue par `setPositionAndColor()` (bleu=?, jaune=?) -> verifier le mapping cote asserv
- [ ] Test : changer la couleur sur le LCD, redemarrer OPOS6UL, verifier que la couleur utilisee est bien celle du LCD

#### Etape 5 : matchState + economies phase match [ ]

Objectif : l'OPOS6UL signale le debut de match, le Teensy bascule en mode eco.

- [ ] `O_State_Init` (ou point de debut de match) : ecrire `settings.matchState = 1` via I2C
- [ ] `LCDScreen.cpp` : dans `screen_loop()`, lire `settings.matchState` et desactiver le traitement tactile + basculer sur ecran minimal quand `matchState >= 1`
- [ ] Tester la transition : menu actif -> debut match -> ecran fige
- [ ] Optionnel : reduire la frequence `lv_task_handler()` en mode match (eco CPU)

#### Etape 6 : affichage score pendant le match [ ]

Objectif : OPOS6UL envoie le score, le LCD et la matrice LED l'affichent.

- [ ] OPOS6UL : ecrire `settings.matchPoints` au fur et a mesure (deja utilise par `LedPanels` pour la matrice LED)
- [ ] `LCDScreen.cpp` : afficher `settings.matchPoints` sur un label LVGL quand `matchState >= 1`
- [ ] Test : envoyer differentes valeurs de score, verifier affichage LED matrix ET LCD
- [ ] A la fin du match, OPOS6UL ecrit `matchState = 2`, verifier ecran "fin"

#### Etape 7 : nettoyage code dormant [ ]

Objectif : supprimer le code mort `registerSlaveLCD` / `SettingsLCD` / `RegistersLCD` maintenant qu'on est sur que l'architecture "slave unique" fonctionne.

- [ ] `LCDScreen.cpp` : supprimer `settings_lcd`, `registers_lcd`, `registerSlaveLCD` (lignes ~57-62)
- [ ] `LCDScreen.h` : supprimer `struct SettingsLCD`, `struct RegistersLCD`
- [ ] `LCDScreen.cpp` : supprimer les commentaires `listen(0x2F)` et `on_read_isr_lcd`
- [ ] Compiler + flasher + regression test complet

#### Etape 8 : evolutions optionnelles (non bloquantes, a prioriser selon besoin) [ ]

- [ ] 8a : extinction backlight post-tirette (`lcdBacklight = 0`)
- [ ] 8b : ecran monitoring live pendant la phase preparation (affichage `nbDetectedBots`, distances, etc.)
- [ ] 8c : persistence EEPROM des derniers choix operateur (`EEPROM.put/get`)
- [ ] 8d : routines `testMode` cote OPOS6UL (1=moteurs, 2=capteurs, 3=AX12, ...)
- [ ] 8e : ecran de monitoring match (score, temps restant, robots detectes)
- [ ] 8f : activation capteurs de collision/proximite (decommenter `SENSORS_VL_CLOSED_COLLISION_ACTIVATED`, brancher les 8 VL53L1X)

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
