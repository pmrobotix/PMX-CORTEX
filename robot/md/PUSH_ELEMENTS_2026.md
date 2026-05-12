# PUSH_ELEMENTS_2026 — Manipulation de poussee des elements de jeu

Manipulation generique permettant au robot de pousser hors de la zone devant
lui un nombre variable d'elements (1, 2 ou 3) selon la disposition reelle des
elements relevee par les juges et configuree depuis la balise. La distance
d'avance est calculee a partir de la couleur du robot (BLEU/JAUNE) et de la
configuration choisie sur l'ecran tactile beacon pour la zone consideree.

## Vue d'ensemble

```
+-----------+     LCD tactile        +-------------------+
|  Beacon   |  selection 6 configs   |  brain (OPOS6UL)  |
|  Teensy   | ---------------------> |  Robot::pickupPN_ |
+-----------+    I2C MatchSettings   +-------------------+
                                              |
                                              v
                                  push_elements_zone(idx, "P14")
                                              |
                                              v
                                 nav.line(+dist) puis nav.line(-D_RETREAT)
```

La config (idx 0..5) est saisie **avant la tirette** sur l'ecran tactile
balise, propagee en I2C, et stockee cote brain dans
[Robot.hpp:180-187](../src/common/Robot.hpp#L180-L187). Pendant le match, la
manipulation lit cette config pour decider de la distance d'avance.

## Lien avec les briques existantes


| Composant               | Role                               | Reference                                                                                       |
| ------------------------- | ------------------------------------ | ------------------------------------------------------------------------------------------------- |
| Menu LCD tactile balise | Saisie 6 configs par zone          | [LCDScreen.cpp create_pickup_config](../../teensy/IO_t41_ToF_DetectionBeacon/src/LCDScreen.cpp) |
| Spec UI beacon          | Documentation complete             | [MATCH_CONFIG_UI.md](../../teensy/IO_t41_ToF_DetectionBeacon/MATCH_CONFIG_UI.md)                |
| Transport I2C           | `MatchSettingsData::pickup_P*`     | [ASensorsDriver.hpp:82-110](../src/common/interface/ASensorsDriver.hpp#L82-L110)                |
| Adoption brain          | `MenuBeaconLCDTouch::pollInputs`   | [MenuBeaconLCDTouch.cpp](../src/common/menu/MenuBeaconLCDTouch.cpp)                             |
| Stockage brain          | `Robot::pickup_P{N}_` + getters    | [Robot.hpp:180-187, 621-637](../src/common/Robot.hpp#L180-L187)                                 |
| Manipulation            | `push_elements_zone` + 16 wrappers forward + 16 wrappers backward | [StrategyActions2026.cpp](../src/bot-opos6ul/StrategyActions2026.cpp)                           |
| Viz SVG                 | `drawConfigAtPose` (auto en prod et en test) | [StrategyActions2026.cpp](../src/bot-opos6ul/StrategyActions2026.cpp) (namespace anonyme)       |

## Numerotation et ordre des configurations

La balise expose 6 configurations par zone, indexees 0..5, dans cet ordre :


| idx | sequence | description                                 |
| ----- | ---------- | --------------------------------------------- |
| 0   | BBYY     | 2 bleus puis 2 jaunes (sens lecture balise) |
| 1   | YYBB     | 2 jaunes puis 2 bleus                       |
| 2   | BYYB     | bleu, jaune, jaune, bleu                    |
| 3   | YBBY     | jaune, bleu, bleu, jaune                    |
| 4   | BYBY     | alternance commencant par bleu              |
| 5   | YBYB     | alternance commencant par jaune             |

**Convention couleur** : la strategie JSON est ecrite EN BLEU (cf commit
3a211c52, JSON = coords BLUE, miroir Asserv applique en interne). En YELLOW,
`push_elements_zone` applique automatiquement 2 transformations equivalentes
au miroir physique :

1. **Suffixe horizontal `_D` <-> `_G`** : le miroir Asserv sur X fait que le
   robot arrive du cote oppose pour les zones horizontales (P3, P4, P13, P14).
   Verticales (P1, P2, P11, P12) non affectees (miroir uniquement sur X).
2. **Index pickup `idx -> SWAP_COLOR_IDX[idx]`** : les paires de configurations
   (0,1) (2,3) (4,5) sont symetriques par swap couleur (ex BBYY <-> YYBB).
   En YELLOW, lire la valeur a l'index symetrique pousse SA propre majorite
   exactement comme le BLEU pousse la sienne, **avec UNE SEULE table de
   distances** au lieu de 2.

```cpp
static constexpr uint8_t SWAP_COLOR_IDX[6] = { 1, 0, 3, 2, 5, 4 };
// 0 BBYY <-> 1 YYBB | 2 BYYB <-> 3 YBBY | 4 BYBY <-> 5 YBYB
```

## Tables de distance

### Constantes reglables (en haut de [StrategyActions2026.cpp](../src/bot-opos6ul/StrategyActions2026.cpp))


| Constante          | Valeur (mm) | Usage                                                      |
| -------------------- | ------------- | ------------------------------------------------------------ |
| `D_RETREAT`        | 200         | recul de degagement apres la pousse                        |
| `D_BASE`           | 400         | distance de base de la pousse (commune a toutes les zones) |
| `distDirecte[idx]` | -125 a +175 | ajustement signe par config (lu par wrappers`_H`/`_G`)     |
| `distInverse[idx]` | -125 a +175 | ajustement signe par config (lu par wrappers`_B`/`_D`)     |
| `kZoneOffset_P4`   | -50         | offset specifique P4  (D_BASE effectif: 350 mm)            |
| `kZoneOffset_P14`  | -50         | offset specifique P14 (D_BASE effectif: 350 mm)            |

Formule finale : `dist = D_BASE + distXxx[idx] + zoneOffset(zone)`.

### Visuel des configurations

![6 configurations idx balise et distances associees](

![](assets/20260510_212026_image.png)

)

(Note : "B" = Bleu, "J" = Jaune dans l'image ; "B" = Bleu, "Y" = Yellow dans
le code C++ — meme convention, juste une lettre differente.)

### Tables compilees (idx balise 0..5)


| idx balise | sequence | distDirecte | distInverse     |
| ------------ | ---------- | ------------- | ----------------- |
| 0          | BBYY     | `D0` (125)  | `D0_INV` (-125) |
| 1          | YYBB     | `D4` (-125) | `D4_INV` (125)  |
| 2          | BYYB     | `D2` (175)  | `D2_INV` (175)  |
| 3          | YBBY     | `D1` (75)   | `D1_INV` (75)   |
| 4          | BYBY     | `D1` (75)   | `D1_INV` (-75)  |
| 5          | YBYB     | `D3` (-75)  | `D3_INV` (75)   |

Code C++ correspondant :

```cpp
// Distance de base commune (mm). P4/P14 ont un offset de -50 mm (effectif 350).
constexpr float D_BASE = 400.0f;

// Ajustements signes par configuration (mm) - PLACEHOLDER A CALIBRER.
static constexpr float distDirecte[6] = {  125, -125,  175,  75,  75, -75 };
static constexpr float distInverse[6] = { -125,  125,  175,  75, -75,  75 };

const float distBase = sensInverse ? distInverse[pickupIdx] : distDirecte[pickupIdx];
const float dist     = D_BASE + distBase + zoneOffsetFor(zoneName);
```

**Calibration** : `D_BASE` est la distance de base de la pousse (commune a
toutes les zones, sauf P4/P14 qui ont un offset de -50mm). Les tables
`distDirecte` / `distInverse` apportent un ajustement signe par configuration
(valeurs derivees du visuel des 6 placements). Garde-fou : si
`D_BASE + distBase + offset <= 0`, la manip log une erreur et retourne `false`.
Avec D_BASE=400 et amplitudes max ±175 (offsets inclus), on a toujours
`dist >= 175 mm`.

## Sequence d'execution

La fonction `push_elements_zone(pickupIdx, zoneName, sensInverse, backward=false)` enchaine :

1. Lecture `pickupIdx` (0..5).
2. Validation `pickupIdx <= 5` (sinon abort, `return false`).
3. Choix `dist` dans `distDirecte` ou `distInverse` selon `sensInverse`.
4. Si `backward=true` : `effectiveDist = dist + kBackwardPushAdjustmentMm`
   (kBackwardPushAdjustmentMm = -25 mm, calibration empirique compensant
   le sous-decalage rear-first). Sinon `effectiveDist = dist`.
5. **`setMaxSpeed(true, 20)`** : vitesse reduite pour avoir du couple et ne
   pas balayer les elements en bord de zone.
6. Configuration capteurs :
   - **Front center actif** : on veut detecter la collision sur l'element
     pousse pour que le retry asserv reagisse correctement.
   - Front lateral + back : ignores pendant la sequence.
7. Viz SVG (auto) : `drawConfigAtPose(initX, initY, initT, idx, yellow,
   extraForward=0, dashed=false, backward)` → 4 rects pleins a la pose initiale
   (cote face de pousse : avant si forward, arriere si backward).
8. `nav.line(signDir * effectiveDist, policyPush)` ou `signDir = backward ? -1 : +1`
   (avance pour pousser : front-first en forward, rear-first en backward).
   - Si `ts != TRAJ_FINISHED` : reset emergency, log erreur, `return false`.
9. Viz SVG (auto) : `drawConfigAtPose(..., extraForward=effectiveDist, dashed=true, backward)`
   → 4 rects pointilles a la pose finale (initiale + effectiveDist dans le sens
   de pousse).
10. `nav.line(-signDir * D_RETREAT, policyPush)` : recul de degagement (sens
    oppose a la pousse → forward recule, backward avance).
    - Echec non-bloquant : la pousse a deja eu lieu, on logue et on continue.
11. `return true`.

**RetryPolicy utilisee** : `RetryPolicy::standard()` — equivaut a
`{ 2000000, 2, 2, 0, 0, false }`, soit 2 retries sur obstacle et 2 retries sur
collision avec 2s d'attente entre tentatives. Le retry est limite parce qu'en
push, l'objet est forcement en contact ; au-dela de 2 retries, c'est un vrai
blocage.

## Wrappers par zone (16 actions)

Pour chaque zone, **2 wrappers** correspondant aux 2 cotes d'arrivee possibles
du robot. Le suffixe `_B/_H/_D/_G` designe le **cote d'arrivee** du robot
dans la zone (= d'ou il vient juste avant la pousse) :


| Suffixe | Cote d'arrivee                | Direction d'avance      | Sens (vs lecture balise) | Table         |
| --------- | ------------------------------- | ------------------------- | -------------------------- | --------------- |
| `_B`    | par le BAS  (Y=0, public)     | vers le HAUT (Y+)       | INVERSE                  | `distInverse` |
| `_H`    | par le HAUT (Y+, fond table)  | vers le BAS  (Y=0)      | DIRECTE                  | `distDirecte` |
| `_D`    | par la DROITE (X=3000, jaune) | vers la GAUCHE (X=0)    | INVERSE                  | `distInverse` |
| `_G`    | par la GAUCHE (X=0, bleu)     | vers la DROITE (X=3000) | DIRECTE                  | `distDirecte` |

**Pourquoi cette correspondance** : la lecture balise va haut→bas (vertical)
ou gauche→droite (horizontal). Le sens DIRECTE = robot avance dans le sens
de lecture, donc arrive par le cote DEBUT (haut ou gauche) → wrappers `_H`
et `_G`. Les wrappers `_B` / `_D` (arrive par le cote FIN) lisent la sequence
"a l'envers" mentalement → sens INVERSE → distance differente (table separee).

Liste des 16 wrappers :


| Verticales (P1, P2, P11, P12)                | Horizontales (P3, P4, P13, P14)              |
| ---------------------------------------------- | ---------------------------------------------- |
| `push_elements_P1_B`,  `push_elements_P1_H`  | `push_elements_P3_D`,  `push_elements_P3_G`  |
| `push_elements_P2_B`,  `push_elements_P2_H`  | `push_elements_P4_D`,  `push_elements_P4_G`  |
| `push_elements_P11_B`, `push_elements_P11_H` | `push_elements_P13_D`, `push_elements_P13_G` |
| `push_elements_P12_B`, `push_elements_P12_H` | `push_elements_P14_D`, `push_elements_P14_G` |

## Wrappers backward (16 actions rear-first) {#backward}

Pour chaque wrapper forward, il existe une variante **rear-first** prefixee
`push_back_elements_*`. Meme mapping zone × suffixe d'arrivee, meme calcul
de `dist` (tables identiques), mais le robot pousse **avec sa face arriere**
au lieu de sa face avant.

Differences techniques avec la variante forward :

| Aspect                          | Forward (`push_elements_*`)         | Backward (`push_back_elements_*`)         |
| --------------------------------- | ------------------------------------- | ------------------------------------------- |
| Face de pousse                  | Avant (`kRobotFrontOffset = 115mm`) | Arriere (`kRobotRearOffset = 130mm`)      |
| Signe `nav.line(dist)`          | `+dist`                             | `-dist` (rear-first)                      |
| Signe `nav.line(D_RETREAT)`     | `-D_RETREAT` (recul)                | `+D_RETREAT` (avance pour se degager)     |
| Ajustement empirique sur `dist` | aucun                               | `-25mm` (`kBackwardPushAdjustmentMm`)     |
| Viz SVG (4 rects)               | en avant du robot (kRobotFrontOffset) | en arriere du robot (kRobotRearOffset)    |
| `faceTo` avant la manip         | face avant vers cubes               | face arriere vers cubes (= cap a 180° de la pousse) |

**Quand utiliser le backward** : pour enchainer 2 zones sans demi-tour. Si le
robot termine une instruction en se dirigeant vers Y- (front pointe vers Y-)
et que la prochaine pousse doit se faire vers Y+, un wrapper backward evite
le `faceTo` + rotation 180°. Gain de temps en match.

**Important** : le `faceTo` precedent doit orienter la **face arriere** du
robot vers les cubes (et non la face avant). C'est la responsabilite de la
strategie, identique au cas forward (le wrapper ne fait pas de rotation).

Liste des 16 wrappers backward :

| Verticales (P1, P2, P11, P12)                          | Horizontales (P3, P4, P13, P14)                        |
| -------------------------------------------------------- | -------------------------------------------------------- |
| `push_back_elements_P1_B`,  `push_back_elements_P1_H`  | `push_back_elements_P3_D`,  `push_back_elements_P3_G`  |
| `push_back_elements_P2_B`,  `push_back_elements_P2_H`  | `push_back_elements_P4_D`,  `push_back_elements_P4_G`  |
| `push_back_elements_P11_B`, `push_back_elements_P11_H` | `push_back_elements_P13_D`, `push_back_elements_P13_G` |
| `push_back_elements_P12_B`, `push_back_elements_P12_H` | `push_back_elements_P14_D`, `push_back_elements_P14_G` |

## Utilisation dans le JSON de strategie

```json
{
  "type": "MANIPULATION",
  "action_id": "push_elements_P1_B",
  "timeout": 5000,
  "desc": "Pousse les elements en P1 (robot arrive par le bas)"
}
```

**Astuce** pour choisir le wrapper : pense en termes de **trajectoire
physique**. Si la task `MOVEMENT/FACE_TO` precedente oriente le robot vers
Y+, c'est qu'il arrive du sud / par le bas → wrapper `_B`. Vers Y=0 → `_H`.
Vers X=3000 → `_G`. Vers X=0 → `_D`.

## Convention d'orientation (recap)

Mapping ecran balise → table physique
([MATCH_CONFIG_UI.md](../../teensy/IO_t41_ToF_DetectionBeacon/MATCH_CONFIG_UI.md)) :


| Sur l'ecran | Sur la table                      |
| ------------- | ----------------------------------- |
| HAUT        | ARRIERE (Y+ grand, fond de table) |
| BAS         | AVANT   (Y=0, cote public)        |
| GAUCHE      | X=0     (cote bleu, NID BLEU)     |
| DROITE      | X=3000  (cote jaune, NID JAUNE)   |

Sens de lecture des sequences balise :


| Zones            | Orientation | Lecture sequence (1er → 4eme caractere)             |
| ------------------ | ------------- | ------------------------------------------------------ |
| P1, P2, P11, P12 | Verticale   | **HAUT → BAS** ecran = ARRIERE → AVANT (Y+ → Y=0) |
| P3, P4, P13, P14 | Horizontale | **GAUCHE → DROITE** ecran = X=0 → X=3000           |

**Transformation auto en YELLOW** (cf section "Convention couleur" plus haut) :

- Suffixe horizontal `_D` <-> `_G` (verticales `_B`/`_H` inchangees)
- Index pickup `idx -> SWAP_COLOR_IDX[idx]`

Une seule paire de tables (`distDirecte`/`distInverse`) suffit pour les 2
couleurs grace a cette symetrie.

### Pourquoi 2 tables (directe vs inverse)

Selon le cote d'arrivee du robot, les elements adverses sont rencontres en
TETE ou en QUEUE de la sequence — ce qui change radicalement la distance a
parcourir pour les pousser hors de la zone. Schema (zone verticale, idx=0
BBYY) :

```
WRAPPER _H (arrive du haut = sens DIRECTE)        WRAPPER _B (arrive du bas = sens INVERSE)
                                                                          
   ARRIERE [B] <- robot ici, arrive                  ARRIERE [B]
           [B]    et pousse vers le bas                      [B]
           [Y]    rencontre Y en QUEUE                       [Y]
   AVANT   [Y]    -> dist courte (250mm)             AVANT   [Y] <- robot ici, arrive
                                                                  rencontre Y en TETE
                                                                  -> dist longue (450mm)
```

Donc :

- `_H` ou `_G` (arrive du sens "lecture") → **sens DIRECTE** → `distDirecte` (court)
- `_B` ou `_D` (arrive du sens "anti-lecture") → **sens INVERSE** → `distInverse` (long)

## Exceptions par zone (offsets specifiques)

Toutes les zones n'ont pas la meme **distance de prise/depose** physique sur
la table. En particulier, **P4 et P14** ont une distance differente des
autres zones horizontales (P3, P13). On applique un offset specifique a ces
2 zones, ajoute a la distance lue dans `distDirecte` ou `distInverse`.


| Constante         | Valeur (mm)       | Usage                      |
| ------------------- | ------------------- | ---------------------------- |
| `kZoneOffset_P4`  | 0.0 (placeholder) | offset specifique pour P4  |
| `kZoneOffset_P14` | 0.0 (placeholder) | offset specifique pour P14 |

L'offset s'ajoute apres le lookup dans la table :

```cpp
const float distBase = sensInverse ? distInverse[idx] : distDirecte[idx];
const float dist     = distBase + zoneOffset(zoneName);  // helper interne
// zoneOffset retourne 0 sauf pour zoneName == "P4" ou "P14"
```

**Les offsets peuvent etre negatifs** si la zone de prise/depose est plus
proche que la moyenne (P4 et P14 ont moins de distance a parcourir d'apres
les mesures terrain). Exemple : `kZoneOffset_P4 = -50.0f` reduit la distance
de pousse de 50mm pour les wrappers `push_elements_P4_D` et `push_elements_P4_G`.

**Garde-fou** : si `distBase + offset <= 0`, la manip log une erreur et
retourne `false` (le runner skip l'instruction et continue). Permet
d'eviter qu'un offset trop negatif ne fasse reculer le robot a la place de
le faire avancer.

Pour generaliser a plus de 2 zones, etendre `zoneOffset()` en ajoutant des
`if` ou en passant a une table `{const char* zone, float offset}[]`.

## Telemetrie de duree (timeout JSON)

Le champ `"timeout"` est utilise par le runner uniquement pour la telemetrie :
le runner mesure la duree reelle de chaque MANIPULATION et logue

- `info` avec la duree mesuree dans tous les cas ;
- `warn` supplementaire si la duree depasse le `timeout_ms` declare.

**Aucun abort automatique** : la manipulation est synchrone et bloquante (cf
`nav.line` qui itere sur RetryPolicy). Un vrai watchdog necessiterait soit un
auto-check coopere par chaque manip entre etapes, soit un thread + abort
asserv via `resetEmergencyOnTraj`. A faire dans une iteration ulterieure.

Pour push_elements : valeur typique attendue 2000..4000 ms (avance + recul a
20-40% PWM). Mettre `"timeout": 5000` puis ajuster selon les mesures reelles.

Avant l'appel, la strategie doit avoir positionne le robot face a la zone
visee (orientation correcte pour que `nav.line(+dist)` pousse dans la bonne
direction). Le push lui-meme ne fait pas de rotation, il delegue ce
positionnement aux tasks `MOVEMENT/FACE_TO` qui le precedent.

## Ce que la manip ne fait pas

- **Pas de navigation vers la zone** : le robot doit deja etre face aux
  elements quand la manip demarre. Contrairement a `push_prise_bas` qui
  embarque un `goToZone`, ici la zone est juste une cle d'index dans la
  config beacon — pas une cible spatiale.
- **Pas de symetrie automatique entre couleurs cote position** : la position
  de depart reste a la charge de la strategie. Le miroir Asserv (auto en
  YELLOW) gere les coordonnees de trajectoire ; `push_elements_zone` gere
  le swap suffixe horizontal et l'index pickup (cf "Convention couleur").
- **Pas de differenciation horizontale/verticale** : on assume que la
  strategie a oriente correctement le robot. Le sens de pousse decoule de
  l'orientation courante, pas d'un parametre de la manip.

## Tests (O_PushElementsTest)

Le test integré
[O_PushElementsTest](../src/bot-test-opos6ul/O_PushElementsTest.cpp)
(code mnemonique `pe`) couvre 2 modes complementaires.

### Mode 1 — Validation logique (sans args)

```bash
cd /home/pmx/git/PMX-CORTEX/robot/build-simu-debug/bin
./bot-opos6ul pe
```

Itere sur ~42 cas et verifie la **mecanique de combinaison** (mapping
`idx -> distDirecte/distInverse`, swap couleur `SWAP_COLOR_IDX`, flip
suffixe horizontal en YELLOW, offsets `kZoneOffset_P4/P14`, garde-fous
sur `dist <= 0` et idx invalide).

Les attendus sont **derives des constantes** via
`push_elements_test_api::distDirecteAt(idx)` etc., donc le test reste
vert quand on retouche `D1..D4` (la calibration physique se fait sur
table reelle, cf section ci-dessous).

Sortie : 5 groupes de cas, tableau OK/FAIL par ligne, recap final
`=== Bilan : 42/42 PASS ===`.

### Mode 2 — Poussage reel (3 args obligatoires)

```bash
./bot-opos6ul pe <zone> <suffixe> <idx>
./bot-opos6ul pe P14 D 3              # push_elements_P14_D, idx=3, BLEU
./bot-opos6ul pe P1  H 0 /y           # push_elements_P1_H, idx=0, YELLOW
```


| Argument  | Valeurs                | Effet                                                           |
| ----------- | ------------------------ | ----------------------------------------------------------------- |
| `zone`    | `P1..P4` ou `P11..P14` | Zone ciblee                                                     |
| `suffixe` | `B                     | H                                                               |
| `idx`     | `0..5`                 | Force la valeur`pickup_P{N}` avant la manip (utile sans balise) |

Le test appelle directement
[`push_elements_zone()`](../src/bot-opos6ul/StrategyActions2026.cpp) :

- log `avance=Xmm (base=Y offset=Z) puis recul=200mm` permet de valider
  le calcul de distance ;
- en YELLOW, log `idx=X sens=YYY (YELLOW post-swap)` montre l'effet du
  swap couleur + flip suffixe ;
- le robot avance puis recule physiquement (en simu : 0.5 m/s ;
  sur table : capteurs front actifs, RetryPolicy::standard 2/2).

### Suite de tests P1 (12 cas, BLEU)

Balayage complet des 6 configs balise × 2 sens d'attaque sur la zone P1
(verticale, X=200). Avant chaque pousse, la visu SVG affiche les 4 rectangles
selon la convention balise (char[0] en HAUT, char[3] en BAS) ; le robot voit
char[3] proche pour `_B` (vient du bas) et char[0] proche pour `_H` (vient du haut).

**`_B` (robot du bas, cap=+90°) — table INVERSE :**

```bash
./bot-opos6ul pe P1 B 0 /+ 200 585 90    # BBYY INV -> 400-125 = 275mm  | vu robot : Y,Y,B,B (court)
./bot-opos6ul pe P1 B 1 /+ 200 585 90    # YYBB INV -> 400+125 = 525mm  | vu robot : B,B,Y,Y (long)
./bot-opos6ul pe P1 B 2 /+ 200 585 90    # BYYB INV -> 400+175 = 575mm  | vu robot : B,Y,Y,B
./bot-opos6ul pe P1 B 3 /+ 200 585 90    # YBBY INV -> 400+ 75 = 475mm  | vu robot : Y,B,B,Y
./bot-opos6ul pe P1 B 4 /+ 200 585 90    # BYBY INV -> 400- 75 = 325mm  | vu robot : Y,B,Y,B
./bot-opos6ul pe P1 B 5 /+ 200 585 90    # YBYB INV -> 400+ 75 = 475mm  | vu robot : B,Y,B,Y
```

**`_H` (robot du haut, cap=-90°) — table DIRECTE :**

```bash
./bot-opos6ul pe P1 H 0 /+ 200 1800 -90  # BBYY DIR -> 400+125 = 525mm  | vu robot : B,B,Y,Y (long)
./bot-opos6ul pe P1 H 1 /+ 200 1800 -90  # YYBB DIR -> 400-125 = 275mm  | vu robot : Y,Y,B,B (court)
./bot-opos6ul pe P1 H 2 /+ 200 1800 -90  # BYYB DIR -> 400+175 = 575mm  | vu robot : B,Y,Y,B
./bot-opos6ul pe P1 H 3 /+ 200 1800 -90  # YBBY DIR -> 400+ 75 = 475mm  | vu robot : Y,B,B,Y
./bot-opos6ul pe P1 H 4 /+ 200 1800 -90  # BYBY DIR -> 400+ 75 = 475mm  | vu robot : B,Y,B,Y
./bot-opos6ul pe P1 H 5 /+ 200 1800 -90  # YBYB DIR -> 400- 75 = 325mm  | vu robot : Y,B,Y,B
```

Note : le SVG est ecrase a chaque lancement (`build-simu-debug/bin/svgAPF.svg`).
Copier le fichier entre 2 tests si comparaison cote a cote necessaire.

**Points a verifier visuellement :**

1. **Rectangles initiaux (pleins)** : couleurs respectent la convention balise
   (char[0] cote Y+ / HAUT, char[3] cote Y- / BAS).
2. **Rectangles finaux (pointilles)** : decales de `dist` mm dans le sens du cap
   (vers Y+ pour `_B`, vers Y- pour `_H`).
3. **Cote physique** : les jaunes adverses doivent **sortir** de la zone P1 ;
   les bleus (la couleur) doivent **rester** dans la zone (ou en limite).

### Suite de tests P1 (12 cas, YELLOW)

Memes commandes que BLEU + option `/y`. Effets en YELLOW :

- Position miroir X : `/+ 200 585 90 /y` -> robot place a `(3000-200, 585, 90) = (2800, 585, 90)`
- Distance : `computeDistance` applique `SWAP_COLOR_IDX[idx]` -> table lue a l'index symetrique
- Suffixe : zones verticales (P1) **non flippees** (le flip `_D <-> _G` ne concerne que les horizontales)
- Affichage SVG : pas de swap (l'idx balise est utilise tel quel pour les couleurs)

**`_B` (cap=+90°) — table INVERSE :**

```bash
./bot-opos6ul pe P1 B 0 /+ 200 585 90 /y    # BBYY YELLOW -> SWAP[0]=1 -> distInv[1]= 125 -> 525mm
./bot-opos6ul pe P1 B 1 /+ 200 585 90 /y    # YYBB YELLOW -> SWAP[1]=0 -> distInv[0]=-125 -> 275mm
./bot-opos6ul pe P1 B 2 /+ 200 585 90 /y    # BYYB YELLOW -> SWAP[2]=3 -> distInv[3]=  75 -> 475mm
./bot-opos6ul pe P1 B 3 /+ 200 585 90 /y    # YBBY YELLOW -> SWAP[3]=2 -> distInv[2]= 175 -> 575mm
./bot-opos6ul pe P1 B 4 /+ 200 585 90 /y    # BYBY YELLOW -> SWAP[4]=5 -> distInv[5]=  75 -> 475mm
./bot-opos6ul pe P1 B 5 /+ 200 585 90 /y    # YBYB YELLOW -> SWAP[5]=4 -> distInv[4]= -75 -> 325mm
```

**`_H` (cap=-90°) — table DIRECTE :**

```bash
./bot-opos6ul pe P1 H 0 /+ 200 1800 -90 /y  # BBYY YELLOW -> SWAP[0]=1 -> distDir[1]=-125 -> 275mm
./bot-opos6ul pe P1 H 1 /+ 200 1800 -90 /y  # YYBB YELLOW -> SWAP[1]=0 -> distDir[0]= 125 -> 525mm
./bot-opos6ul pe P1 H 2 /+ 200 1800 -90 /y  # BYYB YELLOW -> SWAP[2]=3 -> distDir[3]=  75 -> 475mm
./bot-opos6ul pe P1 H 3 /+ 200 1800 -90 /y  # YBBY YELLOW -> SWAP[3]=2 -> distDir[2]= 175 -> 575mm
./bot-opos6ul pe P1 H 4 /+ 200 1800 -90 /y  # BYBY YELLOW -> SWAP[4]=5 -> distDir[5]= -75 -> 325mm
./bot-opos6ul pe P1 H 5 /+ 200 1800 -90 /y  # YBYB YELLOW -> SWAP[5]=4 -> distDir[4]=  75 -> 475mm
```

**Verification YELLOW :**

1. Robot positionne cote droit de la table (X ≈ 2800), pas a gauche.
2. Le log `push_elements_zone P1 idx=X sens=YYY (YELLOW post-swap)` doit indiquer
   l'idx apres swap dans le calcul, et donner la distance attendue.
3. La symetrie par rapport au BLEU doit etre respectee : pour le meme idx balise,
   le scenario physique YELLOW pousse les BLEUS adverses (au lieu des JAUNES en BLEU).

### Autres zones verticales (P2, P11, P12)

Geometrie **identique a P1** : memes valeurs `distDirecte`/`distInverse`, pas
d'offset. Les commandes de test sont les memes que pour P1 en remplacant juste
le nom de zone et en adaptant la position `/+` selon ou la zone se situe sur
la table reelle :

```bash
./bot-opos6ul pe P2  B 0 /+ <x> <y_bas>  90    # distance identique a pe P1 B 0
./bot-opos6ul pe P11 H 3 /+ <x> <y_haut> -90   # distance identique a pe P1 H 3
# etc.
```

→ Inutile de re-calibrer ces zones : si la calibration P1 est bonne, ces 3 zones
le sont aussi.

### Suite de tests P3 (12 cas, BLEU) — zone horizontale

Zones horizontales (P3, P4, P13, P14) : lecture balise **GAUCHE → DROITE**
(`char[0]` cote X-, `char[3]` cote X+). Le robot pousse selon l'axe X :

- **`_G`** : robot vient de GAUCHE (X-), cap=**0°**, table **DIRECTE**
- **`_D`** : robot vient de DROITE (X+), cap=**180°**, table **INVERSE**

Memes distances que P1 (pas d'offset pour P3 / P13).

**`_G` (cap=0°) — table DIRECTE :**

```bash
./bot-opos6ul pe P3 G 0 /+ 875 1800 0    # BBYY DIR -> 525mm | vu robot : B,B,Y,Y (long)
./bot-opos6ul pe P3 G 1 /+ 875 1800 0    # YYBB DIR -> 275mm | vu robot : Y,Y,B,B (court)
./bot-opos6ul pe P3 G 2 /+ 875 1800 0    # BYYB DIR -> 575mm | vu robot : B,Y,Y,B
./bot-opos6ul pe P3 G 3 /+ 875 1800 0    # YBBY DIR -> 475mm | vu robot : Y,B,B,Y
./bot-opos6ul pe P3 G 4 /+ 875 1800 0    # BYBY DIR -> 475mm | vu robot : B,Y,B,Y
./bot-opos6ul pe P3 G 5 /+ 875 1800 0    # YBYB DIR -> 325mm | vu robot : Y,B,Y,B
```

**`_D` (cap=180°) — table INVERSE :**

```bash
./bot-opos6ul pe P3 D 0 /+ 1315 1800 180  # BBYY INV -> 275mm | vu robot : Y,Y,B,B (court)
./bot-opos6ul pe P3 D 1 /+ 1315 1800 180  # YYBB INV -> 525mm | vu robot : B,B,Y,Y (long)
./bot-opos6ul pe P3 D 2 /+ 1315 1800 180  # BYYB INV -> 575mm | vu robot : B,Y,Y,B
./bot-opos6ul pe P3 D 3 /+ 1315 1800 180  # YBBY INV -> 475mm | vu robot : Y,B,B,Y
./bot-opos6ul pe P3 D 4 /+ 1315 1800 180  # BYBY INV -> 325mm | vu robot : Y,B,Y,B
./bot-opos6ul pe P3 D 5 /+ 1315 1800 180  # YBYB INV -> 475mm | vu robot : B,Y,B,Y
```

Position robot a adapter : `<x_gauche>` = X juste a gauche de la zone (octogone
face touche le bord X- des rects), `<x_droite>` = X juste a droite (octogone face
touche le bord X+ des rects). `<y>` = Y centre de la zone.

### Zones avec offset (P4, P14)

Memes commandes que P3 / P13 mais distances **reduites de 50mm** :


| idx | seq  | P4 / P14`_G` DIR | P4 / P14`_D` INV |
| ----- | ------ | ------------------ | ------------------ |
| 0   | BBYY | 475 (525-50)     | 225 (275-50)     |
| 1   | YYBB | 225              | 475              |
| 2   | BYYB | 525 (575-50)     | 525              |
| 3   | YBBY | 425 (475-50)     | 425              |
| 4   | BYBY | 425              | 275 (325-50)     |
| 5   | YBYB | 275              | 425              |

```bash
./bot-opos6ul pe P4  G 0 /+ <x_gauche> <y_P4>  0    # BBYY DIR + offset_P4=-50 -> 475mm
./bot-opos6ul pe P14 D 2 /+ <x_droite> <y_P14> 180  # BYYB INV + offset_P14=-50 -> 525mm
# etc.
```

### P11, P12, P13, P14 et autres : note

Les zones P11..P14 sont les versions cote YELLOW de P1..P4 (selon le plan de table).
La balise distingue les configurations entre les 8 zones (`pickup_P1`..`pickup_P14`),
mais la **mecanique de calcul de distance est identique** au sein des paires
P1/P11, P2/P12, P3/P13, P4/P14 — meme table `distDirecte`/`distInverse`, meme
offset (pour P4/P14). Inutile de re-calibrer.

### Calibration sur table (D1..D4 et offsets)

Le test C++ ne peut pas valider les **valeurs absolues** de `D1..D4` ni
des offsets : ce sont des distances physiques qui dependent du robot
(geometrie, frottement, PWM). La procedure de reglage reste manuelle :

1. Mettre 4 elements sur une zone reelle, en config connue.
2. Selectionner cette config sur le LCD tactile balise.
3. Lancer `./bot-opos6ul pe <zone> <suffixe> <idx>` sur le robot.
4. Observer si tous les elements adverses sortent de la zone.
5. Sinon, ajuster `D1..D4` ou `kZoneOffset_P{4,14}` dans
   [StrategyActions2026.cpp](../src/bot-opos6ul/StrategyActions2026.cpp),
   recompiler, redeployer.
6. Iterer pour les 6 configs et les 2 couleurs.

Astuce : grace au `SWAP_COLOR_IDX` applique automatiquement en YELLOW,
regler les 6 valeurs `D1..D4` (+ `D1_INV..D4_INV`) en BLEU regle
simultanement les 2 couleurs. Une seule passe de calibration suffit.

## Test en simulation (sans balise)

Sans balise, `pickup_P{N}` reste a 0 (BBYY). Le mode 2 du test
[O_PushElementsTest](../src/bot-test-opos6ul/O_PushElementsTest.cpp)
force la valeur souhaitee via son 3eme argument (idx 0..5) avant de
lancer la manip — equivalent simu de la saisie balise.

L'ancienne option CLI `/u <zone> <value>` (utilisee depuis `m /k /s PMX*`) reste disponible dans
[Robot.cpp](../src/common/Robot.cpp) pour les runs strategie complets,
mais pour valider une zone isolee, **prefere `pe <zone> <suffixe> <idx>`** (plus direct, n'a pas besoin d'une strategie JSON).
