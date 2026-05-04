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

| Composant | Role | Reference |
|---|---|---|
| Menu LCD tactile balise | Saisie 6 configs par zone | [LCDScreen.cpp create_pickup_config](../../teensy/IO_t41_ToF_DetectionBeacon/src/LCDScreen.cpp) |
| Spec UI beacon | Documentation complete | [MATCH_CONFIG_UI.md](../../teensy/IO_t41_ToF_DetectionBeacon/MATCH_CONFIG_UI.md) |
| Transport I2C | `MatchSettingsData::pickup_P*` | [ASensorsDriver.hpp:82-110](../src/common/interface/ASensorsDriver.hpp#L82-L110) |
| Adoption brain | `MenuBeaconLCDTouch::pollInputs` | [MenuBeaconLCDTouch.cpp](../src/common/menu/MenuBeaconLCDTouch.cpp) |
| Stockage brain | `Robot::pickup_P{N}_` + getters | [Robot.hpp:180-187, 621-637](../src/common/Robot.hpp#L180-L187) |
| Manipulation | `push_elements_zone` + 16 wrappers | [StrategyActions2026.cpp](../src/bot-opos6ul/StrategyActions2026.cpp) |

## Numerotation et ordre des configurations

La balise expose 6 configurations par zone, indexees 0..5, dans cet ordre :

| idx | sequence | description |
|---|---|---|
| 0 | BBYY | 2 bleus puis 2 jaunes (sens lecture balise) |
| 1 | YYBB | 2 jaunes puis 2 bleus |
| 2 | BYYB | bleu, jaune, jaune, bleu |
| 3 | YBBY | jaune, bleu, bleu, jaune |
| 4 | BYBY | alternance commencant par bleu |
| 5 | YBYB | alternance commencant par jaune |

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

| Constante | Valeur (mm) | Usage |
|---|---|---|
| `D_RETREAT` | 200 | recul de degagement apres la pousse |
| `D1` | 250 | distance courante (1 element a pousser) |
| `D2` | 350 | distance la plus longue (3 elements) |
| `D3` | 100 | distance moyenne |
| `D4` |  50 | distance la plus courte (0..1 element) |
| `D1_INV`, `D2_INV`, `D3_INV`, `D4_INV` | 450, 550, 300, 250 | placeholder sens inverse |

Modifier ces 4 (ou 8) constantes ajuste automatiquement les 2 tables.

### Mapping image utilisateur (1..6) ↔ ordre balise (0..5)

L'image initiale de calibration numerote les 6 placements de 1 a 6, dans un
ordre different de celui de la balise LCD. Tableau de correspondance :

| Image # | Sequence | Distance image | idx balise | Sequence balise |
|---|---|---|---|---|
| 1 | BBJJ | **D1** | 0 | BBYY |
| 2 | BJBJ | **D1** | 4 | BYBY |
| 3 | BJJB | **D2** | 2 | BYYB |
| 4 | JBBJ | **D1** | 3 | YBBY |
| 5 | JBJB | **D3** | 5 | YBYB |
| 6 | JJBB | **D4** | 1 | YYBB |

(Note : "B" = Bleu, "J" = Jaune dans la numerotation image ; "B" = Bleu,
"Y" = Yellow dans la balise — meme convention, juste une lettre differente.)

### Tables compilees (idx balise 0..5)

| idx balise | sequence | distDirecte | distInverse |
|---|---|---|---|
| 0 | BBYY (img #1) | `D1` (250) | `D1_INV` (450) |
| 1 | YYBB (img #6) | `D4` (50)  | `D4_INV` (250) |
| 2 | BYYB (img #3) | `D2` (350) | `D2_INV` (550) |
| 3 | YBBY (img #4) | `D1` (250) | `D1_INV` (450) |
| 4 | BYBY (img #2) | `D1` (250) | `D1_INV` (450) |
| 5 | YBYB (img #5) | `D3` (100) | `D3_INV` (300) |

Code C++ correspondant :
```cpp
constexpr float D1 = 250.0f, D2 = 350.0f, D3 = 100.0f, D4 = 50.0f;
constexpr float D1_INV = 450.0f, D2_INV = 550.0f, D3_INV = 300.0f, D4_INV = 250.0f;

static constexpr float distDirecte[6] = { D1, D4, D2, D1, D1, D3 };
static constexpr float distInverse[6] = { D1_INV, D4_INV, D2_INV, D1_INV, D1_INV, D3_INV };

const float dist = sensInverse ? distInverse[pickupIdx] : distDirecte[pickupIdx];
```

**`distInverse` est un placeholder** : valeurs +200mm par rapport a directe,
a calibrer sur table. Le principe : en sens inverse, on rencontre les
elements adverses en TETE et il faut les pousser jusqu'a la sortie opposee
(donc plus loin qu'en sens directe).

## Sequence d'execution

La fonction `push_elements_zone(pickupIdx, zoneName, sensInverse)` enchaine :

1. Lecture `pickupIdx` (0..5).
2. Validation `pickupIdx <= 5` (sinon abort, `return false`).
3. Choix `dist` dans `distDirecte` ou `distInverse` selon `sensInverse`.
4. **`setMaxSpeed(true, 20)`** : vitesse reduite pour avoir du couple et ne
   pas balayer les elements en bord de zone.
5. Configuration capteurs :
   - **Front center actif** : on veut detecter la collision sur l'element
     pousse pour que le retry asserv reagisse correctement.
   - Front lateral + back : ignores pendant la sequence.
6. `nav.line(+dist, policyPush)` : avance pour pousser.
   - Si `ts != TRAJ_FINISHED` : reset emergency, log erreur, `return false`.
7. `nav.line(-D_RETREAT, policyPush)` : recul de degagement.
   - Echec non-bloquant : la pousse a deja eu lieu, on logue et on continue.
8. `return true`.

**RetryPolicy utilisee** : `RetryPolicy::standard()` — equivaut a
`{ 2000000, 2, 2, 0, 0, false }`, soit 2 retries sur obstacle et 2 retries sur
collision avec 2s d'attente entre tentatives. Le retry est limite parce qu'en
push, l'objet est forcement en contact ; au-dela de 2 retries, c'est un vrai
blocage.

## Wrappers par zone (16 actions)

Pour chaque zone, **2 wrappers** correspondant aux 2 cotes d'arrivee possibles
du robot. Le suffixe `_B/_H/_D/_G` designe le **cote d'arrivee** du robot
dans la zone (= d'ou il vient juste avant la pousse) :

| Suffixe | Cote d'arrivee | Direction d'avance | Sens (vs lecture balise) | Table |
|---|---|---|---|---|
| `_B` | par le BAS  (Y=0, public)       | vers le HAUT (Y+)      | INVERSE | `distInverse` |
| `_H` | par le HAUT (Y+, fond table)    | vers le BAS  (Y=0)     | DIRECTE | `distDirecte` |
| `_D` | par la DROITE (X=3000, jaune)   | vers la GAUCHE (X=0)   | INVERSE | `distInverse` |
| `_G` | par la GAUCHE (X=0, bleu)       | vers la DROITE (X=3000)| DIRECTE | `distDirecte` |

**Pourquoi cette correspondance** : la lecture balise va haut→bas (vertical)
ou gauche→droite (horizontal). Le sens DIRECTE = robot avance dans le sens
de lecture, donc arrive par le cote DEBUT (haut ou gauche) → wrappers `_H`
et `_G`. Les wrappers `_B` / `_D` (arrive par le cote FIN) lisent la sequence
"a l'envers" mentalement → sens INVERSE → distance differente (table separee).

Liste des 16 wrappers :

| Verticales (P1, P2, P11, P12) | Horizontales (P3, P4, P13, P14) |
|---|---|
| `push_elements_P1_B`,  `push_elements_P1_H`  | `push_elements_P3_D`,  `push_elements_P3_G`  |
| `push_elements_P2_B`,  `push_elements_P2_H`  | `push_elements_P4_D`,  `push_elements_P4_G`  |
| `push_elements_P11_B`, `push_elements_P11_H` | `push_elements_P13_D`, `push_elements_P13_G` |
| `push_elements_P12_B`, `push_elements_P12_H` | `push_elements_P14_D`, `push_elements_P14_G` |

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

| Sur l'ecran | Sur la table |
|---|---|
| HAUT     | ARRIERE (Y+ grand, fond de table) |
| BAS      | AVANT   (Y=0, cote public)        |
| GAUCHE   | X=0     (cote bleu, NID BLEU)     |
| DROITE   | X=3000  (cote jaune, NID JAUNE)   |

Sens de lecture des sequences balise :

| Zones | Orientation | Lecture sequence (1er → 4eme caractere) |
|---|---|---|
| P1, P2, P11, P12 | Verticale | **HAUT → BAS** ecran = ARRIERE → AVANT (Y+ → Y=0) |
| P3, P4, P13, P14 | Horizontale | **GAUCHE → DROITE** ecran = X=0 → X=3000 |

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

| Constante | Valeur (mm) | Usage |
|---|---|---|
| `kZoneOffset_P4`  | 0.0 (placeholder) | offset specifique pour P4 |
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

## Tests de validation simu (calibration distances)

Cette section liste les tests a executer en SIMU pour valider chaque
constante (`D1..D4`, `D1_INV..D4_INV`) et chaque offset
(`kZoneOffset_P4`, `kZoneOffset_P14`).

**Principe** : la fonction `push_elements_zone` logue
`avance=Xmm (base=Y offset=Z)` au moment de la manip. Il suffit de regarder
ce log pour verifier que le calcul est correct, sans avoir besoin que la
pousse aboutisse physiquement (la geometrie de la table n'est pas en cause).

### Option CLI `/u` etendue

Format : **`/u <zone[_X]> <value>`** ou
- `<zone>`  = `P1`..`P4` ou `P11`..`P14`
- `_X` (optionnel) = `_B`, `_H`, `_D`, `_G` (force le wrapper a appeler)
- `<value>` = `0..5` (idx config beacon)

Sans `_X`, le wrapper utilise est celui defini dans le JSON. Avec `_X`, le
runner remplace **tous** les `push_elements_P*_*` de la strategie par
`push_elements_<zone>_<X>` (DEBUG, simu uniquement).

### Setup commun

```bash
cd /home/pmx/git/PMX-CORTEX/robot/build-simu-debug/bin
```

Aucune modification du JSON ni de rebuild necessaire entre les tests grace a
l'override `/u Pn_X`.

### Test 1 — Validation `distDirecte` (sens DIRECTE)

Wrapper `_H` (vertical) ou `_G` (horizontal) = sens directe.

```bash
./bot-opos6ul m /k /s PMX0 /u P1_H 0 2>&1 | grep "avance="
./bot-opos6ul m /k /s PMX0 /u P1_H 1 2>&1 | grep "avance="
./bot-opos6ul m /k /s PMX0 /u P1_H 2 2>&1 | grep "avance="
./bot-opos6ul m /k /s PMX0 /u P1_H 3 2>&1 | grep "avance="
./bot-opos6ul m /k /s PMX0 /u P1_H 4 2>&1 | grep "avance="
./bot-opos6ul m /k /s PMX0 /u P1_H 5 2>&1 | grep "avance="
```

| Commande | Sequence | Constante | Distance attendue |
|---|---|---|---|
| `/u P1_H 0` | BBYY (img#1) | `D1` | **250 mm** |
| `/u P1_H 1` | YYBB (img#6) | `D4` | **50 mm**  |
| `/u P1_H 2` | BYYB (img#3) | `D2` | **350 mm** |
| `/u P1_H 3` | YBBY (img#4) | `D1` | **250 mm** |
| `/u P1_H 4` | BYBY (img#2) | `D1` | **250 mm** |
| `/u P1_H 5` | YBYB (img#5) | `D3` | **100 mm** |

→ valide `D1`, `D2`, `D3`, `D4`.

### Test 2 — Validation `distInverse` (sens INVERSE)

Wrapper `_B` (vertical) ou `_D` (horizontal) = sens inverse.

```bash
./bot-opos6ul m /k /s PMX0 /u P1_B 0 2>&1 | grep "avance="
./bot-opos6ul m /k /s PMX0 /u P1_B 1 2>&1 | grep "avance="
# ... idem 2, 3, 4, 5
```

| Commande | Sequence | Constante | Distance attendue (placeholder) |
|---|---|---|---|
| `/u P1_B 0` | BBYY | `D1_INV` | **450 mm** |
| `/u P1_B 1` | YYBB | `D4_INV` | **250 mm** |
| `/u P1_B 2` | BYYB | `D2_INV` | **550 mm** |
| `/u P1_B 3` | YBBY | `D1_INV` | **450 mm** |
| `/u P1_B 4` | BYBY | `D1_INV` | **450 mm** |
| `/u P1_B 5` | YBYB | `D3_INV` | **300 mm** |

→ valide `D1_INV..D4_INV` (placeholders, a calibrer).

### Test 3 — Validation `kZoneOffset_P4`

```bash
# Avec kZoneOffset_P4 = 0 (default) :
./bot-opos6ul m /k /s PMX0 /u P4_G 0 2>&1 | grep "avance="
# Attendu : avance=250mm (base=250 offset=0)
```

Modifier `kZoneOffset_P4` dans
[StrategyActions2026.cpp](../src/bot-opos6ul/StrategyActions2026.cpp) (ex
`-50.0f`), recompiler `bot-opos6ul`, relancer :
```bash
./bot-opos6ul m /k /s PMX0 /u P4_G 0 2>&1 | grep "avance="
# Attendu : avance=200mm (base=250 offset=-50)
```

Garde-fou (offset trop negatif) : `kZoneOffset_P4 = -300.0f` + idx 1 (D4=50)
→ `dist=50-300=-250 <= 0 → abort + log error`. Tester aussi avec `_D`
(sens inverse) si besoin.

→ valide `kZoneOffset_P4`.

### Test 4 — Validation `kZoneOffset_P14`

Identique Test 3 mais avec `/u P14_G <idx>`. Modifier `kZoneOffset_P14` dans
le source, recompiler, relancer.

→ valide `kZoneOffset_P14`.

### Tester toutes les couleurs

Ajouter `/y` a n'importe quelle commande pour passer en JAUNE. En YELLOW :
- pour les zones horizontales (P3, P4, P13, P14), le suffixe `_D`/`_G` est
  flippe automatiquement ;
- l'index pickup est swap via `SWAP_COLOR_IDX[idx]` (paires (0,1)(2,3)(4,5))
  pour conserver la semantique "pousser SA majorite".

Verifier dans le log `push_elements_zone <zone> idx=X sens=YYY (YELLOW post-swap)`
que la valeur d'idx loguee est bien `SWAP_COLOR_IDX[idx_original]` et que le
sens correspond au flip attendu pour les horizontales.

### Script bash optionnel (Tests 1+2 automatises)

```bash
cd /home/pmx/git/PMX-CORTEX/robot
./sh/test_push_elements.sh
```

Sortie : tableau idx / sens / distance mesuree / OK ou FAIL (12 cas).

## Test en simulation (sans balise)

En SIMU la balise n'est pas branchee, donc `pickup_P{N}` reste a sa valeur par
defaut (0 = BBYY). Pour tester les autres configs, l'option CLI **`/u <zone>
<value>`** force la valeur d'une zone juste apres le parsing CLI :

```bash
./bot-opos6ul m /k /s PMX2 /u P1 5         # P1 = idx 5 (YBYB), BLEU -> D3
./bot-opos6ul m /k /s PMX2 /u P14 2 /y     # P14 = idx 2 (BYYB), JAUNE -> D1
```

| Argument | Valeurs | Description |
|---|---|---|
| `zone` | `P1..P4` ou `P11..P14` | Zone visee (8 valeurs possibles) |
| `value` | `0..5` | Config beacon (cf. table d'ordre balise plus haut) |

Limitations :
- 1 seule zone par run (option non repetable). Pour configurer plusieurs
  zones, lancer plusieurs runs ou modifier la valeur dans le menu LCD2x16.
- En PHASE_MATCH la modification est rejetee : l'option doit etre passee au
  demarrage CLI, ce qui est le cas (parsing en PHASE_CONFIG).

Combiner avec `/y` pour tester la table JAUNE (qui est l'inverse de la table
BLEU pour le meme idx).

## Reglage sur table

1. Mettre le robot devant une zone reelle avec 4 elements en config connue.
2. Selectionner la config sur le LCD tactile balise.
3. Lancer une strategie qui appelle uniquement `push_elements_P{N}` (ou
   utiliser un test fonctionnel dedie si cree).
4. Observer si tous les elements adverses sortent de la zone — sinon ajuster
   la valeur correspondante (D1..D4) dans `StrategyActions2026.cpp`.
5. Recompiler / redeployer (cf [BUILD.md](BUILD.md)).
6. Iterer pour les 6 configs et les 2 couleurs.

Astuce : grace au `SWAP_COLOR_IDX` applique automatiquement en YELLOW, regler
les 6 valeurs `D1..D4` (+ `D1_INV..D4_INV`) en BLEU regle simultanement les
2 couleurs. Une seule passe de calibration suffit.
