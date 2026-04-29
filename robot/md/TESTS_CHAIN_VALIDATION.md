# Tests de validation : sleep CBOR 50 ms + mode `chain`

Document de tests pour comparer **deux chemins d'exécution équivalents** sur le même
scénario :

1. **Voie O_Nav** : opcode CLI `lr` (ou `wp`), qui appelle directement
   `Navigator` / `Asserv` sans passer par le runner JSON. Les tests
   fonctionnels n'ont pas de phase ARMED/PRIMED → pas besoin de `/k`.
2. **Voie Runner** : `bot-opos6ul /s PMX1 /k` qui charge `strategyPMX1.json`
   et exécute via `StrategyJsonRunner`. `/k` = skip-setup (bypass tirette
   en simu).

L'objectif : jouer la voie O_Nav (référence sans le sleep 50 ms inter-task), puis
la voie Runner sur le même scénario, et **comparer les traces SVG**
(`svgAPF.svg`) pour vérifier que :

- Modif 1 (sleep 200 → 50 ms) ne dégrade pas le comportement de base.
- Modif 2 (séparation user/obstacle dans `setMaxSpeed`) → SET_SPEED persiste
  entre 2 mouvements consécutifs.
- Modif 3 (`"chain": true`) supprime la dérive d'asserv résiduelle.

## Setup commun

- Position de départ : **(300, 130, 90°)** — robot face Y+ (Nord), espace libre en Y+.
- Surface table : 3000×2000 mm.
- Tolérance retour position : 30 mm pour les boucles fermées.

Les tests **O_Nav** passent la pose de départ via `/+ x y theta_deg` :
pas besoin de modifier `initPMX1.json`. Format segment de `lr` : 4 params
positionnels par segment = `aPre d aPost back` (jusqu'à 5 segments).

Tests **Runner** : copier le JSON dans `bin/strategyPMX1.json` puis lancer
`bot-opos6ul /s PMX1 /k`. Le `/k` (skip setup) bypass la phase ARMED/PRIMED
de la tirette — indispensable en simu où `SwitchDriverSimu::tirettePressed()`
retourne toujours 0. Sur arm-release avec tirette physique, retirer le `/k`.
La pose vient de `bin/initPMX1.json` — adapte-la si tu veux pas la valeur
par défaut (230, 130, 90° + setpos_tasks +180 mm).

Note `lr` : `/p 1` passe par `Navigator::line()` (le même chemin que le
runner). `/pmode 0` passe direct par `Asserv::line()` (court-circuite Navigator).
Pour comparer au runner, **garder `/p 1`**.

---

## Test A — LINE 1 m, baseline

**But** : vérifier que la modif 1 (sleep 50 ms) ne régresse pas le mouvement de base.

### O_Nav

```bash
./bot-opos6ul lr 0 1000 0 0 /+ 300 130 90 /p 1
```

Format segment : `aPre d aPost back` = `0 1000 0 0` (LINE 1000 pure, pas de rot).
→ `Navigator::line(1000)`. Attendu : TRAJ_FINISHED, position (300, 1130, ~90°).

### Runner

`strategyPMX1.json` :
```json
[
  { "id": 1, "desc": "A: LINE 1000 baseline",
    "tasks": [
      { "type": "MOVEMENT", "subtype": "LINE", "dist": 1000, "desc": "1m AV" }
    ]
  }
]
```

### Indicateurs

- Trace SVG : ligne droite bleue de (300, 130) à (300, 1130).
- Position finale : `|x - 300| < 10 mm`, `|y - 1130| < 10 mm`, `|theta - 90°| < 1°`.
- O_Nav et Runner doivent donner **la même position finale** à 5 mm près.

---

## Test B — SET_SPEED persiste sur 2 LINE consécutives

**But** : vérifier que la modif 2 (séparation user/obstacle) garantit que
`SET_SPEED 30` reste actif entre le 1ᵉʳ et le 2ᵉ LINE.

### O_Nav (CLI `lr`)

`lr` accepte `/v <pct>` qui appelle `Asserv::setMaxSpeed(true, pct, pct)`
en début de séquence, **identique au comportement du runner SET_SPEED**.

```bash
./bot-opos6ul lr 0 1000 0 0  0 -1000 0 0 /+ 300 130 90 /p 1 /v 30
```

2 segments : `(0, 1000, 0, 0)` = LINE +1000 pur, `(0, -1000, 0, 0)` = LINE -1000 pur.
Mesurer la durée des 2 LINE dans les logs (`time=XXXms` à la fin de chaque
trajectoire dans `executeWithRetry`).

### Runner

```json
[
  { "id": 1, "desc": "B: SET_SPEED 30 persiste",
    "tasks": [
      { "type": "SPEED", "subtype": "SET_SPEED", "speed_percent": 30, "desc": "cap 30%" },
      { "type": "MOVEMENT", "subtype": "LINE", "dist": 1000, "desc": "1m AV @30%" },
      { "type": "MOVEMENT", "subtype": "LINE", "dist": -1000, "desc": "1m AR @30%" },
      { "type": "SPEED", "subtype": "SET_SPEED", "speed_percent": 100, "desc": "restore" }
    ]
  }
]
```

### Indicateurs

| Comportement | Verdict |
|---|---|
| 1ᵉʳ LINE ≈ 2ᵉ LINE en durée (les 2 à 30%) | ✅ PASS — modif 2 OK |
| 2ᵉ LINE significativement plus rapide que le 1ᵉʳ | ❌ FAIL — SET_SPEED s'efface |
| O_Nav `lr /v 30` durée ≈ Runner durée | ✅ PASS cohérence |

Aussi vérifier dans les logs : pas de `setMaxSpeed [PWM cap] DISABLED` parasite
entre le 1ᵉʳ et le 2ᵉ LINE.

---

## Test C — FACE_TO + LINE **sans** chain (reproduit le bug d'origine)

**But** : voir le biais latéral résiduel entre 2 cmds (50 ms après modif 1,
200 ms avant). Sur arm-release uniquement (la simu n'a pas de dérive Nucleo).

### O_Nav

`lr` ne supporte pas FACE_TO direct (visée vers un point), mais on peut le
proxy avec une **rotation absolue + LINE** dans un seul segment grâce à
`aPre` :

```bash
./bot-opos6ul lr 0 1000 0 0 /+ 300 130 90 /p 1 /m 1
```

→ aPre=0 → `Navigator::rotateAbsDeg(0)` (alignement absolu cap 0° : depuis
cap initial 90°, rotation effective de -90°) puis LINE 1000 vers X+. Position
finale attendue : (1300, 130, 0°). Le FACE_TO du Runner ci-dessous fait
exactement la même séquence (cible (1300,130) depuis (300,130,90°) → cap 0°).

### Runner

```json
[
  { "id": 1, "desc": "C: FACE_TO + LINE no-chain",
    "tasks": [
      { "type": "MOVEMENT", "subtype": "FACE_TO", "position_x": 1300, "position_y": 130, "desc": "face X+" },
      { "type": "MOVEMENT", "subtype": "LINE", "dist": 1000, "desc": "1m AV" }
    ]
  }
]
```

### Indicateurs

- O_Nav : ligne droite quasi parfaite (pas de FACE_TO, juste LINE).
- Runner sans chain : sleep 50 ms entre FACE_TO et LINE → biais latéral attendu
  sur arm. Comparer à Test D (avec chain) pour mesurer le delta.

---

## Test D — FACE_TO chain + LINE (la vraie correction)

**But** : valider que `chain: true` supprime la dérive en empilant les cmds
dans la queue Nucleo (pas d'IDLE intermédiaire).

### O_Nav

**Pas de pendant CLI direct** pour le mode chain primitives. L'API chain n'existe
que dans `Asserv::*Send` (utilisée par le runner quand `chain: true`) et n'est
exposée par aucun test fonctionnel.

→ Comparer visuellement la voie Runner ci-dessous au Test C (même séquence
**sans** chain) pour juger le gain.

### Runner

```json
[
  { "id": 1, "desc": "D: FACE_TO chain + LINE",
    "tasks": [
      { "type": "MOVEMENT", "subtype": "FACE_TO", "position_x": 1300, "position_y": 130, "chain": true, "desc": "face X+ chain" },
      { "type": "MOVEMENT", "subtype": "LINE", "dist": 1000, "desc": "1m AV (queue Nucleo)" }
    ]
  }
]
```

### Indicateurs

- Trace SVG : ligne droite **franche**, biais latéral quasi nul.
- Versus Test C : différence visuelle nette si la dérive existait.
- Sur arm-release uniquement (la simu n'a pas le bug d'origine).

---

## Test E — Carré 20 cm sans chain (dérive cumulative)

**But** : mesurer l'erreur de bouclage après 4 ROTATE + 4 LINE.

### O_Nav

```bash
./bot-opos6ul lr \
  0 200 90 0 \
  0 200 90 0 \
  0 200 90 0 \
  0 200 90 0 \
  /+ 300 130 90 /p 1
```

4 segments `(0, 200, 90, 0)` = pour chacun : LINE 200 puis ROT 90 (relatif).
→ équivalent à 4× `Navigator::line(200)` + `Navigator::rotateDeg(90)`. Mesurer
la position finale dans les logs ou la trace SVG.

### Runner

```json
[
  { "id": 1, "desc": "E: carre 20cm no-chain",
    "tasks": [
      { "type": "MOVEMENT", "subtype": "LINE", "dist": 200 },
      { "type": "MOVEMENT", "subtype": "ROTATE_DEG", "angle_deg": 90 },
      { "type": "MOVEMENT", "subtype": "LINE", "dist": 200 },
      { "type": "MOVEMENT", "subtype": "ROTATE_DEG", "angle_deg": 90 },
      { "type": "MOVEMENT", "subtype": "LINE", "dist": 200 },
      { "type": "MOVEMENT", "subtype": "ROTATE_DEG", "angle_deg": 90 },
      { "type": "MOVEMENT", "subtype": "LINE", "dist": 200 },
      { "type": "MOVEMENT", "subtype": "ROTATE_DEG", "angle_deg": 90 }
    ]
  }
]
```

### Indicateurs

- Distance finale au point de départ : `sqrt(dx² + dy²)`.
- Angle final : `|theta - 0°|`.
- O_Nav vs Runner : Runner doit avoir une dérive ≥ O_Nav (sleep 50 ms × 7
  transitions = 350 ms d'inertie cumulée). Mesurer le delta.

> Carré 20 cm depuis (300, 130, 90°) : le robot trace
> (300,130) → (300,330) → (100,330) → (100,130) → (300,130). Reste dans
> la table.

---

## Test F — Carré 20 cm avec ROTATE chain

**But** : valider que `chain: true` sur les ROTATE diminue la dérive cumulative.

### O_Nav

**Pas de pendant CLI direct** (même raison que Test D). Comparer visuellement
le Runner à Test E.

### Runner

```json
[
  { "id": 1, "desc": "F: carre 20cm chain ROTATE",
    "tasks": [
      { "type": "MOVEMENT", "subtype": "LINE", "dist": 200 },
      { "type": "MOVEMENT", "subtype": "ROTATE_DEG", "angle_deg": 90, "chain": true },
      { "type": "MOVEMENT", "subtype": "LINE", "dist": 200 },
      { "type": "MOVEMENT", "subtype": "ROTATE_DEG", "angle_deg": 90, "chain": true },
      { "type": "MOVEMENT", "subtype": "LINE", "dist": 200 },
      { "type": "MOVEMENT", "subtype": "ROTATE_DEG", "angle_deg": 90, "chain": true },
      { "type": "MOVEMENT", "subtype": "LINE", "dist": 200 },
      { "type": "MOVEMENT", "subtype": "ROTATE_DEG", "angle_deg": 90 }
    ]
  }
]
```

### Indicateurs

- Distance finale au départ < distance finale du Test E.
- Si delta F < E significatif (≥ 30 % par exemple) : la modif chain apporte un gain net.

---

## Test G — SET_SPEED + ROTATE (rotation ralentie)

**But** : vérifier que `SET_SPEED 30` s'applique aussi aux **rotations**, pas
seulement aux LINE. Si la modif 2 (séparation user/obstacle) est correcte,
le cap PWM doit s'appliquer aux 2 moteurs même quand ils tournent en sens
opposé.

### O_Nav (CLI `lr`)

```bash
./bot-opos6ul lr 90 0 0 0  -90 0 0 0 /+ 300 130 90 /p 1 /v 30
```

2 segments : `(90, 0, 0, 0)` = ROT +90° pure (aPre=90, d=0 skip LINE),
`(-90, 0, 0, 0)` = ROT -90° pure. Mesurer la durée de chaque rotation : doit
être identique entre les 2 (le SET_SPEED doit persister entre la 1ʳᵉ et la
2ᵉ rotation). Le robot reste sur place, 300/130 suffit largement.

### Runner

```json
[
  { "id": 1, "desc": "G: SET_SPEED + ROTATE",
    "tasks": [
      { "type": "SPEED", "subtype": "SET_SPEED", "speed_percent": 30, "desc": "cap 30%" },
      { "type": "MOVEMENT", "subtype": "ROTATE_DEG", "angle_deg": 90, "desc": "rot +90 @30" },
      { "type": "MOVEMENT", "subtype": "ROTATE_DEG", "angle_deg": -90, "desc": "rot -90 @30 (revient)" },
      { "type": "SPEED", "subtype": "SET_SPEED", "speed_percent": 100, "desc": "restore" }
    ]
  }
]
```

### Indicateurs

| Comportement | Verdict |
|---|---|
| 1ʳᵉ rotation ≈ 2ᵉ rotation en durée (les 2 ralenties) | ✅ PASS — SET_SPEED ralentit aussi les rotations |
| Rotations à pleine vitesse | ❌ FAIL — `motion_setMaxSpeed` ignore l'angle (vérifier driver CBOR) |
| 2ᵉ rotation plus rapide que la 1ʳᵉ | ❌ FAIL — SET_SPEED s'efface après la 1ʳᵉ rotation (régression modif 2) |

> Note : dans le driver CBOR, le cmd `motion_setMaxSpeed` reçoit
> `speed_dist_percent` ET `speed_angle_percent`. Vérifier en lisant les logs
> `Asserv` que les 2 sont bien envoyés à 30 (et pas que dist).

---

## Tableau récapitulatif

| Test | Voie O_Nav | Voie Runner | Indicateur clé | Attendu |
|------|------------|-------------|----------------|---------|
| A    | `lr 0 1000 0 0 /+ 300 130 90 /p 1` | LINE | (x,y) final | identique |
| B    | `lr 0 1000 0 0 0 -1000 0 0 /+ 300 130 90 /p 1 /v 30` | SET_SPEED + 2 LINE | durée 1ᵉʳ = 2ᵉ LINE | identique |
| C    | `lr 0 1000 0 0 /+ 300 130 90 /p 1 /m 1` (proxy) | FACE_TO + LINE | biais latéral fin LINE | non-zéro sur arm |
| D    | (pas de pendant CLI) | FACE_TO chain + LINE | biais latéral fin LINE | quasi nul sur arm |
| E    | `lr 0 200 90 0 0 200 90 0 0 200 90 0 0 200 90 0 /+ 300 130 90 /p 1` | 4× LINE 200 + ROTATE 90 | dist retour départ | Runner > O_Nav (cumulé) |
| F    | (pas de pendant CLI) | 4× LINE 200 + ROTATE 90 chain | dist retour départ | < Test E |
| G    | `lr 90 0 0 0 -90 0 0 0 /+ 300 130 90 /p 1 /v 30` | SET_SPEED + 2 ROTATE | durée 1ʳᵉ = 2ᵉ rotation | identique |

## Build et exécution sur les 2 cibles

Les modifs (sleep 50 ms + flag `chain` + `*Send`) compilent sur les 2 cibles
utilisees pour les tests : **simu-debug** (iteration rapide en VM) et
**arm-release** (validation sur robot reel).

### Build

Depuis `robot/` :

```bash
# SIMU debug (native x86_64, AsservDriverSimu, le plus rapide pour iterer)
cmake --preset simu-debug && make -C build-simu-debug -j4 bot-opos6ul common-test

# ARM release (cross-compilation OPOS6UL, robot reel)
cmake --preset arm-release && make -C build-arm-release -j4 bot-opos6ul
```

### Recuperation des SVG

Apres execution, dans le dossier `bin/` :

```
svgAPF.svg       # trace des positions (rouge=cmd recue, bleu=traj voulue)
svgIA.svg        # trace IA (zones, paths)
svgSensors.svg   # trace capteurs ToF/balise
index.html       # ouvre les 3 svg superposes dans un browser
```

Sur le robot, recupere les fichiers via SSH/scp depuis `/var/log/pmx-robot/`
ou le dossier d'execution.

### Procedure de comparaison O_Nav vs Runner

1. **Build** la cible voulue.
2. Pour chaque test (A→F) :
   - Voie O_Nav : lancer la commande CLI. Sauvegarder `svgAPF.svg` sous
     `svg_<test>_onav_<cible>.svg` (ex: `svg_B_onav_simudebug.svg`).
   - Voie Runner : copier le JSON dans `bin/strategyPMX1.json`, lancer
     `./bot-opos6ul /s PMX1 /k`. Sauvegarder sous `svg_<test>_runner_<cible>.svg`.
3. Ouvrir les 2 SVG côte à côte dans un browser pour comparer.
4. Pour E et F, mesurer la distance finale au départ (lue dans les logs
   `pos_getX_mm()` / `pos_getY_mm()` en fin d'execution).

### Differences attendues entre cibles

- **simu-debug** : utilise `AsservDriverSimu` (cinematique ideale, pas de
  glissement, pas de buffer Rx). Le bug d'origine (derive residuelle apres
  IDLE) **ne se reproduit pas en simu** parce que le wrapper SIMU n'a pas
  le mecanisme `cmd_id` ack ni la fenetre d'arrivee Nucleo. La simu sert
  donc a valider :
  - que les modifs **n'ont pas casse** les tests existants (regression)
  - que les JSON de tests sont valides (parsing OK, runner execute jusqu'au
    bout, position finale coherente)
  - que SET_SPEED (Test B) persiste correctement (la modif 2 est testable
    en simu car elle est pur logique C++, pas de dependance Nucleo)
- **arm-release** (robot reel) : c'est la seule cible ou les modifs **chain
  et sleep 50 ms** apportent un gain visible. La derive d'asserv residuelle
  entre 2 cmds CBOR n'existe que sur la vraie Nucleo. Les comparaisons C
  vs D et E vs F ne sont pertinentes que sur cette cible.

Conclusion : on lance d'abord simu-debug (rapide, no-regression + Test B),
puis on deploye sur arm-release pour mesurer le gain reel des Tests C, D, E, F.
