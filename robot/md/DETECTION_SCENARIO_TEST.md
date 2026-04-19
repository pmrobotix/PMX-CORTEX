# `O_DetectionScenarioTest` — Plan de validation detection adversaire

Document de specification pour le test de scenarios SIMU qui valide le comportement du robot face a un adversaire plante, depuis la detection jusqu'a la decision strategique.

Objectif : **etablir un baseline comportemental avant T8** (integration `isOnPath` runtime), puis **mesurer le gain** apres T8.

---

## 1. Nature du test

**Test de scenario SIMU integre**, pattern `O_*Test` (comme [O_NavigatorMovementTest.cpp](../src/bot-opos6ul/O_NavigatorMovementTest.cpp)).

- Fichiers : `src/bot-opos6ul/O_DetectionScenarioTest.cpp` + `.hpp`
- Compile dans le binaire `bot-opos6ul`
- Active par CLI : `./bot-opos6ul /detection` (ou `/detection-a`, `/detection-b`, `/detection-c`)
- Utilise tout le stack SIMU :
  - `AsservDriverSimu` (mouvements)
  - `SensorsDriverSimu` (capteurs + injection persistante)
  - `SensorsThread` (ticks 62ms)
  - `Navigator` (retry + pathfinding)
  - `Asserv::waitEndOfTrajWithDetection` (boucle de decision)

Pas un test unitaire `common-test` — on a besoin du vrai cycle asserv/sensors.

## 2. Infrastructure utilisee

### 2.1 Injection d'adversaire persistante (fait)

Ajoute dans `ASensorsDriver` + `SensorsDriverSimu` + wrapper `Sensors` :

```cpp
robot.actions().sensors().setInjectedAdv(x_table_mm, y_table_mm);
// ... mouvement du robot, detection active ...
robot.actions().sensors().clearInjectedAdv();
```

Contrairement a `addvPositionsAdv` (ephemere, 1 tick), `setInjectedAdv` positionne l'adv tant que `clearInjectedAdv()` n'est pas appele. Le `sync()` republie la position a chaque tick 62ms.

No-op sur drivers ARM (l'adv vient de la balise physique).

Tests unitaires : `test/driver/SensorsDriverTest.cpp` (3 tests : persistance, override, clear).

### 2.2 Invocation CLI

```
./bot-opos6ul /detection          # tout (A + B + C)
./bot-opos6ul /detection-a        # niveau A seul (rapide, iteration)
./bot-opos6ul /detection-b        # niveau B (retry)
./bot-opos6ul /detection-c        # niveau C (strategie)
```

### 2.3 Sorties

- **Log ASCII** dans la console (pattern O_NavigatorMovementTest)
- **SVG de synthese** a cote du binaire : `build-simu-debug/bin/detection_scenarios.svg`
  - Grille verticale : niveau A en haut, B au milieu, C en bas
  - Chaque cellule : robot + cible + adv + trajectoire reelle + couloir isOnPath + label `[PASS]/[FAIL]`

### 2.4 Duree estimee

- Niveau A : ~30s (8 scenarios × ~3s)
- Niveau B : ~60s (4 scenarios × ~15s avec retry 2× + tempo 2s)
- Niveau C : ~60s (4 scenarios × ~3 actions)

Total : ~2-3 minutes.

## 3. Structure d'un scenario

```cpp
struct Scenario {
    const char* name;
    float       startX, startY, startThetaDeg;
    float       targetX, targetY;
    float       advX, advY;          // NAN = pas d'adv
    enum Move   { GOTO, MOVE_FORWARD_TO, LINE_FWD, ROTATE_DEG_90, GO_BACK_TO };
    Move        moveType;
    TRAJ_STATE  expected;
};
```

### Deroule d'un scenario

```cpp
// 1. Reset robot
robot.asserv().setPositionAndColor(sc.startX, sc.startY, sc.startThetaDeg, false);

// 2. Injection adv
robot.actions().sensors().clearInjectedAdv();
if (!std::isnan(sc.advX))
    robot.actions().sensors().setInjectedAdv(sc.advX, sc.advY);

// 3. Demarrer le sensorsThread (62ms tick)
robot.actions().sensors().startSensorsThread(20);
utils::sleep_for_micros(100000);  // 100ms stabilisation

// 4. Lancer le mouvement (timeout 10s)
chrono.start();
TRAJ_STATE actual = executerMouvement(sc);
if (chrono.getElapsedTimeInSec() > 10) actual = TRAJ_ERROR;

// 5. Relever position finale
ROBOTPOSITION pEnd = robot.asserv().pos_getPosition();

// 6. Log + SVG + assertion
recordScene(sc, actual, pEnd);
```

---

## 4. Les 3 niveaux de scenarios

Progression : tactique pure -> retry Navigator -> strategie IA.

### Niveau A — Tactique pure (1 mouvement, 1 adv fixe)

**Objectif** : valider que les primitives de mouvement (goTo, moveForwardTo, line, rotateDeg, goBackTo) reagissent correctement au niveau 2 (detection ToF + cone FORWARD actuel).

**Ce qui est teste** : Asserv + DetectionEvent + waitEndOfTrajWithDetection, **sans retry** (policy noRetry).

**Scenarios** :

| # | Nom | Start | Cible | Adv | Mouvement | Attendu aujourd'hui | Attendu apres T8 |
|---|---|---|---|---|---|---|---|
| A1 | `GoToClearPath` | (200,200) 0° | (800,200) | - | `goTo` | FINISHED | FINISHED |
| A2 | `GoToAdvOnPath` | (200,200) 0° | (800,200) | (500,200) | `goTo` | NEAR_OBSTACLE | NEAR_OBSTACLE |
| A3 | `GoToAdvBeside` | (200,200) 0° | (800,200) | (500,600) | `goTo` | **⚠ peut stopper (cone)** | **FINISHED (isOnPath CLEAR)** |
| A4 | `GoToAdvBehindTarget` | (200,200) 0° | (800,200) | (1100,200) | `goTo` | FINISHED | FINISHED |
| A5 | `RotateAdvInFront` | (200,200) 0° | - | (500,200) | `rotateDeg(90)` | FINISHED (ROTATION bypass) | FINISHED |
| A6 | `MoveForwardToAdvLateral` | (200,200) 0° | (800,800) | (400,600) | `moveForwardTo` | **⚠ peut stopper** | **FINISHED** |
| A7 | `GoBackToAdvBehind` | (800,200) 180° | (200,200) | (500,200) | `goBackTo` | NEAR_OBSTACLE | NEAR_OBSTACLE |
| A8 | `GoBackToAdvBeside` | (800,200) 180° | (200,200) | (500,600) | `goBackTo` | **⚠ peut stopper** | **FINISHED** |

**Scenarios critiques T8** : A3, A6, A8 (adv lateral dans le cone → STOP intempestif aujourd'hui, devraient passer apres T8 grace a `isOnPath`).

### Niveau B — Retry Navigator (1 mouvement, adv plante, retry)

**Objectif** : valider la boucle retry du Navigator (niveau 3) apres un TRAJ_NEAR_OBSTACLE.

**Ce qui est teste** :
- `Navigator::executeWithRetry` avec policy standard (2 retries, tempo 2s)
- Optionnellement avec recul (`reculObstacleMm > 0`)
- Abandon propre apres max retries

**Scenarios** :

| # | Nom | Setup | Policy | Attendu |
|---|---|---|---|---|
| B1 | `RetryAdvStaysBlocked` | Robot (200,200), cible (800,200), adv (500,200) plante | `standard` (2 retries) | NEAR_OBSTACLE apres 2 essais + tempo 2s × 2 ≈ 4s |
| B2 | `RetryAdvDisappears` | Robot (200,200), cible (800,200), adv (500,200). Apres 3s le test `clearInjectedAdv` | `standard` | FINISHED au retry 2 (adv parti) |
| B3 | `RetryAggressiveWithRecul` | Robot (200,200), cible (800,200), adv (500,200) plante | `aggressive` (5 retries, recul 50mm) | NEAR_OBSTACLE + robot recule de 50mm × 5 entre essais |
| B4 | `RetryCollisionAfterStop` | Robot (200,200), cible (800,200), adv (500,200) plante | `standard` | Verifier que l'emergency stop est bien reset entre retries |

**Critere de succes** : le robot n'est jamais "bloque" indefiniment. Apres max retries, la main est rendue au niveau 4 (IA) avec un code d'echec net.

**Peu sensible a T8** (le retry marche deja, T8 ameliore quand l'adv bouge latmeralement).

### Niveau C — Mini-strategie IA (N actions prioritaires, skip si bloquee)

**Objectif** : valider le comportement "le robot n'est pas bloque par un adv plante, il passe a l'action suivante".

**Ce qui est teste** : boucle IA simplifiee + Navigator + Asserv + Sensors bout-en-bout. Approche **le comportement reel en match**.

**Mini-IA du test** (~15 lignes dans le test) :

```cpp
struct StrategyAction {
    const char* name;
    float zoneX, zoneY;
    int points;
    TRAJ_STATE result = TRAJ_IDLE;
};

std::vector<StrategyAction> actions = { /* ... */ };

for (auto& a : actions) {
    TRAJ_STATE ts = nav.pathTo(a.zoneX, a.zoneY);
    a.result = ts;
    if (ts == TRAJ_FINISHED) {
        scorePoints += a.points;
        actionsOK++;
    } else {
        robot.asserv().resetEmergencyOnTraj("skip action, passage suivant");
        actionsSkipped++;
    }
}

// Optionnel : 2e passe pour retenter les actions skippees (l'adv a peut-etre bouge)
for (auto& a : actions) {
    if (a.result == TRAJ_FINISHED) continue;
    TRAJ_STATE ts = nav.pathTo(a.zoneX, a.zoneY);
    if (ts == TRAJ_FINISHED) {
        scorePoints += a.points;
        actionsRecovered++;
    }
}
```

**Scenarios** :

| # | Nom | 3 zones | Adv(s) plantes | Attendu |
|---|---|---|---|---|
| C1 | `NoAdv` | ZoneA (500,400), ZoneB (1500,400), ZoneC (2500,400) | - | 3/3 actions, 45 pts |
| C2 | `AdvBlocksZoneA` | idem | (400,400) | 2/3 (B+C), 35 pts, A skippee |
| C3 | `AdvBlocksTwo` | idem | (400,400) + (1400,400) | 1/3 (C), 20 pts |
| C4 | `AdvDisappears` | idem | (400,400) plante 0-5s, puis clear | 3/3 si 2e passe activee (A recuperee) |

**Scoring** par scenario : `actions_ok / actions_total`, points gagnes, duree totale, nb actions recuperees en 2e passe.

**Sensible a T8** : les adversaires sont positionnes de facon a bloquer le chemin direct. Avec le cone actuel, un adv juste a cote d'une zone peut la "planter" injustement. Avec `isOnPath`, seul un adv sur le segment `[robot -> zone]` bloque.

---

## 5. SVG de synthese

Fichier : `build-simu-debug/bin/detection_scenarios.svg`

Grille verticale, 1 colonne, 1 cellule par scenario :

```
+-----------------------------------+
|  Niveau A — Tactique pure         |
+-----------------------------------+
|  A1 GoToClearPath                 |
|  [robot]======>[cible]  CLEAR     |
|  expected=FINISHED actual=F [PASS]|
+-----------------------------------+
|  A2 GoToAdvOnPath                 |
|  [robot]==X   (adv)  [cible]      |
|  expected=NEAR_OBSTACLE [PASS]    |
+-----------------------------------+
...
+-----------------------------------+
|  Niveau B — Retry Navigator       |
+-----------------------------------+
...
+-----------------------------------+
|  Niveau C — Strategie IA          |
+-----------------------------------+
|  C1 NoAdv                         |
|  [robot]-->ZoneA-->ZoneB-->ZoneC  |
|  actions=3/3  score=45  [PASS]    |
+-----------------------------------+
```

Chaque cellule contient :
- Table (fond gris clair)
- Position depart robot (cercle bleu 280mm + petit disque plein)
- Cible (croix bleue)
- Adv injecte (cercle rouge 400mm + disque plein)
- Trajectoire reelle (ligne bleue du start au point final)
- Couloir isOnPath (rectangle avec zones STOP/SLOW en rouge/orange) — visualise ce que T8 verra
- Label : expected / actual / PASS/FAIL en couleur

---

## 6. Comparaison avant / apres T8

### Workflow de validation

```
1. Ecrire O_DetectionScenarioTest avec A + B + C
2. Lancer -> baseline (probablement A3, A6, A8 echouent)
3. Sauvegarder detection_scenarios.svg comme "baseline_before_T8.svg"
4. Implementer T8 (isOnPath runtime + MovementType::GOTO / GOTO_NO_STOP)
5. Relancer -> les scenarios critiques passent au vert
6. Sauvegarder detection_scenarios.svg comme "after_T8.svg"
7. Comparer visuellement -> preuve du gain
```

### Tableau recapitulatif attendu

| Niveau | Scenarios totaux | Attendu avant T8 | Attendu apres T8 |
|---|---|---|---|
| A | 8 | 5 PASS / 3 FAIL (A3, A6, A8) | 8 PASS |
| B | 4 | 4 PASS (retry marche deja) | 4 PASS |
| C | 4 | 3 PASS / 1 FAIL en partie (C2 skippe trop d'actions) | 4 PASS (C2 garde toutes les actions viables) |
| **Total** | **16** | **12 PASS / 4 FAIL** | **16 PASS** |

---

## 7. Ordre d'implementation

1. ✅ **Infrastructure injection** : `setInjectedAdv` / `clearInjectedAdv` dans `ASensorsDriver` + `SensorsDriverSimu` + wrapper `Sensors` + 3 tests unitaires driver-test
2. **Niveau A** (~400 lignes) : ecrire + lancer -> baseline tactique
3. **Niveau B** (~200 lignes) : ecrire + lancer -> valide retry
4. **Niveau C** (~250 lignes) : ecrire + lancer -> valide strategie
5. Commit baseline avant T8
6. Implementer **T8** (isOnPath runtime)
7. Relancer O_DetectionScenarioTest -> tout au vert
8. Commit apres T8 avec comparaison SVG avant/apres

Chaque niveau dans une methode separee du meme test pour pouvoir les lancer independamment.

## 8. Fichiers a creer / modifier

| Fichier | Action | Lignes approx |
|---|---|---|
| `src/common/interface/ASensorsDriver.hpp` | Ajout 2 virtuels no-op | +15 |
| `src/driver-simu/SensorsDriver.hpp` | Champs + override 2 methodes | +10 |
| `src/driver-simu/SensorsDriver.cpp` | Impl + sync() modifie | +15 |
| `src/common/action/Sensors.hpp` | Wrapper 2 methodes | +20 |
| `test/driver/SensorsDriverTest.hpp/cpp` | 3 tests injection | +50 |
| `src/bot-opos6ul/O_DetectionScenarioTest.hpp` | Declarations + Scenario struct | ~80 |
| `src/bot-opos6ul/O_DetectionScenarioTest.cpp` | Impl 3 niveaux + SVG | ~800 |
| `src/bot-opos6ul/OPOS6UL_RobotExtended.cpp` | Cable switch CLI | +5 |
| `CMakeLists.txt` | Ajout fichier test | +1 |

## 9. Evolution post-T8

Une fois T8 stable, enrichir le test :

- **Adv mobile** : thread de test qui deplace l'adv au cours du mouvement (ex: traverse de gauche a droite)
- **Adv qui disparait puis reapparait** : simuler des cas de perte/retour balise
- **Scenarios multi-adv** : jusqu'a 4 adv simultanes (limite de la balise Teensy)
- **Scenarios nominaux match** : reproduire les actions de strategie reelle (ex: `O_push_prise_bas` avec adv plante a proximite)

Ces evolutions ne necessitent que d'enrichir le test, pas de toucher a l'infra.
