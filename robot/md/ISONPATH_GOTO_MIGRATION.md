# Migration Goto externe + `isOnPath` — Détection d'obstacles prédictive

## Contexte

Aujourd'hui PMX-CORTEX décompose systématiquement `moveForwardTo(x, y)` côté OPOS6UL en `rotateAbsDeg(...) + line(dist)` (cf. `Asserv.cpp:628-683`). La carte asserv externe (asserv_chibios) n'exécute jamais de trajectoire atomique rotation+translation dans le flow actif — les primitives `Goto` / `GotoNoStop` existent côté carte (`CommandManager.h:45-46, 93-95`) mais **ne sont pas exposées** par le protocole CBOR (TODO dans `Opos6ulSerialCbor.cpp:9`).

L'objectif 2026 est d'**utiliser au maximum** les primitives `Goto` et `GotoNoStop` de la carte, pour bénéficier :

- des trajectoires lissées rotation+translation (meilleure vitesse moyenne)
- du chaînage de waypoints sans arrêt intermédiaire (`GotoNoStop`)
- d'un asservissement plus précis côté temps réel dur (carte ChibiOS)

Ce changement casse l'hypothèse actuelle selon laquelle la détection d'obstacle pouvait être gérée en distinguant les phases `MovementType::ROTATION` (détection ignorée) et `MovementType::LINE` (détection active). Pendant un `Goto`, le robot peut tourner **en avançant**, donc la distinction n'a plus de sens.

D'où l'introduction d'un prédicat `isOnPath` qui décide si un adversaire bloque réellement la trajectoire prévue, plutôt que de se reposer sur un cône géométrique fixe devant le robot.

## Sources de détection

Deux capteurs complémentaires alimentent déjà le même `DetectionEvent.frontLevel` via `Sensors.cpp` :

| Source | Nature | Force | Faiblesse |
|---|---|---|---|
| **Beacon triangulation** (x, y adv via balise Teensy) | Anticipation | Portée ~2 m, vue globale | Latence ~62 ms + erreur angulaire |
| **ToF c1-c8** (distances physiques directes) | Ground truth | Précis, immédiat (`frontLeft/frontRight/backLeft/backRight`) | Portée ~500 mm, **cônes étroits avant/arrière seulement**, pas de couverture latérale |

**Disposition physique des ToF** : 4 capteurs aux "épaules" avant (c1 AV gauche bas, c2 AV gauche haut, c3 AV droit bas, c4 AV droit haut) et 4 aux épaules arrière (c5 AR gauche bas, c6 AR gauche haut, c7 AR droit bas, c8 AR droit haut). Cônes orientés strictement vers l'avant ou l'arrière du robot. Rôle : vérifier qu'on peut **passer les épaules** sans accrocher un adversaire en translation.

Aujourd'hui seuls les capteurs HAUT sont exploités par `SensorsDriver::frontLeft/frontRight/backLeft/backRight` (c2, c4, c6, c8 — cf. `SensorsDriver.cpp:180-250`). Les capteurs BAS (c1, c3, c5, c7) sont lus mais non utilisés — probablement parce qu'ils voient le sol ou la base du robot. À clarifier avec le hardware avant activation.

**Important** : un adversaire **pile sur le côté** à 10 cm du robot n'est **pas** vu par les ToF (hors cônes). Seule la triangulation beacon peut le signaler, avec sa latence. Les deux sources sont donc réellement complémentaires — aucune n'est redondante avec l'autre.

La compensation de latence beacon via les timestamps `t1-t4_us` est **déjà en place** (`BeaconSensors.cpp:214-224` + `Sensors.cpp:836-840` avec `ARobotPositionShared::getPositionAt()`).

## Décisions architecturales validées

1. **`isOnPath` est un prédicat géométrique pur**, sans effet de bord. Il ne déclenche pas d'action, il renvoie un statut (CLEAR / APPROACHING / BLOCKING ou équivalent).

2. **La suppression de `rotate_ignoring_opponent`** est validée. Le flag ne sert plus dans le nouveau modèle (la rotation n'est plus une phase isolable pendant un Goto, et en ROTATION pure la détection est déjà ignorée par `MovementType`). Il traverse aujourd'hui toute la chaîne `IAbyPath → Asserv` et sera retiré en bloc.

3. **Les actions (slow / stop / recul / replanif) sont décidées par les appelants** de `isOnPath`, pas par le prédicat lui-même. Trois niveaux distincts :

| Niveau | Reçoit | Action |
|---|---|---|
| **Asserv `waitEndOfTrajWithDetection`** (pendant le move) | `APPROACHING` | `setMaxSpeed(reduced)` |
| | `BLOCKING` | `setEmergencyStop()` → `TRAJ_NEAR_OBSTACLE` |
| | `CLEAR` | `setMaxSpeed(false)` (restaure) |
| **Navigator `executeWithRetry`** (après un stop) | `TRAJ_NEAR_OBSTACLE` | Tempo + retry **même cible** (N fois), recul optionnel via `policy.reculObstacleMm` (déjà implémenté) |
| | Après N échecs | Rend la main au DecisionMaker avec code d'échec |
| **DecisionMaker / IA** (stratégie) | Échec Navigator | Choisit une **nouvelle cible**. Appelle `isOnPath` en **pré-flight** sur chaque candidate. Premier `CLEAR` gagne. Option : check historique adv (zone déjà occupée récemment ?) |

4. **La fréquence d'évaluation runtime** de `isOnPath` dans la boucle asserv n'est **pas** 10 ms. Elle est gatée sur changement :
   - Nouveau `seq` beacon (~62 ms) OU
   - Déplacement robot > ~30-50 mm depuis dernier check
   
   Charge CPU négligeable, logique géométrique simple (projection segment + distance).

5. **Les ToF c1-c8 restent un filet de sécurité indépendant** et toujours actifs (sauf en ROTATION pure du mode `DECOMPOSED` — cf. section suivante). Ils interviennent en dernier recours pour les cas où `isOnPath` serait incertain ou mal configuré.

6. **Politique de mouvement par waypoint (`MoveMode`)** : plutôt que d'essayer de deviner avec des heuristiques runtime (check ToF au démarrage, seuil d'angle de rotation, etc.) quand il faut décomposer un mouvement ou quand utiliser le Goto atomique, **c'est l'appelant (stratégie) qui choisit explicitement** le mode de chaque waypoint. Détails dans la section dédiée ci-dessous.

## Politique de mouvement par waypoint (`MoveMode`)

### Motivation

Le scénario critique est le **dégagement initial** : robot arrêté, adversaire pile devant dans le cône ToF, on veut démarrer une chaîne de mouvements vers une cible latérale ou arrière.

Avec un `Goto` atomique de la carte asserv, même la phase d'alignement initial mélange rotation et composante forward (la carte trace une trajectoire lissée). Le cône ToF avant balaie alors l'adversaire statique **et** le robot a une vitesse avant non nulle → `MovementType::GOTO` avec détection active → emergency stop dès le départ → impossible de démarrer.

Avec une décomposition explicite `rotate` pur puis `line` pur :
- La phase `rotate` est envoyée à la carte comme `MovementType::ROTATION` → détection ignorée par `waitEndOfTrajWithDetection` → le robot pivote sans friction, le cône avant balaie l'adv mais les alertes sont bypassées.
- Une fois réorienté, la phase `line` est envoyée comme `MovementType::LINE` → détection active → sécurité normale.

**Les heuristiques runtime pour décider automatiquement (check ToF au démarrage, angle de rotation > seuil) ne marchent pas** : pendant une rotation de 180° par exemple, l'adv peut être hors cône à t=0, apparaître à t=0.5s puis disparaître à t=1s. Un check initial ne voit rien et choisit à tort le Goto atomique, qui déclenche une alerte mid-rotation.

→ **C'est la stratégie qui sait** quand un dégagement est nécessaire (début de match, après un emergency stop, manœuvre tactique explicite), et c'est elle qui doit décider.

### API `MoveMode`

```cpp
enum class MoveMode {
    DECOMPOSED,    // rotate pur + line pur (bypass détection en rotation)
    GOTO,          // atomique, carte asserv (preciseGoto)
    GOTO_NO_STOP,  // atomique chaîné, carte asserv (addGoToNoStop)
};

struct Waypoint {
    float x_mm;
    float y_mm;
    MoveMode mode = MoveMode::DECOMPOSED;  // défaut pendant migration
};

TRAJ_STATE Navigator::executeChain(
    const std::vector<Waypoint>& waypoints,
    const RetryPolicy& policy
);
```

**Valeur par défaut** : `DECOMPOSED` **pendant les étapes 0-5** de la migration (garantit la non-régression, aucune dépendance CBOR). À basculer sur `GOTO` **après validation de l'étape 6** (bascule effective sur le Goto externe). Changement d'une ligne quand le moment viendra.

### Exemples d'usage côté stratégie

**Dégagement initial + course fluide + arrivée précise**
```cpp
nav->executeChain({
    {x1, y1, MoveMode::DECOMPOSED},    // dégagement : rotate sous bypass + line
    {x2, y2, MoveMode::GOTO_NO_STOP},
    {x3, y3, MoveMode::GOTO_NO_STOP},
    {x4, y4, MoveMode::GOTO},          // arrivée précise avec arrêt
}, policy);
```

**Route entièrement libre (pas de dégagement)**
```cpp
nav->executeChain({
    {x1, y1, MoveMode::GOTO_NO_STOP},
    {x2, y2, MoveMode::GOTO_NO_STOP},
    {x3, y3, MoveMode::GOTO},
}, policy);
```

**Sécurité maximale (ancien comportement, tout décomposé)**
```cpp
nav->executeChain({
    {x1, y1, MoveMode::DECOMPOSED},
    {x2, y2, MoveMode::DECOMPOSED},
    {x3, y3, MoveMode::DECOMPOSED},
}, policy);
```

### Règles de composition

- Plusieurs `GOTO_NO_STOP` consécutifs sont **enfilés sur la carte** en une seule file d'attente (continuité fluide, pas d'arrêt intermédiaire).
- Un `DECOMPOSED` inséré au milieu d'une chaîne `GOTO_NO_STOP` **casse la continuité** : la carte s'arrête au point précédent, on exécute le rotate+line, puis on ré-enfile la suite. C'est assumé — le prix pour avoir un dégagement intermédiaire explicite.
- Un `GOTO` en fin de chaîne garantit un arrêt précis à la cible finale (utile avant une action de prise / dépose).
- Mixer `DECOMPOSED` et `GOTO_NO_STOP` est permis mais à utiliser avec parcimonie pour ne pas perdre le bénéfice du lissage.

### Mapping `MoveMode` → `MovementType` → policy de détection

| `MoveMode` waypoint | Phases envoyées à la carte | `MovementType` par phase | Détection active |
|---|---|---|---|
| `DECOMPOSED` | 1. rotate pur | `ROTATION` | **Rien** (bypass) |
|  | 2. line pur | `LINE` (ou `FORWARD`/`BACKWARD` selon signe) | ToF + `isOnPath` |
| `GOTO` | 1. goto atomique | `GOTO` | ToF + `isOnPath` |
| `GOTO_NO_STOP` | 1. goto chaîné | `GOTO_NO_STOP` | ToF + `isOnPath` |

Le bypass détection total est **uniquement** dans la phase rotate du mode `DECOMPOSED`. Partout ailleurs, ToF et `isOnPath` sont actifs.

## Modèle runtime dans `waitEndOfTrajWithDetection`

```
tant que le move n'est pas terminé:
    lire DetectionEvent (publié par SensorsThread)

    [1] ToF direct — filet de sécurité
        si frontLevel == 4 (ToF très près)  → setEmergencyStop, TRAJ_NEAR_OBSTACLE

    [2] isOnPath — anticipation prédictive
        si seq_beacon a changé OU robot a bougé > 30 mm:
            status = isOnPath(pos_robot, cible, adv, width, margins)
            switch status:
                BLOCKING     → setEmergencyStop, TRAJ_NEAR_OBSTACLE
                APPROACHING  → setMaxSpeed(reduced)
                CLEAR        → setMaxSpeed(false)

    sleep 10 ms
```

## Spécification préliminaire de `isOnPath`

À **affiner avant implémentation**. Pistes :

```cpp
enum PathStatus { CLEAR, APPROACHING, BLOCKING };

PathStatus ObstacleZone::isOnPath(
    float x_robot, float y_robot,     // position robot actuelle
    float x_target, float y_target,   // cible du move courant
    const std::vector<AdvPos>& advs,  // adversaires détectés
    float corridor_width_mm,          // largeur du couloir (robot + marge + adv)
    float slow_distance_mm,           // distance à l'adv → slow
    float stop_distance_mm            // distance à l'adv → stop
) const;
```

Géométrie :

- Segment `[robot → cible]`.
- Pour chaque adversaire : projection orthogonale sur le segment (`t ∈ [0, 1]` clampé).
- Distance perpendiculaire entre adv et projection.
- Si **hors du couloir** (distance perpendiculaire > `width/2`) ET **hors de la zone d'arrivée** → `CLEAR`.
- Si dans le couloir **et** distance le long du segment depuis le robot < `stop_distance_mm` → `BLOCKING`.
- Si dans le couloir **et** distance < `slow_distance_mm` → `APPROACHING`.
- Sinon `CLEAR`.

Cas particuliers à traiter dans les tests :

- Adversaire **derrière** le robot (projection `t < 0`) → ignoré.
- Adversaire **au-delà** de la cible (`t > 1`) → ignoré pour le move courant, mais pertinent pour le pré-check de la cible suivante dans une chaîne.
- Couloir **élargi dans la zone de virage** (Goto lissé) ou simplification en segment droit → à trancher.
- Adversaire **mobile** : version v1 = position instantanée. Version future = extrapolation linéaire courte.

## Historique adversaire (optionnel, v2)

Buffer circulaire dans `Sensors` qui log `{x, y, t}` à chaque `DetectionEvent`. Méthode :

```cpp
bool Sensors::wasAdvSeenNear(float x, float y,
                              float radius_mm, uint32_t depuis_ms) const;
```

Utilisable par le DecisionMaker comme **deuxième filtre de pré-flight** sur les cibles candidates. À activer par ordre de stratégie (pas systématique).

## Plan de migration ordonné

### Étape 0 — Design & tests (ne touche pas le robot)

- Figer la signature exacte de `isOnPath` et ses seuils
- Rédiger `test/common/IsOnPathTest.cpp` **avant** l'implémentation (TDD)
- Cas de test : clear, approaching, blocking, adv derrière, adv après cible, couloir latéral

### Étape 1 — Prédicat `isOnPath` isolé

- Implémenter `ObstacleZone::isOnPath(...)` (géométrie pure)
- Faire passer `IsOnPathTest` en vert
- **Aucun câblage** avec l'asserv ou la stratégie à ce stade

### Étape 2 — Suppression de `rotate_ignoring_opponent`

- Retirer le paramètre de toute la chaîne `IAbyPath → Asserv`
  - `IAbyPath.cpp:374, 471, 575, 651, 667, 671, 776, 788, 792, 931, 935` (et variations)
  - `Asserv.cpp:628, 684, 652-676, 704-708, 719`
  - `Asserv.hpp:542` et équivalents
- Supprimer le bloc de rattrapage "reset emergency + continue line" (`Asserv.cpp:652-676`)
- Vérifier que les appels DecisionMaker / stratégies compilent (chercher call-sites)
- Tests de régression : SIMU + robot à vide

### Étape 3 — Activation ToF c1-c4 / c5-c8

- Dans la config `ObstacleZone` (ou `HARDWARE_CONFIG`) : `ignoreFrontLeft/Right = false`, `ignoreBackLeft/Right = false`
- Régler `frontLeftThreshold` / `frontLeftVeryClosedThreshold` (niveaux 3 et 4)
- Valider sur robot avec un obstacle immobile
- `SENSORS_DETECTION_MIGRATION.md` à mettre à jour (cette étape était listée "Phase 5")

### Étape 4 — Intégration `isOnPath` dans `waitEndOfTrajWithDetection`

- Ajouter le check prédictif gate sur `seq` beacon et delta position robot
- Garder le check ToF `frontLevel == 4` comme filet de sécurité
- Pour l'instant uniquement pour les `MovementType::LINE` / `FORWARD` / `BACKWARD` (types existants)
- Tests : SIMU avec `SensorsDriverSimu` qui injecte des positions adversaire

### Étape 5 — Extension CBOR pour `Goto` et `GotoNoStop`

- Étendre `Opos6ulSerialCbor` (côté asserv_chibios) pour exposer :
  - `goto(x, y)` (preciseGotoConfiguration)
  - `gotoNoStop(x, y)` (addGoToNoStop)
  - `clearQueue` (pour interrompre une chaîne)
  - éventuellement `goto_reverse`, `goto_reverse_nostop`
- Côté CORTEX : ajouter `Asserv::gotoExt(x, y)` et `Asserv::gotoNoStop(x, y)`
- Ajouter `MovementType::GOTO` et `MovementType::GOTO_NO_STOP`
- **Chantier lourd** : à discuter avec la partie asserv_chibios, peut être fait en parallèle

### Étape 6 — API `Navigator::executeChain` avec `MoveMode` par waypoint

- Définir `enum class MoveMode { DECOMPOSED, GOTO, GOTO_NO_STOP }` et `struct Waypoint { x, y, mode = DECOMPOSED }`
- Implémenter `Navigator::executeChain(waypoints, policy)` qui dispatche par `MoveMode` :
  - `DECOMPOSED` → `rotateAbsDeg` + `line` (comportement actuel)
  - `GOTO` → `asserv->gotoExt(x, y)` (nécessite étape 5)
  - `GOTO_NO_STOP` → `asserv->gotoNoStop(x, y)` avec enfilage (nécessite étape 5)
- Étendre `waitEndOfTrajWithDetection` pour gérer `MovementType::GOTO` et `GOTO_NO_STOP` avec la policy prédictive (`isOnPath` + ToF, plus de distinction ROTATION/LINE pendant un Goto)
- Garder l'ancien `moveForwardTo` en fallback pendant la migration pour les anciens call-sites
- Défaut du `MoveMode` dans la struct = `DECOMPOSED` pendant migration, à basculer sur `GOTO` une fois étape validée sur robot
- Tests sur table réelle avec adversaire mobile

### Étape 7 — `isOnPath` en pré-flight DecisionMaker

- Exposer `isOnPath` via `Robot` ou un helper accessible depuis `IAbyPath` / `DecisionMaker`
- Ajouter une option dans les ordres de stratégie : "vérifier viabilité avant de lancer"
- Si non viable, la stratégie choisit une alternative

### Étape 8 — Historique adversaire (optionnel)

- Buffer circulaire dans `Sensors` + `wasAdvSeenNear(...)`
- Option d'ordre de stratégie : "éviter les zones récemment occupées"

## Checklist à démarrer demain

Ordre recommandé, chaque item est un commit séparé :

- [ ] **T1** — Relire ce doc, valider les seuils (`corridor_width_mm`, `slow_distance_mm`, `stop_distance_mm`) avec une estimation concrète des vitesses 2026
- [ ] **T2** — Écrire `test/common/IsOnPathTest.cpp` (TDD, tests en rouge)
- [ ] **T3** — Implémenter `ObstacleZone::isOnPath(...)` — faire passer T2 en vert
- [ ] **T4** — Lister précisément les call-sites de `rotate_ignoring_opponent` (grep) et préparer le patch de suppression en un seul commit
- [ ] **T5** — Supprimer `rotate_ignoring_opponent` (IAbyPath + Asserv + call-sites), tests SIMU
- [ ] **T6** — Ouvrir un ticket / note pour la config ObstacleZone : activer les ToF c1-c4 / c5-c8 sur robot (à faire quand hardware prêt)
- [ ] **T7** — Ouvrir un ticket / note pour le chantier CBOR Goto/GotoNoStop (asserv_chibios) à discuter avec l'équipe
- [ ] **T8** — Après T5 vert : intégrer `isOnPath` dans `waitEndOfTrajWithDetection` (version runtime, gatée sur seq beacon)

**Ne PAS faire demain** : le chantier CBOR (étape 5), l'utilisation effective de `Goto` (étape 6), l'historique adv (étape 8). Ce sont des chantiers à lancer après stabilisation de T1-T8.

## Risques et points ouverts

- **Réglage des seuils** : largeur couloir, distance slow, distance stop — à valider empiriquement sur table
- **Trajectoire lissée vs segment droit** : le Goto externe ne trace pas un segment droit. Accepter l'approximation dans v1 (couloir autour du segment `[robot → cible]`), évaluer le besoin d'une version plus fidèle en v2
- **Adversaire mobile rapide** : v1 utilise la position instantanée. Si faux négatifs trop fréquents, envisager l'extrapolation à partir du delta de séquence beacon
- **Interruption d'une chaîne GotoNoStop** : vérifier que `clearQueue` est bien exposé côté carte asserv_chibios avant de se lancer sur `GOTO_NO_STOP`. Sans ça, impossible de replanifier au milieu d'une chaîne.
- **Faux positifs ToF pendant `GOTO` atomique** : le cône avant peut balayer un adversaire statique pendant la phase d'alignement du Goto. Sans bypass, on déclenche un stop intempestif. La parade est côté stratégie via `MoveMode::DECOMPOSED` sur le premier waypoint quand un dégagement est nécessaire (cf. section "Politique de mouvement par waypoint"). Si ce contournement ne suffit pas en pratique, envisager une v2 qui désactive temporairement les ToF pendant un `GOTO` quand la composante angulaire est dominante.
- **Coexistence avec la décomposition actuelle** : pendant la migration, garder les deux modèles côte à côte (le défaut `MoveMode = DECOMPOSED` assure que les call-sites non migrés continuent de marcher) pour pouvoir rollback.
- **Capteurs ToF BAS non utilisés** : clarifier avec le hardware pourquoi seuls c2/c4/c6/c8 (HAUT) alimentent `frontLeft/frontRight/backLeft/backRight`. Peut-être une opportunité d'exploiter c1/c3/c5/c7 en v2 pour élargir la couverture.
