# Migration Détection d'obstacles — Sensors / ObstacleZone / DetectionEvent

## Contexte

Le système de détection d'obstacles gère la sécurité anti-collision pendant les déplacements du robot.
Il utilise une balise Teensy (18 capteurs ToF VL53L0X) communiquant en I2C avec l'OPOS6UL.

### Problèmes identifiés (ancien PMX)

1. **SensorsTimer mélange lecture / filtrage / décision / exécution** dans un seul callback
2. **L'IA ne sait pas où est l'adversaire** — elle reçoit `TRAJ_NEAR_OBSTACLE` sans coordonnées
3. **Erreur de synchronisation ~120mm** — la position adversaire est calculée avec la position robot actuelle, pas celle du moment de la mesure beacon
4. **Pas de simulation adversaire en SIMU** — impossible de tester la détection sans hardware
5. **Pas de visualisation SVG des capteurs L/R** — seule la position beacon centre est tracée
6. **Reprise bloquée** — si l'adversaire est "devant" mais que la nouvelle trajectoire l'évite, le robot reste stoppé

### Ce qui a déjà été fait

| Étape | Statut | Description |
|---|---|---|
| Extraction `TableGeometry` | ✅ | `filtre_IsInsideTable*` sorti de Asserv → `common/geometry/TableGeometry.hpp` |
| Extraction `ObstacleZone` | ✅ | Logique pure (filtres level, config, seuils) sortie de Sensors → `common/geometry/ObstacleZone.hpp/.cpp` |
| `TableGeometryTest` | ✅ | 12 assertions, `test/common/` (stub position local) |
| `ObstacleZoneTest` | ✅ | 12 assertions, `test/common/` (logique pure, 0 dépendance) |
| `SensorsDriverTest` | ✅ | 9 assertions, `test/driver/` (contrat interface ARM+SIMU) |
| Fix ARM `backLeft`/`backRight` | ✅ | Implémenté (c6_mm, c8_mm), était stub `-1` → crash |
| Fix ARM `rightSide`/`leftSide` | ✅ | Ajouté `return 400` (était sans return → undefined behavior) |
| Fix ARM `clearPositionsAdv` | ✅ | Ajouté `vadv_.clear()` (était vide) |
| Sensors délègue vers ObstacleZone | ✅ | Config/seuils/filtres inline → `obstacleZone_` |

## Architecture actuelle

```
          IA / DecisionMaker
              │
              │ TRAJ_STATE (FINISHED, NEAR_OBSTACLE, COLLISION...)
              ▼
          Navigator
              │ executeWithRetry() — retry + recul
              ▼
          Asserv
              │ warnFrontDetectionOnTraj(level, x, y)
              │   level 3 → setMaxSpeed(reduced)
              │   level 4 → setEmergencyStop()
              ▲
              │
    SensorsTimer::onTimer() [~62ms]
              │
              ├── sync("beacon_sync") → SensorsDriver → BeaconSensors I2C
              ├── front(true) → filtre + classification
              │     ├── fL, fR (ToF L/R via c2_mm, c4_mm)
              │     ├── beacon center (x, y, d, theta) → filtre_levelInFront
              │     └── SVG: writePosition_AdvPos()
              ├── back(true) → idem arrière
              └── debounce (nb_ensurefront4 >= 2 → warn)
```

### Couches actuelles

```
ObstacleZone         logique pure : filtre_levelInFront/Back, config, seuils
TableGeometry        logique pure : isPointInsideTable, isSensorReadingInsideTable
Sensors              orchestration : sync + front/back + timer + délègue vers ObstacleZone
SensorsDriver ARM    hardware : BeaconSensors I2C (c1-c8, x1-x4, y1-y4, d1-d4)
SensorsDriver SIMU   stub : retourne 999 (pas d'obstacle), vadv_ vide
```

## Architecture cible

### Principe fondamental : un seul décideur par niveau

```
SensorsTimer         LECTURE + FILTRAGE uniquement, aucun side-effect
                     publie DetectionEvent (level, x_adv, y_adv, timestamp)
                     N'APPELLE JAMAIS l'Asserv
                         │
                         ▼ DetectionEvent (variable partagée)
                         
Asserv               EXÉCUTION + CONSULTATION du DetectionEvent
                     waitEndOfTrajWithDetection(MovementType)
                     boucle 1ms : lit DetectionEvent + status driver
                     LUI décide : ralentir / stopper / continuer
                         │
                         ▼ TRAJ_STATE (FINISHED, COLLISION, NEAR_OBSTACLE)
                         
Navigator            RETRY automatique selon RetryPolicy
                     executeWithRetry() — 2 retries standard
                     en cas d'échec retourne TRAJ_STATE à l'IA
                         │
                         ▼ TRAJ_STATE + lastDetection() disponible
                         
DecisionMaker        STRATÉGIE — décide contournement, attente, abandon
                     lit lastDetection() pour savoir OÙ est l'adversaire
```

### Boucle centrale : waitEndOfTrajWithDetection()

Remplace `waitEndOfTraj()` + `warnFrontDetectionOnTraj()` + tous les flags `temp_*`.

```cpp
enum MovementType { ROTATION, FORWARD, BACKWARD };

TRAJ_STATE Asserv::waitEndOfTrajWithDetection(MovementType type)
{
    while (true) {
        // 1. Status hardware (driver : EsialR ou CborDriver)
        int status = pollDriverStatus();
        if (status == IDLE && queueEmpty)  return TRAJ_FINISHED;
        if (status == BLOCKED)             return TRAJ_COLLISION;

        // 2. Détection obstacles (publié par SensorsTimer toutes les 62ms)
        const DetectionEvent& det = sensors_->lastDetection();

        if (type == ROTATION) {
            // En rotation : on ignore la détection
            // L'adversaire est peut-être "devant" mais on tourne sur place
        }
        else if (type == FORWARD) {
            if (det.frontLevel == 4) {
                emergencyStop();
                sensors_->setStopDetection(det);  // fige pour Navigator/IA
                return TRAJ_NEAR_OBSTACLE;
            }
            if (det.frontLevel >= 3)  setMaxSpeed(reduced);
            else                      setMaxSpeed(normal);
        }
        else if (type == BACKWARD) {
            if (det.backLevel == -4) {
                emergencyStop();
                sensors_->setStopDetection(det);  // fige pour Navigator/IA
                return TRAJ_NEAR_OBSTACLE;
            }
            if (det.backLevel <= -3)  setMaxSpeed(reduced);
            else                      setMaxSpeed(normal);
        }

        sleep(1ms);
    }
}
```

### Deux DetectionEvents : temps réel vs historique

```
lastDetection_     écrasé toutes les 62ms par SensorsTimer
                   lu par waitEndOfTrajWithDetection() à 1ms
                   → décision temps réel : stopper / ralentir / continuer

stopDetection_     figé au moment du STOP par waitEndOfTrajWithDetection()
                   lu par Navigator / DecisionMaker après le STOP
                   → décision stratégique : retry / contourner / abandonner
```

Comparaison dans le DecisionMaker :

```cpp
DetectionEvent stop = sensors.stopDetection();  // où il ÉTAIT au moment du stop
DetectionEvent now  = sensors.lastDetection();  // où il EST maintenant

if (stop.hasPosition() && !now.isBlocking()) {
    // L'adversaire est parti → retry immédiat
}
if (stop.hasPosition() && now.isBlocking()) {
    // Toujours là → contourner ou attendre
}
```

Utilisation dans les mouvements :

```cpp
TRAJ_STATE Asserv::goTo(float x, float y) {
    driver->motion_FaceTo(x, y);
    ts = waitEndOfTrajWithDetection(ROTATION);    // ignore détection
    if (ts != TRAJ_FINISHED) return ts;

    driver->motion_Line(distance);
    ts = waitEndOfTrajWithDetection(FORWARD);     // détection front active
    return ts;
}

TRAJ_STATE Asserv::goBackTo(float x, float y) {
    driver->motion_FaceBackTo(x, y);
    ts = waitEndOfTrajWithDetection(ROTATION);    // ignore détection
    if (ts != TRAJ_FINISHED) return ts;

    driver->motion_Line(-distance);
    ts = waitEndOfTrajWithDetection(BACKWARD);    // détection back active
    return ts;
}
```

### Ce que ça supprime

| Supprimé | Remplacé par |
|---|---|
| `warnFrontDetectionOnTraj()` | `waitEndOfTrajWithDetection(FORWARD)` lit le DetectionEvent |
| `warnBackDetectionOnTraj()` | `waitEndOfTrajWithDetection(BACKWARD)` lit le DetectionEvent |
| `temp_ignoreFrontDetection_` | `MovementType::ROTATION` ignore naturellement |
| `temp_ignoreBackDetection_` | idem |
| `temp_forceRotation_` | idem |
| `pathStatus_` (EsialR) | Plus besoin — c'est Asserv qui set NEAR_OBSTACLE |
| Bloc décisionnel dans `SensorsTimer::onTimer()` | SensorsTimer ne fait que lire + publier |

### Scénario complet : adversaire bloquant puis contournement

```
1. DecisionMaker : navigator.goTo(500, 300)
   │
2. Asserv.goTo(500, 300)
   │ faceTo:  waitEndOfTrajWithDetection(ROTATION) → FINISHED
   │ line:    waitEndOfTrajWithDetection(FORWARD)
   │          DetectionEvent.frontLevel == 4 → emergencyStop → NEAR_OBSTACLE
   │
3. Navigator.executeWithRetry()
   │ retry 1 : sleep(2s) + reset + goTo(500,300)
   │   faceTo: FINISHED (ROTATION ignore l'adversaire "devant")
   │   line:   NEAR_OBSTACLE (adv toujours là)
   │ retry 2 : idem → NEAR_OBSTACLE
   │ max retries → retourne NEAR_OBSTACLE
   │
4. DecisionMaker lit lastDetection() → adv en (400, 350)
   │ décide contourner : navigator.pathTo(500, 300) via (800, 500)
   │
5. Asserv.goTo(800, 500)
   │ faceTo(800, 500): ROTATION → tourne 90° → FINISHED
   │                   (adv toujours en (400,350) mais on IGNORE en rotation)
   │ line(800, 500):   FORWARD → adv n'est plus devant → FINISHED
   │
6. Asserv.goTo(500, 300) → arrive sans obstacle
```

### Compatibilité avec pathfinding chaîné (waypoints)

Les waypoints passent par `goToChain()` qui utilise le même `waitEndOfTrajWithDetection(FORWARD)`.
Si un adversaire apparaît au waypoint 3 sur 5 :
- Stop au waypoint 3
- Navigator retry 2x
- Si échec → DecisionMaker peut recalculer le chemin complet

## Phases de migration

| Phase | Description | Statut |
|---|---|---|
| 0 | Extraction ObstacleZone, TableGeometry, tests, fix ARM drivers | ✅ |
| 1 | DetectionEvent — structure + publication dans Sensors | ✅ |
| 2 | `waitEndOfTrajWithDetection()` — centraliser la décision dans Asserv | ✅ |
| 3 | Supprimer `SensorsTimer` décisionnel + flags `temp_*` + `warnDetection` | ✅ |
| 4 | Fix synchronisation position beacon | ⬜ |
| 5 | Visualisation SVG capteurs L/R | ⬜ |
| 6 | Simulation adversaire en SIMU via UDP | ⬜ |
| 7 | Navigator + isOnPath pour reprise intelligente | ⬜ |
| 8 | Prédiction position adversaire (optionnel) | ⬜ |

### Phase 1 — DetectionEvent (remonter la position adversaire à l'IA)

**Objectif :** L'IA et le Navigator connaissent la position de l'adversaire, pas juste un booléen OBSTACLE.

**Fichier :** `common/geometry/DetectionEvent.hpp` (logique pure, header-only)

```cpp
struct DetectionEvent {
    int frontLevel;          // 0 à 4
    int backLevel;           // 0 à -4
    float x_adv_mm;          // Position adversaire sur la table (mm)
    float y_adv_mm;
    float d_adv_mm;          // Distance robot-adversaire (mm)
    uint64_t timestamp_us;   // Timestamp de la détection (chrono µs)
    bool valid;              // true si une détection récente existe
};
```

**Modifications :**
- `Sensors` possède un `DetectionEvent lastDetection_` mis à jour par `front()`/`back()`
- Accesseur `const DetectionEvent& lastDetection() const`
- Le Navigator peut le lire avant de décider retry vs contournement

**Tests :** `DetectionEventTest` dans `test/common/` — construction, validité, timestamp.

### Phase 2 — Fix synchronisation position beacon

**Objectif :** Réduire l'erreur de ~120mm à ~20mm.

**Problème actuel :**
```
t=0ms     Beacon mesure la position adversaire
t=62ms    SensorsTimer lit les données I2C
t=62ms    front() appelle pos_getPosition() ← position robot ACTUELLE
          Le robot a bougé de 62mm à 1m/s depuis la mesure
          L'adversaire aussi → erreur cumulée ~120mm
```

**Solution :** Sauvegarder la position robot au moment exact du `sync("beacon_sync")`.

```cpp
// Dans SensorsDriver::sync() ou Sensors::front()
ROBOTPOSITION posAtSync_ = sharedPos_->getRobotPosition();
// Utiliser posAtSync_ (pas la position courante) pour convertPositionBeaconToRepereTable
```

**Modifications :**
- `SensorsDriver::sync()` stocke `posAtSync_` (ARM et SIMU)
- `Sensors::front()`/`back()` utilisent `posAtSync_` au lieu de `robot()->sharedPosition()->getRobotPosition()`
- Accesseur `ROBOTPOSITION posAtSync()` dans `ASensorsDriver`

**Tests :** Test unitaire simulant un décalage temporel entre sync et lecture position.

### Phase 3 — Visualisation SVG capteurs L/R

**Objectif :** Voir les détections latérales dans le SVG Sensors.

**Nouvelle méthode dans `SvgWriterExtended` :**

```cpp
void writePosition_SensorLine(float x_robot, float y_robot, float theta_robot,
                              float distance_mm, int side, int color);
// side: -1=gauche, 0=centre, +1=droite
// Dessine une ligne depuis le robot dans la direction du capteur
// avec une croix (×) au point de détection
```

**Appel dans `Sensors::front()` :** quand `enableFrontLeft_` ou `enableFrontRight_` et détection < seuil.

**Tests :** Vérification visuelle en SIMU avec position fixe + détection simulée → SVG attendu.

### Phase 4 — Simulation adversaire en SIMU via UDP

**Objectif :** Tester la chaîne complète de détection sans hardware.

**Composants :**

1. **`OpponentReceiver`** — Thread UDP qui écoute sur le port 9871

```cpp
class OpponentReceiver : public utils::Thread {
    // Reçoit JSON : {"x": 800, "y": 1200}
    // Met à jour SensorsDriverSimu::vadv_ et calcule c2/c4/c6/c8
    // par projection géométrique depuis la position robot courante
};
```

2. **`SensorsDriverSimu` modifié** — accepte des positions injectées par OpponentReceiver

3. **Script Python `opponent_sim.py`** — envoie des positions adversaire en boucle

```python
# Scénario : adversaire traverse de gauche à droite
import socket, json, time
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
for x in range(200, 2800, 50):
    sock.sendto(json.dumps({"x": x, "y": 1000}).encode(), ("127.0.0.1", 9871))
    time.sleep(0.05)
```

**Port UDP :** 9871 (telemetry envoi = 9870, réception adversaire = 9871)

**Tests :** Test d'intégration SIMU : lancer opponent_sim.py + robot SIMU → vérifier que `front()` retourne level > 0 quand l'adversaire est devant.

### Phase 5 — Navigator lit DetectionEvent pour la reprise

**Objectif :** Éviter le blocage "adversaire devant mais nouvelle trajectoire l'évite".

**Modifications dans `Navigator::executeWithRetry()` :**

```cpp
if (ts == TRAJ_NEAR_OBSTACLE) {
    DetectionEvent det = robot_->actions().sensors().lastDetection();
    
    // Nouvelle trajectoire évite l'adversaire ?
    if (!isOnPath(det.x_adv_mm, det.y_adv_mm, robot_pos, destination, safetyRadius)) {
        // L'adversaire n'est pas sur le nouveau chemin → autoriser la reprise
        robot_->asserv().resetEmergencyOnTraj("Navigator: adv not on path");
        continue; // retry immédiat sans attendre
    }
    
    // Sinon : retry classique avec attente
    obstacleCount++;
    sleep(policy.waitTempoUs);
    ...
}
```

**`isOnPath()` dans `common/geometry/` :**

```cpp
bool isOnPath(float x_adv, float y_adv,
              float x_from, float y_from,
              float x_to, float y_to,
              float safetyRadius_mm);
// Distance point-segment < safetyRadius → true (adversaire sur le chemin)
```

**Tests :** `IsOnPathTest` dans `test/common/` — cas géométriques purs.

### Phase 6 — Prédiction position adversaire (optionnel)

**Objectif :** Anticiper le mouvement de l'adversaire entre deux mesures beacon.

**Principe :** Avec 2 mesures consécutives (t1, t2), estimer la vitesse de l'adversaire et extrapoler.

```cpp
// Dans DetectionEvent ou ObstacleTracker
float vx_adv = (x2 - x1) / (t2 - t1);  // mm/ms
float vy_adv = (y2 - y1) / (t2 - t1);
float x_predicted = x2 + vx_adv * dt;
float y_predicted = y2 + vy_adv * dt;
```

**Pas prioritaire** — la phase 2 (fix synchro) réduit déjà l'erreur de moitié. La prédiction n'est utile que pour des adversaires rapides (>1m/s).

## Capteurs physiques — état actuel

### Caméras ToF (BeaconSensors, Teensy I2C 0x2D)

```
         AVANT du robot
    ┌─────────────────────┐
    │  c1 (AV G bas)      │  c3 (AV D bas)
    │  c2 (AV G haut) →fL │  c4 (AV D haut) →fR
    │                     │
    │      ROBOT          │
    │                     │
    │  c5 (AR G bas)      │  c7 (AR D bas)
    │  c6 (AR G haut) →bL │  c8 (AR D haut) →bR
    └─────────────────────┘
         ARRIERE du robot
```

| Capteur API | Registre | Driver ARM | Config robot actuelle |
|---|---|---|---|
| `frontLeft()` | c2_mm | Actif (distance ou 500) | **Désactivé** `addConfigFront(false, true, false)` |
| `frontRight()` | c4_mm | Actif | **Désactivé** |
| `frontCenter` | beacon x/y/d + filtre | Actif | **Activé** |
| `backLeft()` | c6_mm | Actif (implémenté phase 0) | **Désactivé** `addConfigBack(false, true, false)` |
| `backRight()` | c8_mm | Actif (implémenté phase 0) | **Désactivé** |
| `backCenter` | beacon x/y/d + filtre | Actif | **Activé** |
| `rightSide()` | Sharp Gp2y0e02b | Stub (400) | Non connecté |
| `leftSide()` | Sharp Gp2y0e02b | Stub (400) | Non connecté |

### Beacon — données position adversaire

| Registre | Contenu | Utilisé |
|---|---|---|
| `x1_mm, y1_mm, a1_deg, d1_mm` | Adversaire 1 : position table, angle, distance | Oui |
| `x2-x4, y2-y4, a2-a4, d2-d4` | Adversaires 2 à 4 | Oui (si nbDetectedBots >= N) |
| `nbDetectedBots` | Nombre d'adversaires détectés | Oui |
| `flags` | Status beacon (0xFF = erreur) | Oui (retry 3x) |

## Zones de détection (ObstacleZone)

```
         y (devant robot)
         ▲
         │
    ┌────┼────┐ threshold_Front (620mm)
    │ 3  │  3 │ level 3 : zone moyenne → ralentir
    ├────┼────┤ threshold_veryclosed (460mm)
    │2G  │4 1D│ level 4 : dead center → STOP
    ├────┼────┤   level 1/2 : côtés
    │    │    │
────┼────┼────┼───> x (droite robot)
    │    │    │
    ├────┼────┤ -threshold_veryclosed (420mm)
    │-2G │-4-1D│ level -4 : dead center arrière → STOP
    ├────┼────┤ -threshold_Back (580mm)
    │ -3 │ -3│ level -3 : zone moyenne arrière → ralentir
    └────┼────┘
         │
         
    threshold_LR = 140 + 20 + 250 = 410mm (rayon robot + marge + rayon adversaire)
```

## Flux de la détection — du capteur à l'IA

```
1. BeaconSensors::getData()          I2C lecture registres Teensy
       ↓
2. SensorsDriver::sync()             Remplit vadv_ (positions adversaires)
       ↓
3. Sensors::front(true)              Pour chaque adversaire détecté :
   ├── convertPositionBeaconToRepereTable()   projection polaire → table
   ├── TableGeometry::isPointInsideTable()    filtre hors terrain
   ├── ObstacleZone::filtre_levelInFront()    classification level 0-4
   └── SvgWriter::writePosition_AdvPos()      tracé SVG
       ↓
4. SensorsTimer::onTimer()           Debounce (2 frames level 4 → warn)
   ├── Asserv::warnFrontDetectionOnTraj(4)    → setEmergencyStop()
   ├── Asserv::warnFrontDetectionOnTraj(3)    → setMaxSpeed(reduced)
   └── Asserv::warnFrontDetectionOnTraj(2)    → setMaxSpeed(normal)
       ↓
5. Asserv::setEmergencyStop()        Stoppe les moteurs immédiatement
       ↓
6. Navigator::executeWithRetry()     Reçoit TRAJ_NEAR_OBSTACLE
   ├── sleep(2s) + resetEmergencyOnTraj()
   ├── retry (max 2x standard)
   └── retourne TRAJ_NEAR_OBSTACLE à l'IA si max atteint
       ↓
7. IA (DecisionMakerIA)              Reçoit le TRAJ_STATE final
   └── décide : action suivante ou abandon
```

## Fichiers concernés

| Fichier | Rôle | Phase |
|---|---|---|
| `common/geometry/ObstacleZone.hpp/.cpp` | Logique pure filtres | ✅ fait |
| `common/geometry/TableGeometry.hpp` | Logique pure table | ✅ fait |
| `common/geometry/DetectionEvent.hpp` | Structure événement détection | Phase 1 |
| `common/geometry/IsOnPath.hpp` | Distance point-segment | Phase 5 |
| `common/action/Sensors.hpp/.cpp` | Orchestration + publication DetectionEvent | Phase 1-2 |
| `common/log/SvgWriter.hpp/.cpp` | Nouvelle méthode writePosition_SensorLine | Phase 3 |
| `bot-opos6ul/OPOS6UL_SvgWriterExtended.cpp` | Implémentation SVG L/R | Phase 3 |
| `driver-simu/SensorsDriver.cpp/.hpp` | Injection positions via OpponentReceiver | Phase 4 |
| `common/navigator/Navigator.cpp` | Lecture DetectionEvent + isOnPath | Phase 5 |
| `test/common/DetectionEventTest.cpp` | Tests logique pure | Phase 1 |
| `test/common/IsOnPathTest.cpp` | Tests géométriques | Phase 5 |
| `scripts/opponent_sim.py` | Script simulation adversaire UDP | Phase 4 |
