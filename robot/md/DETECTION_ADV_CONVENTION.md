# Convention canonique détection adversaire

Convention partagée **balise Teensy ↔ driver ARM ↔ driver SIMU ↔ filtre `ObstacleZone` ↔ tests** pour la position d'un adversaire détecté. Aucun swap ni transformation cachée entre les couches.

---

## 1. Convention canonique (en repère ROBOT)

```
RobotPos.x         = composante AVANT  (>0 devant, <0 derrière)
RobotPos.y         = composante LATÉRALE (>0 gauche, <0 droite)
RobotPos.theta_deg = angle adv vu du robot (0° = pile devant,
                     sens trigo positif CCW = gauche)
RobotPos.d         = distance centre robot ↔ centre adversaire (mm)
```

**Cohérence trigonométrique** :
```
x = d · cos(theta_deg)
y = d · sin(theta_deg)
```

C'est ce que publie la balise Teensy ([TofSensors.cpp:444-445](../../teensy/IO_t41_ToF_DetectionBeacon/src/TofSensors.cpp)) avec `decalage_deg = -110` qui aligne `0° = devant`.

### Exemples concrets (robot orienté vers l'Est, theta_table=0)

| Position adv table | Position adv en repère robot | theta_deg balise |
|---|---|---|
| pile devant à 500mm  | x=+500, y=0     | 0°    |
| pile à gauche 500mm  | x=0,    y=+500  | +90°  |
| pile à droite 500mm  | x=0,    y=-500  | -90°  |
| pile derrière 500mm  | x=-500, y=0     | 180°  |
| devant-droit à 45°   | x=+354, y=-354  | -45°  |
| arrière-gauche 135°  | x=-354, y=+354  | +135° |

---

## 2. Zones du filtre (avec thresholds typiques LR=200, Front=600, veryClosed=300)

```
                                 +x (avant)
                                  ↑
                  600 ┌─────┬─────┬─────┐
                      │     │  3  │     │  ← zone moyenne (300<x≤600)
                  300 ├─────┼─────┼─────┤
                      │  2  │  4  │  1  │  ← très proche (x≤300)
        +y (gauche) ←─┼─────┼─────┼─────┼─→ -y (droite)
                      │     │  0  │     │  ← x=0 ni front ni back
                      ├─────┼─────┼─────┤
                      │ -2  │ -4  │ -1  │
                 -300 ├─────┼─────┼─────┤
                      │     │ -3  │     │
                 -600 └─────┴─────┴─────┘
                          200 -200
```

| Level | Conditions (après patch balise +/-50 sur x et y) | Action Asserv |
|---|---|---|
| **+1** | x≤300, -600≤y≤-200 (devant-droit proche) | `obstacleSpeedPercent_` |
| **+2** | x≤300, +200≤y≤+600 (devant-gauche proche) | `obstacleSpeedPercent_` |
| **+3** | 300<x≤600, |y|≤600 (zone moyenne devant) | `obstacleSpeedPercent_` |
| **+4** | x≤300, -200≤y≤+200 (dead front) | `setEmergencyStop()` |
| **0** | x=0 OU x>600 OU |y|>600 OU adv derrière (côté front) | rien |
| **-1** | x≥-300, -600≤y≤-200 (derrière-droit proche) | `obstacleSpeedPercent_` |
| **-2** | x≥-300, +200≤y≤+600 (derrière-gauche proche) | `obstacleSpeedPercent_` |
| **-3** | -600≤x<-300, |y|≤600 (zone moyenne derrière) | `obstacleSpeedPercent_` |
| **-4** | x≥-300, -200≤y≤+200 (dead back) | `setEmergencyStop()` |

**Anti-mix front/back** :
- Filtre front teste `xdist_adv > 0` (strict)
- Filtre back teste `xdist_adv < 0` (strict)
- À `x=0` exactement → les deux retournent `0` (zone latérale pure ignorée)
- Patch balise (+/-50) ne s'applique pas si valeur exactement `0`

---

## 3. Activation en match

Configuration faite **une seule fois** au démarrage match dans
[O_State_DecisionMakerIA::execute()](../src/bot-opos6ul/O_State_DecisionMakerIA.cpp) :

```cpp
robot.actions().sensors().setIgnoreFrontNearObstacle(true, false, true); // L+R ignored, C actif
robot.actions().sensors().setIgnoreBackNearObstacle (true, false, true); // L+R ignored, C actif
robot.actions().sensors().startSensorsThread(20);
```

Capteurs activés/ignorés en match :

| | Front-L | Front-C (balise) | Front-R | Back-L | Back-C (balise) | Back-R |
|---|---|---|---|---|---|---|
| `enable` ([OPOS6UL_ActionsExtended.cpp:129-130](../src/bot-opos6ul/OPOS6UL_ActionsExtended.cpp)) | ❌ | ✅ | ❌ | ❌ | ✅ | ❌ |
| `ignore` (ci-dessus) | true | false | true | true | false | true |
| **Effectif** | ❌ | ✅ | ❌ | ❌ | ✅ | ❌ |

Les capteurs ToF latéraux gauche/droite (registres balise `c1..c8`) sont
hardware-prêts mais **désactivés en match 2026** : pas qualifiés côté
géométrie table (risque de faux positifs sur bordures/décor). Réactivation
future : passer `addConfigFront(true, true, true)` + `setIgnoreFrontNearObstacle(false, false, false)`.

> Pas besoin de jouer avec `setIgnore*` avant chaque LINE/GO_TO :
> les flags sont **persistants**, posés une fois au start match.

---

## 4. Réaction de l'asservissement

Logique symétrique front/back dans
[Asserv::waitEndOfTrajWithDetection()](../src/common/asserv/Asserv.cpp) :

```cpp
if (type == FORWARD) {
    if (det.frontLevel == 4) { setEmergencyStop(); return TRAJ_NEAR_OBSTACLE; }
    if (det.frontLevel >= 3) motion_setAccDecPercent(obstacleSpeedPercent_);
    else                     applySpeedSnapshotDirect(...);  // restaure user
}
else if (type == BACKWARD) {
    if (det.backLevel == -4) { setEmergencyStop(); return TRAJ_NEAR_OBSTACLE; }
    if (det.backLevel <= -3) motion_setAccDecPercent(obstacleSpeedPercent_);
    else                     applySpeedSnapshotDirect(...);
}
// type == ROTATION : détection ignorée
```

`TRAJ_NEAR_OBSTACLE` remonte dans Navigator qui retry `maxObstacleRetries`
fois (avec recul). Si épuisé, l'instruction strategy renvoie
`InstructionOutcome::SKIPPED_OBSTACLE` et le runner passe à la suivante
(comportement v2, pas d'abort).

---

## 5. Tests qui ancrent la convention

### 5.1 Tests unitaires filtre — [ObstacleZoneTest.cpp](../test/common/ObstacleZoneTest.cpp)

23 asserts, dans `common-test` :

- **9 cas filtres directs** : Level1 droite, Level2 gauche, Level3 moyenne, Level4 dead-front, Outside-Behind, Back Level4 dead-behind, Back Outside-In-Front, Overlap1and4, ZeroPosition
- **8 cas chaîne balise → filtre** : adv devant / droite -60° / gauche +60° / arrière 180° / droite pure -90° / gauche pure +90° / arrière-droit -135° / arrière-gauche +135°
- **6 cas frontières et zones exclues** : trop latéral droite, trop latéral gauche, trop loin devant, trop loin derrière, frontière back LR, x=0 axe latéral pur

### 5.2 Tests d'intégration runner — [O_StrategyJsonRunnerTest.cpp](../src/bot-test-opos6ul/O_StrategyJsonRunnerTest.cpp)

Setup test aligné sur match (`setIgnoreBackNearObstacle(true, false, true)` =
back centre actif).

- **SR06** : adv devant (cône front) → `SKIPPED_OBSTACLE` ✓
- **SR07** : adv pile derrière → `BACKWARD: STOP level -4` → `SKIPPED_OBSTACLE` ✓

Voir [STRATEGY_DECISION_RUNNER.md §6](STRATEGY_DECISION_RUNNER.md) pour la table complète SR01..SR09.

### 5.3 Test debug live — option CLI `/a <x> <y>`

Injection d'un adv fixe en coordonnées table mm pour reproduire un scénario
en SIMU **et** sur robot réel (cf. [Robot.cpp](../src/common/Robot.cpp) +
[O_State_DecisionMakerIA.cpp](../src/bot-opos6ul/O_State_DecisionMakerIA.cpp)
+ [driver-arm/SensorsDriver.cpp](../src/driver-arm/SensorsDriver.cpp)).

```bash
./bot-opos6ul m /k /s PMX1 /a 230 350    # adv injecté sur trajet PMX1
```

Sur ARM, le fake adv s'**ajoute** aux vrais adv balise (utile pour
homologation hors-table). À ne **pas** oublier de retirer en match.

---

## 6. Bug historique corrigé (avril 2026)

**Avant** : le filtre `ObstacleZone::filtre_levelInFront` lisait
`y > 0 = devant` et `x = lateral`, alors que la balise Teensy publie
`x = avant` et `y = lateral`. Conséquence : en match réel, **le filtre
retournait quasi-toujours 0**, l'asserv ne déclenchait ni le ralentissement
ni l'arrêt d'urgence sur adversaire détecté par balise.

Le simu, lui, avait sa propre convention `transformPosTableToPosRobot` qui
matchait le filtre — donc les tests `SR06` (et seulement le simu) passaient,
masquant le bug.

**Après** : le filtre + simu + tests sont alignés sur la convention balise
(`x = avant`, `y = gauche`). Plus aucun swap entre couches. Détection
front/back fonctionnelle de bout en bout.

Voir [commit history](https://github.com/pmrobotix/PMX-CORTEX/commits/main/robot/src/common/geometry/ObstacleZone.cpp) pour le diff complet.
