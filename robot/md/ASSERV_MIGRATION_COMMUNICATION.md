# Migration Communication PMX : OPOS6UL (AsservDriver) ↔ Nucleo (Opos6ulSerialIO)

## Contexte

Le PMX utilise :
- **Côté OPOS6UL** (brain) : `AsservDriver.cpp` qui envoie des commandes série sur `/dev/ttymxc1` (115200 baud)
- **Côté Nucleo** (asserv) : `Opos6ulSerialIO` (hérite de `SerialIO`) qui reçoit et exécute les commandes

L'objectif est de documenter le mapping complet des commandes et identifier ce qui reste à migrer.

## Handshake

Au démarrage, la Nucleo attend `###` avant d'accepter toute commande série.
C'est une protection contre le bruit sur RX quand l'OPOS6UL n'est pas connectée.

- **Nucleo** : `Opos6ulSerialIO::commandInput()` attend 3x `#` consécutifs
- **OPOS6UL** : `AsservDriver` constructeur envoie `###` après l'ouverture du port série

Voir `ASSERV_BUG_GLITCH_I2C.md` pour le détail du problème.

## Télémétrie (Nucleo → OPOS6UL)

Format envoyé par `SerialIO::positionOutput()` toutes les 100ms :
```
#x;y;theta;status;pending_count;left_speed;right_speed\r\n
```

Parsé par `AsservDriver::parseAsservPosition()` qui attend 8 champs (avec `debug_nb`).

**⚠️ Incompatibilité** : SerialIO envoie 7 champs, AsservDriver en attend 8 (le champ `debug_nb` manque).
**Action** : adapter `positionOutput()` dans `Opos6ulSerialIO` pour ajouter le compteur debug, ou adapter `AsservDriver` pour accepter 7 champs.

## Mapping complet des commandes

### ✅ Commandes fonctionnelles (SerialIO → AsservDriver)

| Char | SerialIO (Nucleo) | AsservDriver (OPOS6UL) | Description |
|------|-------------------|------------------------|-------------|
| `v` | `addStraightLine(dist)` | `motion_DoLine(dist)` | Avancer/reculer en mm |
| `t` | `addTurn(degToRad(angle))` | `motion_DoRotate(angle)` | Tourner (degrés envoyés) |
| `g` | `addGoTo(x, y)` | `motion_Goto(x, y)` | Aller à un point |
| `b` | `addGoToBack(x, y)` | `motion_GotoReverse(x, y)` | Aller à un point en marche arrière |
| `e` | `addGoToNoStop(x, y)` | `motion_GotoChain(x, y)` | Goto enchaîné sans arrêt |
| `f` | `addGoToAngle(x, y)` | `motion_DoFace(x, y)` | Se tourner vers un point |
| `h` | `setEmergencyStop()` | `path_InterruptTrajectory()` | Arrêt d'urgence |
| `r` | `resetEmergencyStop()` | `path_ResetEmergencyStop()` | Reset arrêt d'urgence |
| `P` | `setPosition(x, y, angle)` | `odo_SetPosition(x, y, angle)` | Recaler l'odométrie |
| `M0` | `enableMotors(false)` | `motion_FreeMotion()` | Libérer les moteurs |
| `M1` | `enableMotors(true)` | `motion_AssistedHandling()` | Activer l'asservissement |
| `S` | `limitMotorControllerConsignToPercentage(%)` | `motion_setLowSpeedForward/Backward(%)` | Limiter la vitesse |

### ⚠️ Commandes manquantes (à ajouter dans Opos6ulSerialIO)

| Char | AsservDriver (OPOS6UL) | Action | Priorité |
|------|------------------------|--------|----------|
| `n` | `motion_GotoReverseChain(x, y)` | Goto marche arrière enchaîné. **N'existe ni dans SerialIO ni RaspIO**. À créer dans `Opos6ulSerialIO`. | **Haute** — utilisé pour les trajectoires |
| `N` | `motion_setMaxSpeed(dist%, angle%)` | Vitesse max avec 2 paramètres. Dans RaspIO c'est "restore accélération normale" (différent !). À redéfinir. | **Moyenne** |
| `!` | `motion_setMaxSpeed(false)` | Désactiver la limitation de vitesse. Dans RaspIO c'est "stop actions" (différent !). À redéfinir. | **Moyenne** |
| `R` | `motion_ActivateManager(true)` | Reset asserv. Dans RaspIO c'est géré. À migrer. | **Moyenne** |
| `A` | `motion_ActivateReguAngle(bool)` | Activer/désactiver régulateur angle. **N'existe nulle part**. À créer. | **Basse** — debug |
| `D` | `motion_ActivateReguDist(bool)` | Activer/désactiver régulateur distance. **N'existe nulle part**. À créer. | **Basse** — debug |

### 🔇 Commandes SerialIO non utilisées par AsservDriver (raccourcis debug uniquement)

| Char | Description | Verdict |
|------|-------------|---------|
| `z` | Avancer 20cm (hardcodé) | Debug shell uniquement, pas besoin côté brain |
| `s` | Reculer 20cm (hardcodé) | Idem |
| `q` | Tourner 45° gauche (hardcodé) | Idem |
| `d` | Tourner 45° droite (hardcodé) | Idem |
| `p` | Lire position (format texte) | Redondant avec le flux `#x;y;...` de positionOutput |
| `C` | Régler entraxe codeurs | Pas utilisé par le brain |

### ❌ Méthodes deprecated dans AAsservDriver — Audit complet

Toutes les implémentations (EsialR, AsservDriver, AsservCborDriver) ont des stubs vides/TODO pour ces méthodes.

#### Dead code (jamais appelé dans le brain) → à supprimer de AAsservDriver

| Méthode | EsialR | AsservDriver | Appelant | Action |
|---------|--------|-------------|----------|--------|
| `setMotorLeftPosition/RightPosition` | TODO stub | stub vide | ❌ Aucun | **Supprimer** |
| `getMotorLeftCurrent/RightCurrent` | TODO stub | stub vide | ❌ Aucun | **Supprimer** |
| `getLeftExternalEncoder` | TODO stub | stub vide | ❌ Aucun | **Supprimer** |
| `getRightExternalEncoder` | TODO stub | stub vide | ❌ Aucun | **Supprimer** |
| `getLeftInternalEncoder` | TODO stub | stub vide | ❌ Aucun | **Supprimer** |
| `getRightInternalEncoder` | TODO stub | stub vide | ❌ Aucun | **Supprimer** |
| `getCountsInternal` | TODO stub | stub vide | ❌ Aucun | **Supprimer** |
| `getDeltaCountsExternal` | TODO stub | stub vide | ❌ Aucun | **Supprimer** |
| `resetEncoders` | TODO stub | stub vide | ❌ Aucun | **Supprimer** |
| `resetInternalEncoders` | TODO stub | stub vide | ❌ Aucun | **Supprimer** |
| `resetExternalEncoders` | TODO stub | stub vide | ❌ Aucun | **Supprimer** |
| `motion_DisablePID` | → FreeMotion | stub | ❌ Aucun | **Supprimer** |
| `path_GetLastCommandStatus` | return -1 | return -1 | ❌ Aucun | **Supprimer** |

#### Utilisé uniquement dans les tests O_* → garder en stub

| Méthode | Appelant | Action |
|---------|----------|--------|
| `setMotorLeftPower/RightPower` | `O_AsservCalibrationTest.cpp` (test) | Garder en stub, test non critique |
| `stopMotors` | `O_AsservXYRotateTest.cpp` (test) | Garder en stub, redirige vers emergency_stop |
| `stopMotorLeft/Right` | ❌ Aucun | **Supprimer** |
| `getCountsExternal` | `O_Asserv_SquareTest.cpp`, `O_AsservCalibrationTest.cpp` (tests) | Garder en stub |

#### Utilisé dans Asserv.cpp (wrapper interne) → garder pour l'instant

| Méthode | Appelant | Action |
|---------|----------|--------|
| `motion_ActivateReguDist` | `Asserv.cpp` (doCalage, doLine, etc.) | Garder — implémenté dans EsialR, stub dans les autres |
| `motion_ActivateReguAngle` | `Asserv.cpp` (doCalage, doLine, etc.) | Garder — idem |

## Comparaison AsservEsialR (asserv interne) vs AsservDriver (série Nucleo)

`AsservEsialR` est l'ancienne asserv qui tournait directement sur l'OPOS6UL avec accès direct aux objets
(codeurs, odo, consignController, commandManager). `AsservDriver` délègue tout à la Nucleo via série.

### Commandes communes

| Méthode | EsialR (accès direct) | AsservDriver (série) |
|---|---|---|
| `motion_DoLine(dist)` | `commandM_->addStraightLine()` | `v` |
| `motion_DoRotate(angle)` | `commandM_->addTurn()` | `t` |
| `motion_DoFace(x, y)` | `commandM_->addGoToAngle()` | `f` |
| `motion_DoFace(x, y, **back**)` | `commandM_->addGoToAngleReverse()` | ⚠️ **Manque** — `f` ne supporte pas le reverse |
| `motion_Goto(x, y)` | DoFace + DoLine (décomposé, 2 étapes) | `g` (atomique, le Nucleo corrige l'angle en continu) |
| `motion_GotoReverse(x, y)` | DoFace(reverse) + DoLine(-dist) | `b` (atomique) |
| `motion_GotoChain(x, y)` | **TODO** (non implémenté !) | `e` ✅ |
| `motion_GotoReverseChain(x, y)` | **TODO** (non implémenté !) | `n` à créer |
| `path_InterruptTrajectory()` | `commandM_->setEmergencyStop()` | `h` |
| `path_ResetEmergencyStop()` | `commandM_->resetEmergencyStop()` | `r` |
| `odo_SetPosition(x, y, a)` | `odo_->setX/Y/Theta()` | `P` |
| `odo_GetPosition()` | `odo_->getXmm/Ymm/Theta()` | flux `#x;y;...` |
| `motion_FreeMotion()` | `consignC/commandM->perform_On(false)` | `M0` |
| `motion_AssistedHandling()` | `consignC/commandM->perform_On(true)` | `M1` |
| `motion_setLowSpeedForward/Backward()` | `consignC_->setLowSpeed...()` | `S` |

### Commandes EsialR en plus (accès direct objets internes)

| Méthode | Objet accédé | Dispo série ? | Verdict |
|---|---|---|---|
| `motion_DoDirectLine(dist)` | `consignC_->add_dist_consigne()` bypass CommandManager | ❌ | Pas besoin |
| `motion_ActivateManager(bool)` | `initAsserv()/stopAsserv()` crée/détruit les objets | `R` partiel | À évaluer |
| `motion_ActivateReguDist(bool)` | `consignC_->dist_Regu_On()` | ❌ | Debug — basse priorité |
| `motion_ActivateReguAngle(bool)` | `consignC_->angle_Regu_On()` | ❌ | Debug — basse priorité |
| `motion_ResetReguDist()` | `consignC_->reset_regu_dist()` | ❌ | Debug |
| `motion_ResetReguAngle()` | `consignC_->reset_regu_angle()` | ❌ | Debug |
| `motion_ActivateQuadRamp(bool)` | `consignC_->setQuadRamp_Angle/Dist()` | ❌ | Debug |
| `motion_setMaxSpeed(dist, angle)` | **TODO** (pas implémenté !) | `N` à créer | À faire |

### Différences d'implémentation importantes

| Point | EsialR (interne) | AsservDriver (Nucleo) |
|---|---|---|
| **Goto** | Décomposé en DoFace + DoLine (2 commandes séparées, moins précis) | Atomique `g` (le Nucleo corrige l'angle en continu pendant l'avance) |
| **GotoReverse** | Décomposé aussi | Atomique `b` |
| **DoFace reverse** | Supporte `back_reversed` via `addGoToAngleReverse()` | **Manque** — `f` ne passe pas le flag reverse |
| **GotoChain** | Non implémenté (TODO) | `e` fonctionne ✅ |
| **waitEndOfTraj** | Poll direct `commandM_->getCommandStatus()` | Poll indirect via flux série `#...;status;...` |

### Commandes deprecated (communes aux 3 implémentations)

Toutes marquées `TODO` avec `logger().error()` dans EsialR, stubs vides dans AsservDriver et AsservCborDriver.
13 méthodes sont du **dead code** (jamais appelées dans le brain) → voir l'audit complet plus haut.

**→ À supprimer de `AAsservDriver` et de toutes les implémentations (EsialR, AsservDriver, AsservCborDriver, AsservDriverSimu).**

---

## Architecture cible

```
SerialIO (base partagée : 16 commandes + positionOutput)
  └── Opos6ulSerialIO (handshake ### + commandes spécifiques PMX)
        - commandInput() override : handshake puis SerialIO::commandInput()
        - TODO : ajouter n, F (doface reverse), N, !, R dans un customCommandHandle()
        - TODO : override positionOutput() pour ajouter debug_nb
```

## SerialCbor — Protocole binaire avec cmd_id et CRC (utilisé par Princess/Picrate)

### Définition

`SerialCbor` est le protocole de communication binaire utilisé par les robots **Princess** et **Picrate** du projet `asserv_chibios`. C'est aussi le protocole utilisé par **EsialRobotik** (même base de code). PMX est le seul robot à ne pas l'utiliser.

### Format de trame

```
| 0xDEADBEEF (4 oct) | CRC32 (4 oct) | Taille payload (4 oct) | Payload CBOR (N oct) |
```

- **Sync word** `0xDEADBEEF` : permet de retrouver le début d'une trame en cas de désynchronisation
- **CRC32** : calculé sur le payload CBOR → détection de corruption (bruit série, glitches)
- **Taille** : longueur du payload en octets
- **Payload** : encodé en [CBOR](https://cbor.io/) (binaire compact, auto-descriptif)

### Commandes (Brain → Nucleo)

Chaque commande est une **map CBOR** avec un type, un **ID unique**, et des paramètres typés :

```
{ "cmd": 23, "ID": 42, "X": 500.0, "Y": 300.0 }   → goto_front(500, 300), commande #42
```

Types de commandes :
| Valeur | Nom | Paramètres |
|---|---|---|
| 10 | `emergency_stop` | aucun |
| 11 | `emergency_stop_reset` | aucun |
| 15 | `normal_speed_acc_mode` | aucun |
| 16 | `slow_speed_acc_mode` | aucun |
| 17 | `max_motor_speed` | pourcentage |
| 20 | `turn` | angle (degrés) |
| 21 | `straight` | distance (mm) |
| 22 | `face` | x, y |
| 23 | `goto_front` | x, y |
| 24 | `goto_back` | x, y |
| 25 | `goto_nostop` | x, y |
| 26 | `set_position` | x, y, theta |
| 30 | `orbital_turn` | angle, forward, right |

### Télémétrie (Nucleo → Brain)

Array CBOR envoyé à 10Hz :
```
[ X, Y, theta, cmd_id, status, pending_count, left_speed, right_speed ]
```

Le champ `cmd_id` est la clé : il renvoie l'**ID de la commande en cours d'exécution**. Le brain peut vérifier : `received_id >= sent_id AND status == IDLE` → commande terminée avec certitude.

### Avantages vs SerialIO (ASCII)

| Aspect | SerialIO (PMX actuel) | SerialCbor (Princess/Picrate) |
|---|---|---|
| **Encodage** | ASCII (1 char = 1 commande) | CBOR binaire (compact, typé) |
| **Intégrité** | Aucune vérification | CRC32 sur chaque trame |
| **Framing** | Aucun (vulnérable au bruit) | Sync word `0xDEADBEEF` |
| **Command ID** | Non → impossible de savoir quelle commande est finie | Oui → matching précis commande envoyée/terminée |
| **waitEndOfTraj** | Poll status seul (ambiguïté si commande perdue) | Poll status + cmd_id (confirmation fiable) |
| **Robustesse bruit** | Handshake `###` requis, bruit = commandes parasites | CRC invalide = trame ignorée silencieusement |
| **Extensibilité** | Ajout de commandes = nouveau caractère (limité à ~50) | Ajout = nouveau enum, pas de limite |
| **Complexité côté Brain** | Simple (write char + read string) | Lib CBOR nécessaire (cbor2 en Python, qcbor en C) |

### Implémentations existantes du protocole CBOR

| Composant | Langage | Repo | Rôle |
|---|---|---|---|
| `SerialCbor` | C++ (ChibiOS) | `asserv_chibios/src/Communication/SerialCbor.cpp` | Nucleo : reçoit commandes CBOR, envoie position CBOR |
| `CborStreamStateMachine` | C++ (ChibiOS) | `asserv_chibios/src/Communication/cborStream/` | Machine à états : sync word + CRC + décodage CBOR |
| `Ia-Asserv_Com` | Python | [github.com/EsialRobotik/Ia-Asserv_Com](https://github.com/EsialRobotik/Ia-Asserv_Com) | Brain (Raspberry Pi) : encode commandes, décode position |
| `qcbor` | C | `asserv_chibios/src/util/qcbor/` | Lib CBOR encode/decode (utilisée par SerialCbor) |
| `cppCrc.h` | C++ | `asserv_chibios/src/Communication/Crc/cppCrc.h` | CRC32 header-only (table lookup) |

`Ia-Asserv_Com` est la **référence Python** du protocole côté Brain, utilisée par EsialRobotik sur Raspberry Pi. 
Pour PMX (brain en C++ sur OPOS6UL), on créera l'équivalent C++ : `AsservCborDriver`.

### Coût de migration vers SerialCbor

**Côté Nucleo** : rien à faire, `SerialCbor` existe déjà et supporte toutes les commandes nécessaires. Il suffit de l'instancier à la place de `Opos6ulSerialIO` dans `main.cpp`.

**Côté OPOS6UL** : créer `AsservCborDriver` (nouvelle classe, même interface `AAsservDriver`) :

```
AAsservDriver (interface abstraite)
  ├── AsservDriver       ← actuel, protocole ASCII, à conserver comme fallback
  ├── AsservCborDriver   ← nouveau, protocole CBOR + CRC + cmd_id
  └── AsservDriverSimu   ← simulateur, existe déjà
```

Dépendances à intégrer dans PMX-CORTEX :
- `qcbor` (lib C) → copier depuis `asserv_chibios/src/util/qcbor/`
- `cppCrc.h` (CRC32) → copier depuis `asserv_chibios/src/Communication/Crc/`
- `serialib` → déjà présent dans `src/driver-arm/`

Implémentation :
- Encoder les commandes en CBOR maps `{ "cmd": N, "ID": id, "X": x, "Y": y }` + frame sync+CRC+taille
- Décoder les réponses position (CBOR array + frame sync+CRC) avec machine à états
- `waitEndOfTraj` amélioré : poll `received_cmd_id >= sent_cmd_id AND status == IDLE`
- Anti-race `status_countdown = 2` (déjà présent dans AsservDriver actuel)

---

## Plan de migration

### Phase 1 — Fait ✅
- [x] Handshake `###` dans `Opos6ulSerialIO::commandInput()`
- [x] Envoi `###` dans `AsservDriver` constructeur
- [x] 12 commandes de base fonctionnelles via `SerialIO`

### Phase 2 — Commandes manquantes (haute priorité)
- [ ] Ajouter DoFace reverse (variante de `f` avec flag reverse) dans `Opos6ulSerialIO` — utilisé par `motion_GotoReverse()`
- [ ] Ajouter `n` (GotoReverseChain) dans `Opos6ulSerialIO`
- [ ] Ajouter `N` (setMaxSpeed avec dist% et angle%) dans `Opos6ulSerialIO`
- [ ] Ajouter `!` (désactiver limitation vitesse) dans `Opos6ulSerialIO`
- [ ] Ajouter `R` (reset asserv) dans `Opos6ulSerialIO`

### Phase 3 — Optionnel (debug)
- [ ] Ajouter `A` (régulateur angle on/off)
- [ ] Ajouter `D` (régulateur distance on/off)

### Phase 4 — Nettoyage deprecated (AAsservDriver) ⚠️ en grande partie fait

Supprimer de `AAsservDriver.hpp` et de **toutes** les implémentations (EsialR, AsservDriver, AsservCborDriver, AsservDriverSimu) :

**Dead code — supprimé ✅ (avril 2026) :**
- [x] `setMotorLeftPosition` / `setMotorRightPosition`
- [x] `getMotorLeftCurrent` / `getMotorRightCurrent`
- [x] `getLeftExternalEncoder` / `getRightExternalEncoder`
- [x] `getLeftInternalEncoder` / `getRightInternalEncoder`
- [x] `getCountsInternal`
- [x] `resetInternalEncoders` / `resetExternalEncoders`
- [x] `motion_DisablePID`
- [x] `path_GetLastCommandStatus`

**Dead code — encore présent :**
- [ ] `getDeltaCountsExternal`
- [ ] `resetEncoders`

**Garder en stub (appelé par des tests O_*) :**
- `setMotorLeftPower` / `setMotorRightPower` → `O_AsservCalibrationTest.cpp`
- `stopMotors` → `O_AsservXYRotateTest.cpp`
- `getCountsExternal` → `O_Asserv_SquareTest.cpp`, `O_AsservCalibrationTest.cpp`

**Garder (appelé par Asserv.cpp) :**
- `motion_ActivateReguDist` / `motion_ActivateReguAngle` → utilisé dans doCalage, doLine, etc.

**Supprimé :** `stopMotorLeft` / `stopMotorRight` ✅

**Autres nettoyages :**
- [ ] Résoudre l'incompatibilité télémétrie (7 vs 8 champs)
- [ ] Supprimer `raspIO.cpp` / `raspIO.h` du dossier PMX (plus utilisés)

### Phase 5 — Fait ✅ : AsservCborDriver

- [x] Intégrer qcbor côté OPOS6UL (`libs/qcbor/`)
- [x] Intégrer cppCrc.h (`libs/crc/`)
- [x] Créer `CborFrameDecoder` (décodeur trames position CBOR)
- [x] Créer `AsservCborDriver` (encode commandes CBOR, décode position, waitEndOfTraj avec cmd_id)
- [x] Mettre à jour `CMakeLists.txt`
- [x] Mettre à jour `AAsservDriver::create()` pour utiliser `AsservCborDriver`
- [x] Côté Nucleo : remplacer `Opos6ulSerialIO` par `SerialCbor` dans `main.cpp`
- [x] Test de bout en bout

### Phase 6 — Commandes non-bloquantes + gotoChain (pathfinding fluide)

Objectif : aligner sur EsialRobotik (Ia-Python) — commandes non-bloquantes, `waitEndOfTraj` explicite, `gotoChain` pour le pathfinding fluide.

Ref : [EsialRobotik Ia-Python](https://github.com/EsialRobotik/Ia-Python) — `movement_manager.py` utilise `go_to_chain()` (non-bloquant) + `wait_for_asserv()` (bloquant avec cmd_id)

**Étape 1 — Commandes non-bloquantes dans AsservCborDriver**
- [ ] Les `motion_*` (DoLine, DoRotate, Goto, etc.) envoient la commande et retournent immédiatement
- [ ] `waitEndOfTraj()` ajouté dans `AAsservDriver` (interface abstraite)
- [ ] `AsservCborDriver` : poll status + cmd_id (`lastReceivedCmdId_ >= lastSentCmdId_`)
- [ ] `AsservDriver` (ASCII) : poll status (comme `nucleo_waitEndOfTraj` actuel)
- [ ] `AsservDriverSimu` : attend la fin de la simulation interne

**Étape 2 — Wrappers bloquants dans Asserv.cpp**
- [ ] `Asserv::doLine(dist)` = `motion_DoLine(dist)` + `waitEndOfTraj()` → bloquant, compatibilité assurée
- [ ] Idem pour `gotoXY`, `doRotate`, `doFaceTo`, etc.
- [ ] Aucun test ni code IA à modifier

**Étape 3 — gotoChain pour le pathfinding fluide**
- [ ] `Asserv::gotoChain(x, y)` = `motion_GotoChain(x, y)` **sans** `waitEndOfTraj` → non-bloquant
- [ ] `IAbyPath::doPathForwardTo` : `gotoChain` pour les waypoints intermédiaires, `gotoXY` pour le dernier
- [ ] Le robot enchaîne les waypoints sans s'arrêter → trajectoire fluide
- [ ] `waitEndOfTraj` vérifie `lastReceivedCmdId_ >= lastSentCmdId_ AND status == IDLE`

---

## ⚠️ Points d'attention

### Goto atomique vs décomposé
Le Nucleo gère le Goto de manière atomique (`g` = rotation + avance avec correction angle continue).
C'est plus précis que l'ancienne approche EsialR (DoFace puis DoLine en 2 étapes séparées).
**Avantage de la migration vers la Nucleo.**

### DoFace reverse
`AsservEsialR::motion_DoFace(x, y, back_reversed=true)` appelle `addGoToAngleReverse()`.
Cette variante n'existe pas dans SerialIO (`f` fait toujours face avant).
Il faut soit :
- Ajouter un nouveau caractère (ex: `F` pour DoFace reverse) dans `Opos6ulSerialIO`
- Ou ajouter un paramètre optionnel à `f` (ex: `f<x>#<y>#1\n` pour reverse)

### Conflit de commandes N/n/!
Les caractères `N`, `n`, `!` ont des significations **différentes** entre :
- **RaspIO** : `N` = restore accélération normale, `n` = accélération lente
- **AsservDriver** : `N` = set max speed (dist%, angle%), `n` = goto reverse chain

**Décision** : dans `Opos6ulSerialIO`, utiliser la sémantique **AsservDriver** (c'est le brain qui envoie).

### Cast vitesse int8_t
`SerialIO::positionOutput()` caste les vitesses en `int8_t` → perte si vitesse > 127%.
À surveiller, mais en pratique les vitesses MD22 sont limitées à ±100%.

## Fichiers concernés

| Fichier | Rôle |
|---------|------|
| `asserv_chibios/src/Robots/PMX/Opos6ulSerialIO.h` | Classe spécifique PMX (handshake + commandes custom) |
| `asserv_chibios/src/Robots/PMX/Opos6ulSerialIO.cpp` | Implémentation |
| `asserv_chibios/src/Robots/PMX/main.cpp` | Instanciation `Opos6ulSerialIO` |
| `asserv_chibios/src/Communication/SerialIO.cpp` | Base (non modifié) |
| `PMX-CORTEX/robot/src/driver-arm/AsservDriver.cpp` | Côté OPOS6UL, envoie `###` + commandes |
| `PMX-CORTEX/robot/src/driver-arm/AsservDriver.hpp` | Interface (méthodes deprecated à nettoyer) |
| `PMX-CORTEX/robot/src/common/asserv.esial/AsservEsialR.cpp` | Ancienne asserv interne (référence) |
| `asserv_chibios/src/Robots/PMX/raspIO.cpp` | **À supprimer** (plus utilisé) |
| `asserv_chibios/src/Robots/PMX/raspIO.h` | **À supprimer** (plus utilisé) |
