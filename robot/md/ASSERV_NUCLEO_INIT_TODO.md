# Asserv Nucleo — état session 2026-04-26 et TODO

Fichier de référence pour la suite des travaux sur la communication
OPOS6UL ↔ Nucleo CBOR + sequence d'init match.

Compagnon des docs existants :
- [ASSERV_MIGRATION_COMMUNICATION.md](ASSERV_MIGRATION_COMMUNICATION.md) — handshake cmd_id ack (architecture)
- [O_STATE_NEW_INIT.md](O_STATE_NEW_INIT.md) — state machine d'init match
- [ASSERV_BUG_GLITCH_I2C.md](ASSERV_BUG_GLITCH_I2C.md) — historique glitch série/I2C

## 1. État actuel (fin de session 2026-04-26)

### 1.1 Fixes appliqués et stabilisés

| Sujet | Fichier | Détail |
|---|---|---|
| Handshake `cmd_id` ack | [src/common/asserv/Asserv.cpp](../src/common/asserv/Asserv.cpp), [src/driver-arm/AsservCborDriver.cpp](../src/driver-arm/AsservCborDriver.cpp), [src/driver-simu/AsservDriver.cpp](../src/driver-simu/AsservDriver.cpp) | `Asserv::waitEndOfTrajWithDetection` attend `lastReceivedCmdId() >= lastSentCmdId() && status == IDLE`. Plus de Phase 1 wait RUNNING. Atomiques sur `nextCmdId_` / `lastReceivedCmdId_`. Suppression du HACK `statusCountDown_`. |
| `WAIT NUCLEO...` (asserv arrive après le boot OPOS6UL) | [src/bot-opos6ul/O_State_NewInit.cpp](../src/bot-opos6ul/O_State_NewInit.cpp) `waitForAsserv()` | Boucle de retry `tryReconnect()` toutes les 500ms. RESET (BACK shield ou bouton balise) annule l'attente. Mode `/k` : bloquant. |
| Sleep 1s stabilisation post-tryReconnect | `O_State_NewInit::waitForAsserv()` | La Nucleo qui vient de booter envoie ses frames de position avant que `CommandManager` accepte les commandes. 1s de marge. |
| Reconnect tardif Nucleo → retour CONFIG | `O_State_NewInit::waitForAsserv()` | Si on est passe par "WAIT NUCLEO..." (Nucleo allumee tardivement, ARU lache, etc.), la Nucleo peut avoir glitche au demarrage et le robot peut avoir bouge. Au lieu de faire setPos directement avec la pose initiale (qui ne correspond plus au robot reel), on **freeMotion + retour PHASE_CONFIG** + message LCD `"REPOSITIONNER PUIS SETPOS"`. L'operateur doit physiquement repositionner le robot et re-cliquer SETPOS. Securite cruciale : un setPos avec pose erronee fausserait toute la trajectoire pre-tirette puis match. |
| Sleep 200ms `startMotionTimerAndOdo` (override PMX) | [src/bot-opos6ul/OPOS6UL_AsservExtended.cpp:72](../src/bot-opos6ul/OPOS6UL_AsservExtended.cpp) | Après `setPositionAndColor`, attendre que la Nucleo applique la pose et qu'une frame CBOR mette `sharedPosition` à jour. Sans ça, `Navigator::line()` capture `pBefore = (0, 0)` → calcul `d_parcourue` faux (mais robot bouge bien physiquement). |
| Sleep 200ms inter-task `StrategyJsonRunner` | [src/common/ia/StrategyJsonRunner.cpp `executeInstruction`](../src/common/ia/StrategyJsonRunner.cpp) | Race entre fin d'une commande et envoi de la suivante : la Nucleo peut perdre la 2e cmd si arrive trop vite après la fin de la 1re (overflow Rx buffer ChibiOS pendant `onTimer` finalisation). 200ms entre 2 tasks consécutives. |
| Reset decoder + flush serial post-tryReconnect | [src/driver-arm/AsservCborDriver.cpp:154-165](../src/driver-arm/AsservCborDriver.cpp) | Après le 1er readBytes(500ms) qui confirme la connexion, on **jette** ces bytes (peuvent être au milieu d'une trame ou résidus boot). `serial_.flushReceiver()` + `frameDecoder_.reset()` pour repartir d'un état propre. |
| Phase tirette splittée ARMED → PRIMED | [src/common/Robot.hpp](../src/common/Robot.hpp) enum `MatchPhase` | Voir [O_STATE_NEW_INIT.md](O_STATE_NEW_INIT.md). Évite la confusion "tirette inserée puis retirée" comme un seul état. |
| Flush `matchState` aux transitions | `O_State_NewInit::execute()` boot + après PRIMED→MATCH | Push `matchState=0` au boot pour sortir le LCD tactile balise du `show_match_screen` du run précédent. Push `matchState=3` après tirette retirée pour déclencher le screen match Teensy. |
| `setupActivitiesZone2026` déplacé en branche legacy commentée | [src/bot-opos6ul/O_State_DecisionMakerIA.cpp](../src/bot-opos6ul/O_State_DecisionMakerIA.cpp) | En mode JSON (PMX1/2/3), plus d'erreur "strategy 'PMX1' inconnue". `initPlayground()` reste appelé (utile pour `MOVEMENT/PATH_TO`). Branche legacy ia_start dans `#if 0`. |
| `SensorsThread` démarré au début match JSON | `O_State_DecisionMakerIA::execute()` mode JSON | Détection adverse active uniquement à partir du début match (après tirette retirée). Pas de détection pendant CONFIG/ARMED/PRIMED/setPos. |
| Init JSON auto-load par défaut PMX1 | [src/common/Robot.cpp:268-276](../src/common/Robot.cpp) | Sans `/s` explicite, charger quand même `initPMX1.json` (best-effort, warn si absent). Fix oubli depuis migration "default = PMX1". |
| `ax12_init()` skip si servos absents | [src/bot-opos6ul/OPOS6UL_ActionsExtended.hpp:317](../src/bot-opos6ul/OPOS6UL_ActionsExtended.hpp) | Membre `servosAx12Connected_` set par `start()` au boot. Évite ~2s de timeout par appel `ax12_init_banderole()` sans servo. |
| Loggers menu/state activés | [src/bot-opos6ul/LoggerInitialize.cpp](../src/bot-opos6ul/LoggerInitialize.cpp) | `O_State_NewInit`, `MenuController`, `MenuShieldLCD`, `MenuBeaconLCDTouch` log INFO sur console. Cause du bug "setPos déclenché tout seul" identifiée (clic accidentel SELECT). |

### 1.2 Merge `asserv_chibios` master → PMX2026

Branche PMX2026 mergée avec `origin/master` (commit `50a3832` "current_index logic" + `3a2c3de` "set_position fix") :
- `m_current_index` persiste après finition de la commande (clé du handshake cmd_id ack).
- `set_position` opcode déplacé de **26 → 31** (zone four-param, 3 floats).
- Côté OPOS6UL : `CMD_SET_POSITION = 31` aligné dans [src/driver-arm/AsservCborDriver.cpp:47](../src/driver-arm/AsservCborDriver.cpp).
- Fix `QuadratureEncoder::GpioPinInit` PMX `main.cpp` : ajout `&QEID3` / `&QEID2` explicites (commit fb81f0b).

## 2. Bugs intermittents qui restent

### 2.1 Bug A — `set_position` parfois pas appliquée (intermittent)

**Symptôme** : après `tryReconnect` + sleep 1s, on envoie `setPositionAndColor(230, 130, 90°)`. Parfois la Nucleo n'applique pas la pose. Robot croit être à `(0, 0, 0°)`. Premier `LINE 80mm` part en X au lieu de Y.

**Diagnostic actuel** :
- Cas Nucleo UP au boot OPOS6UL → marche systématiquement.
- Cas Nucleo allumée APRÈS message "WAIT NUCLEO..." → **intermittent** (1 essai sur 2-3 environ).
- `received_cmd=3` après LINE 80 OK, mais position absolue = `(0+78, 0)` = `(78, 0)`. Donc `set_position` est arrivée avant LINE 80 mais n'a pas été appliquée par la Nucleo.

**Hypothèses non confirmées** :
1. Bytes parasites du boot Nucleo dans le buffer Rx Nucleo désynchronisent son décodeur CBOR. La 1re trame `set_position` est rejetée (CRC fail). Les trames suivantes (`LINE 80`) re-synchronisent sur `SYNC_WORD` et passent.
2. Race ChibiOS : threads `commandInput` et `AsservMain::onTimer` pas encore stabilisés en priorité. La 1re cmd est manquée.

**Workaround actuel** : sleep 1s stabilisation + reset decoder/flush côté ARM. Aide mais ne résout pas à 100%.

**Vraie solution** : handshake explicite (voir §3.1).

### 2.2 Bug B — 2e LINE perdue après allumage tardif Nucleo (intermittent)

**Symptôme** : LINE 80 OK + LINE 200 timeout 10s avec `target_cmd=4 received_cmd=3 status=0`. La Nucleo ne consomme pas la cmd 4.

**Diagnostic** :
- `received_cmd=3` → la 1re cmd a été ackée (`m_current_index=3`).
- `target_cmd=4` → l'OPOS6UL a bien envoyé la 2e cmd CBOR.
- `status=0` (IDLE) → la Nucleo est repassée au repos sans avoir consommé la cmd 4.
- `d_parcourue=2` → robot quasi-immobile sur la 2e LINE.

**Hypothèse** : overflow buffer Rx Nucleo pendant la transition fin-mouvement → IDLE. Le thread `commandInput` est mis en pause par `onTimer` qui finalise la régulation, et les bytes de la cmd 4 dépassent la taille du buffer hardware avant d'être lus.

**Workaround actuel** : sleep 200ms inter-task dans `StrategyJsonRunner::executeInstruction`. Le user a confirmé que ça stabilise (essais OK).

**Vraie solution** :
- Augmenter la taille du buffer série Rx côté firmware Nucleo (modif ChibiOS USART config).
- Ou : implémenter un protocole d'ACK applicatif (envoi → attente ACK avant d'enchainer).

## 3. TODO restants

### 3.1 Handshake explicite Nucleo (remplace les sleeps)

**Objectif** : remplacer les sleeps "magiques" (1s post-reconnect, 200ms post-setPositionAndColor, 200ms inter-task) par un protocole d'ACK déterministe.

**Plomberie déjà en place** :
- `nextCmdId_` / `lastReceivedCmdId_` atomiques côté `AsservCborDriver`.
- Receive thread CBOR met à jour `lastReceivedCmdId_` à chaque frame de position reçue (la Nucleo y inclut son `m_current_index` qui PERSISTE après finition grâce au merge master).
- Pas de modif firmware Nucleo nécessaire.

**Décomposition par type de commande** :

#### A. Commandes motion (LINE, ROTATE, FACE, GOTO, ORBITAL_TURN)

Double check :
1. **Wait ACK queue** (~ms après envoi) : `lastReceivedCmdId() >= sent_cmd_id`. Confirme que la Nucleo a poppé la commande de sa queue. **Élimine Bug B** (2e LINE perdue).
2. **Wait fin mouvement** (existe déjà) : `status == IDLE` après l'ack queue. Confirme que la trajectoire est terminée.

Pseudo-code :
```cpp
TRAJ_STATE Asserv::sendMotionAndWait(MovementType type, std::function<void()> sendFn, int timeout_queue_ms = 200)
{
    int targetCmdId;
    int retry = 0;
    constexpr int MAX_RETRY = 3;

    while (retry < MAX_RETRY) {
        sendFn();   // motion_Line / motion_Rotate / etc.
        targetCmdId = asservdriver_->lastSentCmdId();

        // Phase A : attendre ACK queue (la Nucleo a popé notre cmd)
        auto deadline = now() + timeout_queue_ms;
        while (now() < deadline) {
            if (asservdriver_->lastReceivedCmdId() >= targetCmdId) goto move_started;
            sleep_ms(5);
        }

        // Pas d'ACK -> commande probablement perdue (overflow Rx Nucleo). Retry.
        logger().warn() << "Motion cmd " << targetCmdId << " lost, retry " << retry+1 << logs::end;
        retry++;
    }
    return TRAJ_ERROR;   // 3 retries failed

move_started:
    // Phase B : attendre status==IDLE (existant via waitEndOfTrajWithDetection)
    return waitEndOfTrajWithDetection(type);
}
```

#### B. `set_position` (pas de cmd_id retour direct)

`set_position` n'incrémente pas le cmd_id Nucleo de la même façon que les motion (la cmd est traitée immédiatement dans `decode_cmd`, pas via `CommandManager::add`). Mais elle a quand même un `cmd_id` envoyé.

**Stratégie** : vérifier que `sharedPosition` (alimenté par les frames Nucleo) reflète la pose cible dans une fenêtre de tolérance.

```cpp
bool Asserv::setPositionAndWait(float x, float y, float theta_rad, int timeout_ms = 500)
{
    setPositionReal(x, y, theta_rad);   // envoi CBOR

    auto deadline = now() + timeout_ms;
    while (now() < deadline) {
        ROBOTPOSITION p = sharedPosition()->getRobotPosition();
        if (std::abs(p.x - x) < 50.0f &&            // 50mm tolerance
            std::abs(p.y - y) < 50.0f &&
            std::abs(p.theta - theta_rad) < 0.1f) {  // ~6 degres
            return true;
        }
        sleep_ms(20);
    }
    return false;   // timeout, retry par l'appelant
}
```

Couvre **Bug A** (set_position pas appliquée).

#### C. Reconnect

Après `tryReconnect`, envoyer un `emergency_stop_reset` (qui a un cmd_id) et attendre l'ACK. Si OK → la Nucleo accepte les commandes. Si timeout → retry tryReconnect.

Élimine le sleep 1s "stabilisation Nucleo".

**Étapes d'implémentation** :
1. **AsservCborDriver** : exposer `bool waitForCmdAck(int targetCmdId, int timeout_ms)` qui poll `lastReceivedCmdId()` jusqu'au target ou timeout.
2. **Asserv (wrapper)** : nouvelle méthode `sendMotionAndWait()` (motion) et `setPositionAndWait()` (set_position).
3. **Asserv::motion_Line / rotate / etc.** : passer par la nouvelle méthode (transparente pour les appelants Navigator).
4. **`waitForAsserv()`** : remplacer le sleep 1s post-reconnect par envoi `emergency_stop_reset` + waitForCmdAck.
5. **`OPOS6UL_AsservExtended::startMotionTimerAndOdo`** : remplacer le sleep 200ms par `setPositionAndWait` (ou un wait sur `motion_AssistedHandling` ack).
6. **`StrategyJsonRunner::executeInstruction`** : retirer le sleep 200ms inter-task (devient redondant grâce au wait ACK queue dans `sendMotionAndWait`).
7. **Tests** : valider sur 10+ essais consécutifs avec Nucleo allumée tardivement (pour stresser le cas glitch).

**Effort estimé** : 1-2h de codage + tests sur robot réel.

**Bénéfices** :
- Plus de bug intermittent (Bug A et Bug B).
- Tempo match plus rapide : ~1.4s gagnés sur l'init match (1s + 200ms + 200ms × N tasks).
- Robustesse industrielle, déterministe.

**Stratégie commit recommandée** :
1. **Commit 1 (cette session)** : tout ce qui est fait actuellement côté PMX-CORTEX (cmd_id ack, WAIT NUCLEO, sleeps workarounds, fixes JSON runner, etc.). Sécurise les ~3-4 jours de travail validés.
2. **Commit 2 (session dédiée handshake)** : implémentation handshake explicite + suppression des sleeps. Idéalement testé sur 10+ essais avant commit. Branche dédiée `feat/asserv-handshake-ack` puis merge main.
3. **Commit 3 (parallèle ou plus tard)** : merge master → PMX2026 dans `asserv_chibios` (déjà préparé en working tree).

### 3.2 Bug LCDshield blanc après "WAIT NUCLEO..." (à investiguer)

Signalé par l'utilisateur : après "WAIT NUCLEO..." et allumage Nucleo, le LCDshield n'affiche plus son menu. À reproduire et confirmer.

**Pistes** :
- `MenuShieldLCD::alive_` peut basculer à false sur erreur I2C, jamais reverse.
- `lastLine0_` / `lastLine1_` du shield mal initialisés après le bypass `lcd2x16().clear() + print("WAIT NUCLEO...")` direct.
- Race entre `pollInputsOnly` (utilisé pendant wait) et le `refreshDisplay` qui reprend.

### 3.3 Phase 2 — Zones d'évitement dynamiques

Voir [ASSERV_MIGRATION_COMMUNICATION.md §Phase 2](ASSERV_MIGRATION_COMMUNICATION.md) pour la philosophie.

**3.3.1 Implémenter `ELEMENT/ADD_ZONE` et `DELETE_ZONE` dans le runner JSON**

Actuellement [StrategyJsonRunner.cpp:186-189](../src/common/ia/StrategyJsonRunner.cpp) :
```cpp
if (t.type == "ELEMENT") {
    logger().info() << "[STUB Phase 2] ELEMENT/" << t.subtype << " ...";
    return TRAJ_FINISHED;
}
```

→ Implémenter :
```cpp
if (t.type == "ELEMENT") {
    if (t.subtype == "ADD_ZONE" && t.item_id) {
        iap_->playground()->addZone(*t.item_id);
    } else if (t.subtype == "DELETE_ZONE" && t.item_id) {
        iap_->playground()->removeZone(*t.item_id);
    }
    return TRAJ_FINISHED;
}
```

Vérifier que `Playground::addZone` / `removeZone` existent ou les implémenter.

**3.3.2 Auto-publier zone d'évitement adversaire depuis `Sensors::lastDetection()`**

Quand la détection beacon ToF voit un adversaire à `(x_adv, y_adv)`, créer dynamiquement une zone d'évitement temporaire dans le `Playground` du pathfinding. Retirée quand l'adversaire bouge ou disparaît.

Probablement un thread/timer qui poll `Sensors::lastDetection()` à 5-10 Hz et synchronise avec une zone nommée (ex: `zone_adv_dynamic`).

## 4. Commits à venir

Tout est encore en working tree, **pas commité** :
- `/home/pmx/git/asserv_chibios` (PMX2026) : merge master déjà préparé + résolution conflits + fix QuadratureEncoder PMX. Message commit déjà rédigé (cf historique de session).
- `/home/pmx/git/PMX-CORTEX` : ~25 fichiers modifiés cumulant tous les fixes ci-dessus + docs MD. Message commit à composer.

## 5. Tests à valider avant clôture

- [ ] Match complet (boot → tirette → match → fin) avec Nucleo UP au boot, 3-5 essais consécutifs.
- [ ] Match complet avec Nucleo allumée pendant le message "WAIT NUCLEO...", 3-5 essais consécutifs.
- [ ] Stratégies PMX1, PMX2, PMX3 chargées correctement (init JSON + strategy JSON).
- [ ] `O_AsservLineRotateTest` (test fonctionnel) en simu et hardware.
- [ ] `O_DetectionScenarioTest` (forward et backward) en simu — détection beacon active.
- [ ] LCD tactile balise revient bien sur l'écran menu au boot (pas resté sur le `show_match_screen` précédent).
- [ ] Bouton SETPOS/RESET du LCD touch bien en accord avec la phase (CONFIG/ARMED/PRIMED/MATCH).
