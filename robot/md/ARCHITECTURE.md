# Architecture robot/ — PMX-CORTEX

## Migrations effectuées

- **Timers POSIX : scheduler unifié à thread persistant** (avril 2026) — Refactoring complet du système de timers suite à la découverte d'un bug historique de deadlock. 3 étapes :
  1. **SIGEV_SIGNAL → SIGEV_THREAD** (étape intermédiaire) pour corriger le deadlock (signal handler qui tenait un mutex).
  2. **SIGEV_THREAD → thread persistant** car glibc créait un nouveau pthread à chaque expiration → OOM kill à haute fréquence (10ms).
  3. **Scheduler unifié `ActionTimerScheduler`** : 1 seul thread persistant pour toutes les `IAction` + tous les `ITimerScheduledListener`. Voir [section détaillée](#migration-sigev_signal--sigev_thread--actiontimerscheduler-avril-2026-). Même bug présent sur l'ancien robot EV3/PMX — voir [rétrospective EV3](#bug-historique-ev3--deadlock-sigalrm-rétrospective).
- **Démarrage asserv : `setPositionAndColor` avant `startMotionTimerAndOdo`** (avril 2026) — Le set position reset la Nucleo et définit la couleur de match ; rien ne doit fonctionner avant. Filet de sécurité : flag `positionInitialized_` dans `AsservCborDriver`. Voir [section détaillée](#démarrage-asserv--ordre-setpositionandcolor--startmotiontimerandodo).
- **Clôture propre threads + SVG valide** (avril 2026) — Correction du SVG invalide (éléments `<circle>` écrits après `</svg>`) causé par le thread CBOR qui continuait à tourner pendant la fermeture du SVG. Voir [section détaillée](#clôture-propre--thread-cbor--svg-valide).
- **Source de vérité unique pour la position robot** (avril 2026) — `Asserv::pos_getPosition()` lit directement depuis `sharedPosition` (alimentée par `setPositionReal()` + thread CBOR), au lieu du `p_` local du driver qui avait ~50ms de latence au démarrage.
- **Configuration hardware dynamique** (avril 2026) — Singleton `HardwareConfig` + fichier `hardware.conf` à côté de l'exécutable. Les 7+ factory `create()` consultent `isEnabled()` pour instancier le driver ARM réel ou le stub simu. `Main.cpp` charge la config avant la création du robot. Voir [HARDWARE_CONFIG.md](HARDWARE_CONFIG.md) (reste : lib `pmx-driver-stub` séparée pour le build SIMU pur).
- **SensorsTimer → SensorsThread** (avril 2026) — Le timer de lecture capteurs/beacon est devenu un thread dédié (`SensorsThread : public utils::Thread`) pour éviter les contraintes de callback timer et permettre un sleep contrôlé entre les lectures I2C.
- **Navigator : classe de navigation unifiée** (avril 2026) — Nouvelle classe `Navigator` avec retry automatique (`executeWithRetry`), waypoints, pathfinding A*. Centralise la logique de déplacement qui était dispersée entre `IAbyPath`, `IAbyZone` et `Asserv`. Migration des appelants : suppression des `while*` de IAbyPath et Robot.
- **Refactoring nommage unifié mouvements** (avril 2026) — Nommage cohérent Driver/Asserv/Navigator : `goBackTo`, `faceBackTo` + combos `FaceBackTo`. Uniformisation des signatures sur les 3 couches.
- **DetectionEvent + waitEndOfTrajWithDetection** (avril 2026) — Structure `DetectionEvent` (level, position, timestamp) publiée par `SensorsThread`. Boucle centrale `waitEndOfTrajWithDetection(MovementType)` dans `Asserv` qui consulte le DetectionEvent en temps réel. Suppression de `warnFrontDetectionOnTraj` et flags `temp_*`. Voir [SENSORS_DETECTION_MIGRATION.md](SENSORS_DETECTION_MIGRATION.md) phases 0-4.
- **Synchronisation beacon** (avril 2026) — Buffer circulaire positions + timestamps Teensy (`t1-t4_us`, `seq`). `posAtSync` capture la position robot au moment du sync I2C. `getPositionAt(t_mesure_ms)` retrouve la position historique pour la projection beacon→table. Réduit l'erreur de ~100mm à ~10mm.
- **AsI2cAtomic — wrapper I2C avec repeated start** (avril 2026) — Header-only `AsI2cAtomic.hpp` pour lectures I2C atomiques avec repeated start et bus recovery. Utilisé par `BeaconSensors`, `Adafruit_MCP23017`, et tous les drivers I2C.
- **Télémétrie UDP** (avril 2026) — Envoi position robot + adversaires en JSON Lines sur UDP port 9870, flush toutes les 300ms. Appender configurable via `/i ip` et `/p port` en CLI. Voir [Télémétrie.md](Télémétrie.md).
- **O_State_NewInit — menu multi-sources** (avril 2026) — Remplacement de `O_State_Init` par un système multi-sources (`MenuController` + `IMenuSource`). Supporte LCD shield 2x16 + LCD tactile balise en parallèle avec dégradation gracieuse. Source de vérité unique dans `Robot` (phase-aware setters). Architecture `syncFull()` → `pollInputs()` → `refreshDisplay()`. Voir [O_STATE_NEW_INIT.md](O_STATE_NEW_INIT.md) (étapes 1-7, 9 faites ; étape 8 partielle ; étape 10 cleanup restant).

## Migrations en cours

- [Communication asserv](ASSERV_MIGRATION_COMMUNICATION.md) — Phases 2-4, 6 restantes : commandes manquantes (n/N/!/R), nettoyage dead code, gotoChain non-bloquant
- [Détection d'obstacles](SENSORS_DETECTION_MIGRATION.md) — Phases 5-7 restantes : visualisation SVG L/R, simulation adversaire UDP, isOnPath pour reprise intelligente
- [Migration Goto externe + isOnPath](ISONPATH_GOTO_MIGRATION.md) — T1-T5 faits (prédicat `isOnPath` géométrique + 15 tests TDD + suppression `rotate_ignoring_opponent`). Restants : T6 (ToF c1-c8), T7 (CBOR Goto/GotoNoStop), T8 (intégration runtime dans `waitEndOfTrajWithDetection`), MoveMode, `Navigator::executeChain`.

### Règles de migration PMX → PMX-CORTEX

1. **Validation obligatoire** — Chaque modification doit être validée par l'utilisateur avant d'être appliquée. Aucun changement sans confirmation et explication pertinente.
2. **Conserver la configuration robot PMX** — Les paramètres, réglages et configurations du robot PMX doivent être préservés tels quels. Aucune modification de configuration sans validation et justification claire.
3. **Dossier `old/`** — Les fichiers remplacés ne sont pas supprimés : ils sont déplacés dans un dossier `old/` au même niveau. Cela permet de comparer et de revenir en arrière. On nettoiera une fois que tout fonctionne.
4. **Comparaison PMX2025** — En cas de doute, comparer avec la version PMX2025 (ancien projet `/home/pmx/git/PMX/`).
5. **Vérification PlotJuggler** — Après chaque étape qui compile et fonctionne, vérifier que les courbes d'asservissement sont toujours disponibles dans PlotJuggler (`plotjuggler_asservstream_plugin`). La communication série et le format des données doivent rester compatibles.

## Hardware status LEDs

Au démarrage, les 8 LEDs vertes s'allument (lamp test 500ms), puis chacune s'éteint si le composant est OK. **LED allumée = erreur.**


| LED | Composant        | Bus               | Classe driver      |
| ----- | ------------------ | ------------------- | -------------------- |
| 0   | LcdShield        | MCP23017 I2C      | `LcdShieldDriver`  |
| 1   | Tirette/Switch   | PCA9555 I2C       | `SwitchDriver`     |
| 2   | BeaconSensors    | Teensy I2C        | `SensorsDriver`    |
| 3   | GroveColorSensor | TCS3414 I2C       | `GroveColorSensor` |
| 4   | Servos AX12      | Teensy CCAx12 I2C | `ServoDriver`      |
| 5   | *(réserve)*     |                   |                    |
| 6   | *(réserve)*     |                   |                    |
| 7   | AsservDriver     | Nucleo série USB | `AsservDriver`     |

Tous les logs de diagnostic sont préfixés `Hardware status:` (filtrable via `grep "Hardware status"`).

Code source : `OPOS6UL_ActionsExtended::OPOS6UL_ActionsExtended()`.

## Structure des dossiers

```
robot/
├── CMakeLists.txt                      # Build unique, sélection par plateforme
├── CMakePresets.json                   # Presets : simu-debug, simu-release, arm-debug, arm-release
├── cmake/
│   └── toolchain-arm-opos6ul.cmake
│
├── src/
│   ├── common/                         # Toujours compilé (ARM + SIMU)
│   │   │
│   │   ├── interface/                  # Interfaces abstraites drivers
│   │   │   ├── AActionDriver.hpp
│   │   │   ├── AAsservDriver.hpp
│   │   │   ├── AButtonDriver.hpp
│   │   │   ├── AColorDriver.hpp
│   │   │   ├── ALcdShieldDriver.hpp
│   │   │   ├── ALedDriver.hpp
│   │   │   ├── ARobotPositionShared.hpp
│   │   │   ├── ASensorsDriver.hpp
│   │   │   ├── AServoDriver.hpp
│   │   │   ├── AServoUsingMotorDriver.hpp
│   │   │   ├── ASoundDriver.hpp
│   │   │   └── ASwitchDriver.hpp
│   │   │
│   │   ├── geometry/                    # Géométrie table et zones obstacles
│   │   │   ├── TableGeometry.hpp         # Dimensions table + isPointInsideTable()
│   │   │   └── ObstacleZone.cpp/hpp      # Classification obstacles par zones (logique pure)
│   │   │
│   │   ├── action/                     # Gestion actions et périphériques
│   │   │   ├── AActionsElement.hpp
│   │   │   ├── ActionManagerTimer.cpp/hpp  # ◆ THREAD (event-driven, sem_wait)
│   │   │   ├── Actions.cpp/hpp
│   │   │   ├── ButtonBar.cpp/hpp
│   │   │   ├── IAction.hpp
│   │   │   ├── ITimerListener.hpp
│   │   │   ├── ITimerPosixListener.hpp
│   │   │   ├── LcdShield.cpp/hpp
│   │   │   ├── LedBar.cpp/hpp            # ◆ TIMER onTimer (variable µs) — animation LEDs
│   │   │   ├── Sensors.cpp/hpp           # ◆ THREAD SensorsThread (sleep variable ms) — lecture beacon + détection obstacle
│   │   │   ├── ServoObjectsSystem.cpp/hpp # ◆ TIMER onTimer (50ms) — interpolation servos
│   │   │   ├── ServoUsingMotor.cpp/hpp
│   │   │   ├── SoundBar.cpp/hpp
│   │   │   ├── TimerPosix.cpp/hpp
│   │   │   └── Tirette.cpp/hpp
│   │   │
│   │   ├── asserv/                     # Logique mouvement
│   │   │   ├── Asserv.cpp/hpp
│   │   │   ├── EncoderControl.cpp/hpp
│   │   │   ├── MotorControl.cpp/hpp
│   │   │   └── MovingBase.cpp/hpp
│   │   │
│   │   ├── asserv-esial/               # Bibliothèque asserv ESIAL
│   │   │   ├── AsservEsialR.cpp/hpp    # ◆ THREAD + LOOP (20Hz / 50ms, PID + odométrie)
│   │   │   ├── config.default.txt
│   │   │   ├── codeurs/
│   │   │   │   ├── BotCodeurs.cpp/h
│   │   │   │   ├── Codeur.cpp/h
│   │   │   │   ├── CodeursDirects.cpp/h
│   │   │   │   └── CodeursInterface.h
│   │   │   ├── commandManager/
│   │   │   │   ├── CommandManagerA.cpp/h
│   │   │   │   └── CMDList/
│   │   │   │       └── CMDList.cpp/h
│   │   │   ├── config/
│   │   │   │   ├── config.cpp/h
│   │   │   │   ├── parameter.h
│   │   │   │   └── params.h
│   │   │   ├── consignController/
│   │   │   │   └── ConsignController.cpp/h
│   │   │   ├── filtres/
│   │   │   │   ├── Filtre.h
│   │   │   │   ├── Pid/
│   │   │   │   │   └── Pid.cpp/h
│   │   │   │   └── QuadRampDerivee/
│   │   │   │       └── QuadRampDerivee.cpp/h
│   │   │   ├── motorsController/
│   │   │   │   ├── BotMotors.cpp/h
│   │   │   │   ├── DummyMotorsController.h
│   │   │   │   └── MotorsController.h
│   │   │   ├── odometrie/
│   │   │   │   └── Odometrie.cpp/h
│   │   │   ├── regulateur/
│   │   │   │   └── Regulateur.cpp/h
│   │   │   └── Utils/
│   │   │       └── Utils.cpp/h
│   │   │
│   │   ├── state/                      # Machine d'état
│   │   │   ├── AAutomateState.hpp
│   │   │   ├── Automate.cpp/hpp
│   │   │   └── IAutomateState.hpp
│   │   │
│   │   ├── ia/                         # Intelligence artificielle / stratégie
│   │   │   ├── IA.cpp/hpp
│   │   │   ├── IAbyPath.cpp/hpp
│   │   │   ├── IAbyZone.cpp/hpp
│   │   │   └── IACommon.hpp
│   │   │
│   │   ├── log/                        # Logging
│   │   │   ├── LoggerFactory.cpp/hpp   # ◆ THREAD + LOOP (flush appenders en continu)
│   │   │   ├── Logger.cpp/hpp
│   │   │   ├── Level.cpp/hpp
│   │   │   ├── Exception.hpp
│   │   │   ├── SvgWriter.cpp/hpp
│   │   │   ├── backward.cpp/hpp
│   │   │   ├── Appender/              # Sous-dossier appenders
│   │   │   └── resources-svg/         # Ressources SVG
│   │   │
│   │   ├── thread/                     # Thread POSIX wrapper
│   │   │   ├── Thread.cpp/hpp
│   │   │   └── Mutex.cpp/hpp
│   │   │
│   │   ├── utils/                      # Utilitaires
│   │   │   ├── Chronometer.cpp/hpp
│   │   │   ├── ConsoleKeyInput.hpp
│   │   │   ├── File.hpp
│   │   │   ├── Hex.hpp
│   │   │   ├── itoa.cpp/hpp
│   │   │   ├── json.hpp
│   │   │   ├── PointerList.hpp
│   │   │   └── WString.cpp/hpp
│   │   │
│   │   ├── Arguments.cpp/hpp           # Parsing arguments CLI
│   │   ├── ConsoleManager.cpp/hpp      # Menu console interactif
│   │   ├── FunctionalTest.hpp          # Interface test fonctionnel
│   │   ├── Print.cpp/hpp               # Helpers d'affichage
│   │   ├── Printable.hpp
│   │   └── Robot.cpp/hpp               # Classe de base Robot
│   │
│   ├── driver-arm/                     # Implémentations ARM OPOS6UL
│   │   ├── AsservDriver.cpp/hpp        #   serial /dev/ttymxc1  ◆ THREAD + LOOP (~10Hz / 100ms, lecture série carte ST)
│   │   ├── SensorsDriver.cpp/hpp       #   agrégateur capteurs I2C
│   │   ├── ServoDriver.cpp/hpp         #   Dynamixel AX-12
│   │   ├── ServoUsingMotorDriver.cpp/hpp
│   │   ├── LedDriver.cpp/hpp           #   LEDs hardware
│   │   ├── ButtonDriver.cpp/hpp        #   boutons physiques
│   │   ├── SwitchDriver.cpp/hpp        #   switchs
│   │   ├── SoundDriver.cpp/hpp         #   son
│   │   ├── LcdShieldDriver.cpp/hpp     #   écran LCD I2C
│   │   ├── ActionDriver.cpp            #   actions hardware
│   │   ├── RobotPositionShared.cpp/hpp #   position partagée
│   │   ├── serialib.cpp/hpp            #   lib série cross-platform
│   │   ├── GpioPCA9555.cpp/hpp         #   GPIO expander I2C
│   │   ├── MD25.cpp/hpp                #   contrôleur moteurs I2C (0x58)
│   │   ├── DynamixelDriver.cpp/hpp     #   protocole Dynamixel
│   │   ├── CCAx12Teensy.cpp/hpp        #   AX-12 via Teensy I2C (0x08)
│   │   ├── CCAx12Adc.cpp/hpp           #   ADC via I2C
│   │   ├── Gp2y0e02b.cpp/hpp           #   capteur distance Sharp I2C (0x40)
│   │   ├── IrSensor.cpp/hpp            #   capteurs IR
│   │   ├── BeaconSensors.cpp/hpp       #   balises I2C (0x2D)
│   │   ├── GroveColorSensor.cpp/hpp    #   capteur couleur
│   │   ├── AsservDriver_mbed_i2c.cpp/hpp  # variante asserv mbed I2C  ◆ THREAD + LOOP
│   │   ├── Adafruit_MCP23017.cpp/hpp   #   GPIO expander Adafruit
│   │   ├── Adafruit_PWMServoDriver.cpp/h  # PWM servo Adafruit
│   │   └── Adafruit_RGBLCDShield.cpp/hpp  # LCD RGB Adafruit
│   │
│   ├── driver-simu/                    # Implémentations simulation (PC)
│   │   ├── AsservDriver.cpp/hpp        #   simulation moteurs  ◆ THREAD + LOOP
│   │   ├── SensorsDriver.cpp/hpp       #   capteurs simulés
│   │   ├── ServoDriver.cpp/hpp         #   stubs servos
│   │   ├── ServoUsingMotorDriver.cpp/hpp
│   │   ├── LedDriver.cpp/hpp           #   stubs LEDs
│   │   ├── ButtonDriver.cpp/hpp        #   stubs boutons
│   │   ├── SwitchDriver.cpp/hpp        #   stubs switchs
│   │   ├── SoundDriver.cpp/hpp         #   stubs son
│   │   ├── LcdShieldDriver.cpp/hpp     #   stubs LCD
│   │   └── RobotPositionShared.cpp/hpp #   position simulée
│   │
│   └── bot/
│       ├── opos6ul/                    # Robot principal OPOS6UL (stable)
│       │   ├── Main.cpp                #   point d'entrée + LoggerInitialize
│       │   ├── LoggerInitialize.cpp
│       │   ├── OPOS6UL_RobotExtended.cpp/hpp    # singleton robot
│       │   ├── OPOS6UL_ActionsExtended.cpp/hpp   # bindings devices
│       │   ├── OPOS6UL_AsservExtended.cpp/hpp    # asserv + filtres
│       │   ├── OPOS6UL_IAExtended.cpp/hpp        # IA / stratégie
│       │   ├── OPOS6UL_SvgWriterExtended.cpp/hpp # logging SVG
│       │   ├── config_OPOS6UL_Robot.txt           # calibration PID, ticks/m, roues
│       │   │
│       │   ├── states/                 # Machine d'état match (change chaque année)
│       │   │   ├── O_State_DecisionMakerIA.cpp/hpp   # ◆ THREAD (stratégie, exécution unique)
│       │   │   ├── O_State_Init.cpp/hpp
│       │   │   └── O_State_WaitEndOfMatch.cpp/hpp
│       │   │
│       │   └── tests/                  # Tests fonctionnels spécifiques robot
│       │       ├── O_ActionManagerTimerTest.cpp/hpp  # ◆ TIMER onTimer (100-500ms) — test seulement
│       │       ├── O_AsservTest.cpp/hpp
│       │       ├── O_AsservEsialTest.cpp/hpp
│       │       ├── O_AsservLineRotateTest.cpp/hpp
│       │       ├── O_AsservXYRotateTest.cpp/hpp
│       │       ├── O_Asserv_CalageTest.cpp/hpp
│       │       ├── O_Asserv_SquareTest.cpp/hpp
│       │       ├── O_ButtonBarTest.cpp/hpp
│       │       ├── O_GroveColorTest.cpp/hpp
│       │       ├── O_IAbyPathTest.cpp/hpp
│       │       ├── O_LcdBoardTest.cpp/hpp
│       │       ├── O_LedBarTest.cpp/hpp
│       │       ├── O_SensorsTest.cpp/hpp
│       │       ├── O_ServoObjectsTest.cpp/hpp
│       │       ├── O_ServoStepTest.cpp/hpp
│       │       └── O_TiretteTest.cpp/hpp
│       │
│       └── bot2/                       # Exemple : 2e robot (autre config)
│           ├── Main.cpp
│           ├── LoggerInitialize.cpp
│           ├── Bot2_RobotExtended.cpp/hpp
│           ├── Bot2_ActionsExtended.cpp/hpp
│           ├── Bot2_AsservExtended.cpp/hpp
│           ├── Bot2_IAExtended.cpp/hpp
│           ├── Bot2_SvgWriterExtended.cpp/hpp
│           ├── config_Bot2_Robot.txt
│           ├── states/
│           │   ├── B2_State_DecisionMakerIA.cpp/hpp
│           │   ├── B2_State_Init.cpp/hpp
│           │   └── B2_State_WaitEndOfMatch.cpp/hpp
│           └── tests/
│               └── B2_*Test.cpp/hpp
│
├── test/
│   ├── suite/                          # Framework de test
│   │   ├── UnitTest.cpp/hpp            #   classe abstraite test
│   │   ├── UnitTestSuite.cpp/hpp       #   runner / agrégateur
│   │   └── UnitTestAppender.cpp/hpp    #   appender logging test
│   │
│   ├── common/                         # Common-UnitTest (logique pure, pas de driver)
│   │   ├── Main.cpp
│   │   ├── LoggerInitialize.cpp
│   │   ├── ActionManagerTimerTest.cpp/hpp
│   │   ├── ChronometerTest.cpp/hpp
│   │   ├── LoggerTest.cpp/hpp
│   │   ├── ThreadTest.cpp/hpp
│   │   ├── RetryPolicyTest.cpp/hpp
│   │   ├── TableGeometryTest.cpp/hpp   #   géométrie table (stub position local)
│   │   └── ObstacleZoneTest.cpp/hpp    #   classification obstacles (logique pure)
│   │
│   ├── driver/                         # Driver-UnitTest (contrat interfaces, ARM+SIMU)
│   │   ├── Main.cpp
│   │   ├── LoggerInitialize.cpp
│   │   ├── LedDriverTest.cpp/hpp
│   │   ├── SwitchDriverTest.cpp/hpp
│   │   ├── SensorsDriverTest.cpp/hpp
│   │   └── NavigatorTest.cpp/hpp       #   (SIMU seulement)
│   │
│   └── main_test.cpp                  # Test validation plateforme (existant)
│
└── cmake/
    └── toolchain-arm-opos6ul.cmake
```

## Targets CMake

```
                    pmx-common              (STATIC, toujours compilé)
                   /          \
        pmx-driver-arm    pmx-driver-simu   (STATIC, un seul selon plateforme)
                   \          /
            ┌───────┴──────────┐
            │                  │
        pmx-suite          bot/
       /         \         /    \
  common-test  driver-test  opos6ul  bot2   (exécutables)
```

### Détail des targets


| Target            | Type   | Sources              | Dépendances                |
| ------------------- | -------- | ---------------------- | ----------------------------- |
| `pmx-common`      | STATIC | `src/common/**`      | pthread                     |
| `pmx-driver-arm`  | STATIC | `src/driver-arm/**`  | pmx-common, as_devices      |
| `pmx-driver-simu` | STATIC | `src/driver-simu/**` | pmx-common                  |
| `pmx-suite`       | STATIC | `test/suite/**`      | pmx-common                  |
| `common-test`     | EXE    | `test/common/**`     | pmx-suite +**driver auto**  |
| `driver-test`     | EXE    | `test/driver/**`     | pmx-suite +**driver auto**  |
| `opos6ul`         | EXE    | `src/bot/opos6ul/**` | pmx-common +**driver auto** |
| `bot2`            | EXE    | `src/bot/bot2/**`    | pmx-common +**driver auto** |

### Sélection automatique du driver

```cmake
if(CMAKE_CROSSCOMPILING)          # arm-debug ou arm-release
    set(DRIVER_LIB pmx-driver-arm)
else()                            # simu-debug ou simu-release
    set(DRIVER_LIB pmx-driver-simu)
endif()
```

### Ajouter un nouveau robot

Pour ajouter `bot2`, il suffit de :

1. Créer `src/bot/bot2/` avec ses propres `*Extended` et `config.txt`
2. Ajouter dans CMakeLists.txt :

```cmake
add_executable(bot2 src/bot/bot2/Main.cpp src/bot/bot2/Bot2_RobotExtended.cpp ...)
target_link_libraries(bot2 pmx-common ${DRIVER_LIB} pthread)
```

Chaque robot a sa propre calibration, ses propres actions, sa propre IA.
Ils partagent `pmx-common` (Robot, Asserv, Actions base) et le même driver.

## Correspondance avec l'ancien projet PMX (Eclipse)


| Ancien projet Eclipse              | Nouveau dans robot/                                                                   |
| ------------------------------------ | --------------------------------------------------------------------------------------- |
| `Common-UnitTest_SIMU`             | `cmake --preset simu-debug && cmake --build --preset simu-debug --target common-test` |
| `Common-UnitTest_OPOS6UL_ARM`      | `cmake --preset arm-debug && cmake --build --preset arm-debug --target common-test`   |
| `Driver-UnitTest_SIMU`             | `cmake --preset simu-debug && cmake --build --preset simu-debug --target driver-test` |
| `Driver-UnitTest_OPOS6UL_ARM`      | `cmake --preset arm-debug && cmake --build --preset arm-debug --target driver-test`   |
| `Bot_ArmadeusOPOS6UL_SIMU`         | `cmake --build --preset simu-debug --target opos6ul`                                  |
| `Bot_ArmadeusOPOS6UL_ARM`          | `cmake --build --preset arm-debug --target opos6ul`                                   |
| Virtual folders (linked resources) | `target_link_libraries()` dans CMake                                                  |
| Sélection du projet à builder    | `--preset` + `--target`                                                               |

## Correspondance fichiers ancien PMX → nouveau


| Ancien (PMX/src/)          | Nouveau (robot/src/)                             |
| ---------------------------- | -------------------------------------------------- |
| `Common/Interface.Driver/` | `common/interface/`                              |
| `Common/Action/`           | `common/action/`                                 |
| `Common/Asserv/`           | `common/asserv/`                                 |
| `Common/State/`            | `common/state/`                                  |
| `Common/IA/`               | `common/ia/`                                     |
| `Common/Utils/`            | `common/utils/`                                  |
| `Common/*.cpp/hpp`         | `common/` (racine)                               |
| `Log/`                     | `common/log/`                                    |
| `Thread/`                  | `common/thread/`                                 |
| `Asserv.Esial/`            | `common/asserv-esial/`                           |
| `Driver-OPOS6UL_ARM/`      | `driver-arm/`                                    |
| `Driver-SIMU/`             | `driver-simu/`                                   |
| `Bot-OPOS6UL/`             | `bot/opos6ul/`                                   |
| `Bot-OPOS6UL.Main/`        | `bot/opos6ul/` (Main.cpp + LoggerInitialize.cpp) |


| Ancien (PMX/test/)              | Nouveau (robot/test/) |
| --------------------------------- | ----------------------- |
| `Suite/`                        | `suite/`              |
| `Common-Test.Main/`             | `common/`             |
| `Driver-Test_OPOS6UL_ARM.Main/` | `driver/`             |
| `Driver-Test_SIMU.Main/`        | `driver/`             |

## Commandes de build

```bash
# --- Tests unitaires ---

# Common-UnitTest SIMU (sur PC)
cmake --preset simu-debug
cmake --build --preset simu-debug --target common-test

# Driver-UnitTest SIMU (sur PC)
cmake --build --preset simu-debug --target driver-test

# Common-UnitTest ARM (cross-compile pour OPOS6UL)
cmake --preset arm-debug
cmake --build --preset arm-debug --target common-test

# Driver-UnitTest ARM (cross-compile pour OPOS6UL)
cmake --build --preset arm-debug --target driver-test

# --- Robot principal ---

# Robot OPOS6UL en SIMU (test sur PC)
cmake --build --preset simu-debug --target opos6ul

# Robot OPOS6UL en ARM (pour déployer sur la carte)
cmake --build --preset arm-release --target opos6ul

# --- Tout compiler d'un coup ---
cmake --build --preset simu-debug
```

## Position partagée du robot (ARobotPositionShared)

### Rôle

`ARobotPositionShared` est le **point central d'accès à la position du robot** sur la table.
Elle fournit un accès thread-safe en lecture/écriture à la structure `ROBOTPOSITION` (x, y, theta en mm/rad).

### Pattern singleton

L'implémentation (SIMU et ARM) utilise un **singleton statique** :

```cpp
ARobotPositionShared* ARobotPositionShared::create()
{
    static RobotPositionShared *instance = new RobotPositionShared();
    return instance;
}
```

**Conséquence importante** : `create()` retourne toujours la même instance. Ne **jamais** `delete` le pointeur retourné — c'est un singleton dont la durée de vie est celle du programme.

### Qui l'utilise


| Composant             | Usage                                          | Accès               |
| ----------------------- | ------------------------------------------------ | ---------------------- |
| Asserv / AsservDriver | Écriture de la position courante (odométrie) | `setRobotPosition()` |
| Sensors               | Lecture pour projeter les détections capteurs | `getRobotPosition()` |
| TableGeometry         | Lecture pour`isSensorReadingInsideTable()`     | `getRobotPosition()` |
| SvgWriter             | Lecture pour tracer la position                | `getRobotPosition()` |
| Robot                 | Propriétaire du singleton (`sharedPosition_`) | `sharedPosition()`   |

### Structure ROBOTPOSITION

```cpp
struct sRobotPosition {
    float x;                // Position X en mm
    float y;                // Position Y en mm
    float theta;            // Angle en radians
    int asservStatus;       // 0=idle, 1=running, 2=emergency stop, 3=blocked
    unsigned int queueSize; // Commandes en file d'attente
    unsigned int debug_nb;  // Compteur debug
};
```

### Utilitaires dans le même header


| Fonction                                  | Usage                                                          |
| ------------------------------------------- | ---------------------------------------------------------------- |
| `degToRad(float)` / `radToDeg(float)`     | Conversion d'angles                                            |
| `WrapAngle2PI(float)`                     | Normalise dans ]-π, π]                                       |
| `cmpf(float, float, epsilon)`             | Comparaison flottants avec tolérance                          |
| `convertPositionBeaconToRepereTable(...)` | Projette une détection balise (polaire) en coordonnées table |

## Threads et boucles (◆)

Deux mécanismes d'exécution périodique :

- **THREAD + LOOP** : hérite de `utils::Thread` (pthread, SCHED_FIFO), boucle `while(1)` dans `execute()`
- **TIMER** : implémente `ITimerPosixListener`, callback `onTimer()` déclenché par SIGALRM via `ActionManagerTimer`

### Séquence de démarrage

```
Main thread
  │
  ├─→ LoggerFactory              THREAD prio 0    LOOP flush appenders en continu
  │
  ├─→ AsservEsialR               THREAD prio 2    LOOP 20Hz / 50ms (PID + odométrie)
  │   ou AsservDriver (ARM)      THREAD prio 2    LOOP ~10Hz / 100ms (lecture série carte ST)
  │   ou AsservDriver (SIMU)     THREAD prio 2    LOOP simulée
  │
  ├─→ ActionManagerTimer         THREAD prio 2    event-driven (sem_wait/sem_post)
  │     │
  │     ├── ServoObjectsTimer    TIMER  50ms              onTimer → interpolation position servos
  │     └── LedBarTimer          TIMER  variable µs       onTimer → animation LEDs
  │
  ├─→ SensorsThread              THREAD prio 2    LOOP variable ms (sleep) — lecture beacon I2C + détection obstacle
  │
  └─→ O_State_DecisionMakerIA   THREAD prio 3    exécution unique (stratégie match)
```

### Détail des threads (THREAD + LOOP)


| Fichier                                      | Classe                  | Prio | Loop                      | Période                     | Rôle                                        |
| ---------------------------------------------- | ------------------------- | ------ | --------------------------- | ------------------------------ | ---------------------------------------------- |
| `common/asserv-esial/AsservEsialR`           | AsservEsialR            | 2    | `while(1)` + chronoTimer  | **50ms** (20Hz) configurable | Odométrie, PID, commandes mouvement         |
| `driver-arm/AsservDriver`                    | AsservDriver            | 2    | `while(1)` + sleep        | **100ms** (~10Hz)            | Lecture série carte ST, parse position      |
| `driver-arm/AsservDriver_mbed_i2c`           | AsservDriver            | 2    | `while(1)` + chronoTimer  | variable                     | Variante I2C (ancien mbed)                   |
| `driver-simu/AsservDriver`                   | AsservDriver            | 2    | `while(1)` + timer        | variable                     | Simulation moteurs (math pure)               |
| `common/action/ActionManagerTimer`           | ActionManagerTimer      | 2    | `while(!stop)` + sem_wait | event-driven                 | Queue d'actions async, gère les TIMER       |
| `common/action/Sensors`                      | SensorsThread           | 2    | `while(!stop)` + sleep    | **variable ms**              | Lit beacon I2C, filtre niveaux 0-4, stoppe le robot |
| `common/log/LoggerFactory`                   | LoggerFactory           | 0    | `while(!stop)`            | non borné                   | Flush buffers log vers appenders             |
| `bot/opos6ul/states/O_State_DecisionMakerIA` | O_State_DecisionMakerIA | 3    | exécution unique         | **une fois**                 | Stratégie match, appels mouvement bloquants |

### Détail des timers (TIMER onTimer)

Tous gérés par `ActionManagerTimer`, enregistrés via `actions().addTimer()`.


| Fichier                                      | Classe            | Période         | onTimer() fait quoi                                 | Décide ?                               |
| ---------------------------------------------- | ------------------- | ------------------ | ----------------------------------------------------- | ----------------------------------------- |
| `common/action/ServoObjectsSystem`           | ServoObjectsTimer | **50ms**         | Calcule position interpolée, envoie commande servo | Oui (contrôle moteur servo)            |
| `common/action/LedBar`                       | LedBarTimer       | **variable µs** | Anime LEDs (alternate, K2000, blink)                | Non (affichage)                         |
| `bot/opos6ul/tests/O_ActionManagerTimerTest` | TestTimer         | **100-500ms**    | Log messages                                        | Non (test seulement)                    |

### Notes

- **AsservEsialR vs AsservDriver** : mutuellement exclusifs. AsservEsialR = asserv interne (PID sur OPOS6UL). AsservDriver = asserv externe (carte ST via série).
- **Priorités SCHED_FIFO** : prio 2 pour le temps réel (asserv, actions), prio 3 pour la stratégie (moins critique que le mouvement).
- **ActionManagerTimer** : ne boucle pas à fréquence fixe, il dort sur un sémaphore et se réveille quand une action est ajoutée (`sem_post`). Il gère aussi tous les TIMER (SIGALRM).
- **SensorsThread** (anciennement SensorsTimer, refactor avril 2026) : thread dédié à la lecture beacon/capteurs et à la décision d'arrêt sur détection. Migré du modèle TIMER vers THREAD pour permettre un sleep contrôlé entre lectures I2C — voir section refactoring plus bas.

## Refactoring prévu : AAsservDriver → AAsserv + AMotorDriver

### Problème actuel (ancien PMX)

L'interface `AAsservDriver` contient **47 méthodes** mais mélange deux responsabilités :

- Commandes haut niveau (goTo, doLine, doRotate) → utilisées par le robot
- Contrôle bas niveau moteurs (setPower, getEncoder) → utilisé seulement en simulation interne

Résultat : chaque implémentation a **~15 stubs vides** (méthodes non pertinentes pour elle).

```
Ancien : AAsservDriver (47 méthodes, interface gonflée)
├── AsservDriver ARM    → stubs sur moteurs/encodeurs (la carte ST gère)
├── AsservDriver SIMU   → stubs sur régulation (simule les moteurs directement)
└── AsservEsialR        → stubs sur moteurs/encodeurs (utilise ses propres codeurs)
```

### Analyse méthode par méthode


| Groupe                                        | Nb      | ARM (carte ST) | SIMU       | EsialR     | Constat        |
| ----------------------------------------------- | --------- | ---------------- | ------------ | ------------ | ---------------- |
| Moteurs directs (setPower, setPosition, stop) | 7       | STUB           | FULL       | STUB       | SIMU seulement |
| Encodeurs (getEncoder, getCounts, reset)      | 11      | STUB           | FULL       | STUB       | SIMU seulement |
| Odométrie (setPosition, getPosition)         | 2       | FULL           | FULL       | FULL       | **Commun**     |
| Mouvements (doLine, doRotate, goto...)        | 8       | FULL           | FULL       | FULL*      | **Commun**     |
| Vitesse (setLowSpeed, setMaxSpeed)            | 3       | FULL           | FULL       | PARTIEL    | **Commun**     |
| Contrôle (freeMotion, activateManager...)    | 4       | FULL           | FULL       | FULL       | **Commun**     |
| Interruption (interrupt, resetEmergency)      | 2       | FULL           | FULL       | FULL       | **Commun**     |
| Régulation (activateReguDist/Angle)          | 2       | FULL           | STUB       | FULL       | ARM+EsialR     |
| Courant moteur (getMotorCurrent)              | 2       | STUB           | STUB       | STUB       | **Code mort**  |
| Deprecated (path_GetLastCommandStatus)        | 1       | returns -1     | returns -1 | returns -1 | **Code mort**  |
| **Total**                                     | **~47** |                |            |            |                |

*EsialR : GotoChain utilise addGoToEnchainement (enchainement sans arret).

### Solution cible : deux interfaces

#### Répartition des 47 méthodes


| Destination                                 | Méthodes                                                                                                                                                                                                                                                                                                                                                                                     | Nb      |
| --------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------- |
| **→ AAsserv** (haut niveau)                | odo_SetPosition, odo_GetPosition, motion_DoLine, motion_DoRotate, motion_DoFace, motion_Goto, motion_GotoReverse, motion_GotoChain, motion_GotoReverseChain, motion_FreeMotion, motion_DisablePID, motion_AssistedHandling, motion_ActivateManager, motion_setLowSpeedForward, motion_setLowSpeedBackward, motion_setMaxSpeed, path_InterruptTrajectory, path_ResetEmergencyStop, endWhatTodo | **19**  |
| **→ AMotorDriver** (bas niveau)            | setMotorLeftPower, setMotorRightPower, setMotorLeftPosition, setMotorRightPosition, stopMotorLeft, stopMotorRight, stopMotors, getLeftExternalEncoder, getRightExternalEncoder, getLeftInternalEncoder, getRightInternalEncoder, getCountsExternal, getDeltaCountsExternal, resetEncoders                                                                                                     | **14**  |
| **→ À décider** (AAsserv ou spécifique) | motion_ActivateReguDist, motion_ActivateReguAngle, motion_DoArcRotate                                                                                                                                                                                                                                                                                                                         | **3**   |
| **→ Fusionnées** (doublons inutiles)      | getCountsInternal, resetInternalEncoders, resetExternalEncoders → fusionnés dans AMotorDriver                                                                                                                                                                                                                                                                                               | **3**   |
| **→ Supprimées** (code mort)              | getMotorLeftCurrent, getMotorRightCurrent, path_GetLastCommandStatus                                                                                                                                                                                                                                                                                                                          | **3**   |
|                                             | **Total ancien**                                                                                                                                                                                                                                                                                                                                                                              | **~47** |
|                                             | dont réellement utiles                                                                                                                                                                                                                                                                                                                                                                       | **~36** |

```
Robot / IA / States
       │
       ▼
    AAsserv  ←── interface haut niveau (19 méthodes)
    │
    ├── AsservST ─────────── proxy série vers carte ST     (ARM, externe)
    │                         ◆ THREAD boucle lecture série
    │
    ├── AsservEsialR ──┐     PID interne sur OPOS6UL       (SIMU ou ARM)
    │                  │     ◆ THREAD boucle 20Hz
    │                  ▼
    │             AMotorDriver ←── interface bas niveau (14 méthodes)
    │             ├── MotorDriverSIMU    simulation math
    │             └── MotorDriverARM     (optionnel, si PID interne sur ARM)
    │
    └── AsservNewVersion ──→ AMotorDriver   futur algo
```

#### AAsserv — interface haut niveau (ce que le robot utilise)

```cpp
class AAsserv {
    // Odométrie
    virtual void odo_SetPosition(float x, float y, float angle) = 0;
    virtual RobotPosition odo_GetPosition() = 0;

    // Mouvements
    virtual TRAJ_STATE motion_DoLine(float dist_mm) = 0;
    virtual TRAJ_STATE motion_DoRotate(float angle_rad) = 0;
    virtual TRAJ_STATE motion_DoFace(float x, float y) = 0;
    virtual TRAJ_STATE motion_Goto(float x, float y) = 0;
    virtual TRAJ_STATE motion_GotoReverse(float x, float y) = 0;
    virtual TRAJ_STATE motion_GotoChain(float x, float y) = 0;
    virtual TRAJ_STATE motion_GotoReverseChain(float x, float y) = 0;
    virtual TRAJ_STATE motion_DoArcRotate(float angle, float radius) = 0;  // à confirmer

    // Contrôle état
    virtual void motion_FreeMotion() = 0;
    virtual void motion_DisablePID() = 0;
    virtual void motion_AssistedHandling() = 0;
    virtual void motion_ActivateManager(bool enable) = 0;
    virtual void endWhatTodo() = 0;

    // Vitesse
    virtual void motion_setLowSpeedForward(bool enable, int percent) = 0;
    virtual void motion_setLowSpeedBackward(bool enable, int percent) = 0;
    virtual void motion_setMaxSpeed(bool enable, int speed_dist, int speed_angle) = 0;

    // Urgence
    virtual void path_InterruptTrajectory() = 0;
    virtual void path_ResetEmergencyStop() = 0;
};
```

#### AMotorDriver — interface bas niveau (seulement pour asserv interne)

```cpp
class AMotorDriver {
    // Commande moteurs
    virtual void setMotorLeftPower(int power, int time_ms) = 0;
    virtual void setMotorRightPower(int power, int time_ms) = 0;
    virtual void setMotorLeftPosition(int power, long ticks) = 0;
    virtual void setMotorRightPosition(int power, long ticks) = 0;
    virtual void stopMotorLeft() = 0;
    virtual void stopMotorRight() = 0;
    virtual void stopMotors() = 0;

    // Encodeurs
    virtual long getLeftEncoder() = 0;
    virtual long getRightEncoder() = 0;
    virtual void getCounts(long& countR, long& countL) = 0;
    virtual void getDeltaCounts(long& deltaR, long& deltaL) = 0;
    virtual void resetEncoders() = 0;
};
```

#### Méthodes supprimées (code mort, stub dans toutes les implémentations)

- `getMotorLeftCurrent()`, `getMotorRightCurrent()` → stub partout, jamais utilisé
- `path_GetLastCommandStatus()` → deprecated, returns -1 partout
- `getCountsInternal()`, `resetInternalEncoders()`, `resetExternalEncoders()` → doublons, fusionnés dans AMotorDriver

### Plan de migration

Ce refactoring se fera **progressivement**, pas en une fois :

1. **Phase 1** : migrer le code tel quel (AAsservDriver avec ses 47 méthodes)
2. **Phase 2** : extraire AAsserv (interface haut niveau) depuis AAsservDriver
3. **Phase 3** : extraire AMotorDriver (interface bas niveau)
4. **Phase 4** : supprimer les stubs et le code mort
5. **Phase 5** : intégrer asserv-newversion comme nouvelle implémentation de AAsserv

## Timers et scheduler (ActionTimerScheduler + ITimerScheduledListener)

Le système utilise un **scheduler unifié** `ActionTimerScheduler` : **1 seul thread persistant**
qui gère à la fois toutes les `IAction` et tous les `ITimerScheduledListener` (timers périodiques).

### Liste des timers

| Timer                 | Période       | Fichier                                      | onTimer() fait quoi                                    | Décide ?                    |
| ----------------------- | ---------------- | ---------------------------------------------- | -------------------------------------------------------- | ------------------------------ |
| **ServoObjectsTimer** | 50ms           | `common/action/ServoObjectsSystem.cpp/hpp`   | Interpole position servo, envoie commande PWM          | Oui (contrôle moteur servo) |
| **LedBarTimer**       | variable (µs) | `common/action/LedBar.cpp/hpp`               | Anime LEDs (alternate, K2000, blink)                   | Non (affichage)              |
| **TestTimer**         | 100-500ms      | `bot/opos6ul/tests/O_ActionManagerTimerTest` | Log messages (test seulement)                          | Non                          |

### Mécanisme du scheduler

```
ActionTimerScheduler (1 seul thread persistant)
  │
  │  boucle execute() :
  │  ├─ sem_wait() si pas d'action ni timer → 0% CPU au repos
  │  ├─ clock_nanosleep ABSTIME jusqu'au prochain tick → pas de drift
  │  ├─ tick les timers prêts (next_tick_us <= now)
  │  └─ execute 1 IAction par cycle (pour ne pas bloquer les timers)
  │
  ├── liste de IAction (one-shot ou récurrentes terminables)
  │     └─ execute() return true = à repousser en queue
  │     └─ execute() return false = terminée, retirée
  │
  └── liste de ITimerScheduledListener
        ├── ServoObjectsTimer ──→ onTimer() toutes les 50ms
        └── LedBarTimer ──→ onTimer() toutes les N µs
```

**Contrainte importante** : les `onTimer()` et `IAction::execute()` doivent être **courts** (<1ms
typiquement) car ils bloquent le scheduler pendant leur exécution. Les opérations longues
doivent être découpées en state machine (ex: attendre qu'un servo arrive en position sans
bloquer le scheduler).

Enregistrement :
- `actions().addAction(IAction*)` — pour une tâche asynchrone one-shot ou récurrente
- `actions().addTimer(ITimerScheduledListener*)` — pour un callback périodique

### Migration SIGEV_SIGNAL → SIGEV_THREAD → ActionTimerScheduler (avril 2026) ✅

#### Bug historique : deadlock par signal handler

L'ancienne implémentation utilisait `SIGEV_SIGNAL` + `SIGALRM` : le timer POSIX envoyait
un signal UNIX qui **interrompait le thread principal** pour exécuter `onTimer()` dans
un signal handler (`alarmFunction`).

**Problème critique** : le signal handler s'exécutait dans le contexte du thread interrompu.
Si ce thread tenait un mutex au moment de l'interruption (ex: `msync_` dans SensorsDriver,
ou le mutex SvgWriter), le callback `onTimer()` tentait de verrouiller le même mutex
depuis le même thread → **deadlock** (mutex POSIX non-récursif par défaut).

```
Thread principal:  getvPositionsAdv() → msync_.lock()  ← tient le verrou
                          ↑ SIGALRM interrompt ici
Signal handler:    onTimer() → sync() → msync_.lock()  → DEADLOCK
                   (même thread, même mutex non-récursif)
```

Ce bug se manifestait par des **blocages aléatoires** dont la fréquence augmentait
avec le nombre de mutex utilisés. Il a probablement affecté aussi le robot EV3
(PMX, brique Lego sous Debian Linux) qui utilisait le même mécanisme.

De plus, utiliser `pthread_mutex_lock()` dans un signal handler est un **comportement
indéfini** selon POSIX (fonction non async-signal-safe).

#### Solution : SIGEV_THREAD

`SIGEV_THREAD` demande à glibc de créer un **vrai thread séparé** pour chaque expiration
du timer au lieu d'envoyer un signal. Les mutex fonctionnent normalement entre threads.

| | SIGEV_SIGNAL (ancien) | SIGEV_THREAD (actuel) |
|---|---|---|
| Exécution | Signal handler dans le thread interrompu | Vrai thread séparé créé par glibc |
| Mutex | **Deadlock** si le thread interrompu tient un mutex | Fonctionnement normal inter-thread |
| Norme POSIX | Mutex dans signal handler = comportement indéfini | Conforme POSIX |
| Latence | ~0 (interruption directe) | ~100µs (pthread_create sur ARM Cortex-A7) |
| Pour timer ≥ 10ms | OK | OK (overhead < 1%) |

Fichier modifié : `common/timer/ITimerPosixListener.hpp`

#### Étape 3 : SIGEV_THREAD → thread persistant (ActionTimerScheduler)

**Nouveau problème découvert** : avec `SIGEV_THREAD`, glibc crée **un nouveau pthread
à chaque expiration** du timer. À haute fréquence (10ms → 100 threads/sec), combiné avec
`mlockall(MCL_CURRENT|MCL_FUTURE)` qui verrouille les pages de stack, la mémoire se
sature en quelques secondes → **OOM kill** ("Killed" sans explication au démarrage).

**Solution finale** : nouveau scheduler unifié `ActionTimerScheduler` — **1 seul thread
persistant** pour toutes les `IAction` et tous les `ITimerScheduledListener`.

| | SIGEV_SIGNAL | SIGEV_THREAD | ActionTimerScheduler (final) |
|---|---|---|---|
| Threads créés | 0 (signal handler) | 1 par expiration | **1 seul persistant** |
| Mutex | **Deadlock** (signal handler) | OK | OK |
| Haute fréquence (5-10ms) | OK (signal léger) | **OOM kill** | **OK** (pas d'allocation) |
| CPU au repos | N/A | 0% (glibc dort) | **0%** (sem_wait + clock_nanosleep) |
| Précision timing | Excellente | Bonne | **Excellente** (clock_nanosleep ABSTIME, pas de drift) |
| Latence ajout action | 0 | 0 | ≤ période du plus petit timer |
| Mixer actions + timers | Séparé (ActionManagerTimer) | Séparé | **Unifié** (1 scheduler) |

**Nouvelle interface `ITimerScheduledListener`** : callback pur, pas d'héritage `Thread`,
pas de signal handler. Membres `onTimer()`, `onTimerEnd()`, `name()`, `timeSpan_us()`,
`requestStop()`, `setPause()`. Utilisé par `LedBarTimer`,
`ServoObjectsTimer`, `TestTimer`. (Note : `Sensors` a été migré vers un thread dédié
`SensorsThread` et n'utilise plus l'interface timer.)

**Code legacy conservé pour rollback** : `ActionManagerTimer`, `ITimerPosixListener`,
`ITimerListener`. Pourront être supprimés une fois le nouveau scheduler validé en match.

**Fichiers** :
- `common/timer/ActionTimerScheduler.hpp/.cpp` (nouveau)
- `common/timer/ITimerScheduledListener.hpp` (nouveau)
- `test/common/ActionTimerSchedulerTest.hpp/.cpp` (15 tests dont 3 régressions critiques :
  mutex partagé, haute fréquence sans OOM, 0% CPU au repos)

#### Concurrence des callbacks

`mAlarm_` (mutex statique) sérialise les exécutions de `onTimer()` : si le timer
re-fire pendant qu'un callback est en cours, le nouveau thread attend la fin du précédent.

#### Références

- [timer_create(2)](https://man7.org/linux/man-pages/man2/timer_create.2.html)
- [sigevent(7)](https://man7.org/linux/man-pages/man7/sigevent.7.html)
- [Exemple TLPI ptmr_sigev_thread.c](https://man7.org/tlpi/code/online/dist/timers/ptmr_sigev_thread.c.html)

#### Protection des données partagées (SensorsDriver)

En parallèle de la migration SIGEV_THREAD, les accès concurrents dans SensorsDriver
ont été corrigés :

- `getvPositionsAdv()` : lecture de `vadv_` protégée par `msync_` (était commenté)
- `sync()` : lectures I2C sorties du mutex pour ne verrouiller `msync_` que brièvement
  lors de la mise à jour de `vadv_` et `regs_`
- `setPositionsAdvByBeacon()` : réservé au timer (seul écrivain de `opponents_last_positions`)
- `getPositionsAdv()` ajouté : lecture thread-safe pour les consommateurs (tests, IA, Navigator)

## Refactoring prévu : détection d'obstacles (SensorsThread)

> **Note** : la migration TIMER → THREAD est faite (avril 2026). La section ci-dessous
> décrit les responsabilités historiquement mélangées dans la boucle `Sensors`
> (lecture / filtrage / décision) qui restent à séparer en composants distincts.

### Problème actuel (ancien PMX)

La boucle `SensorsThread` (ex `SensorsTimer::onTimer()`) mélange **3 responsabilités** :

```
SensorsThread::execute()         ← boucle thread (sleep variable ms)
  │
  ├─ 1. LECTURE         sync("beacon_sync")
  │                       └─ I2C vers BeaconSensors (ToF)
  │
  ├─ 2. FILTRAGE        filtre_levelInFront/Back()
  │                       └─ seuils distance → level 0-4
  │                       └─ debounce (nb_ensurefront4, nb_sensor_front_a_zero)
  │                       └─ patch hardcodé +/-50mm ("//patch balise!!!!!")
  │
  └─ 3. DÉCISION        robot()->passerv()->warnFrontDetectionOnTraj(level, x, y)
     + EXÉCUTION           └─ Asserv::setEmergencyStop()
                               └─ path_InterruptTrajectory()  ← STOPPE LE ROBOT
```

### Conséquences

**Conflit de décision** entre IA et capteurs :

```
DecisionMakerIA (thread prio 3) : "va à (500, 300)"
     │  motion_Goto() — bloquant
     │
SensorsThread (thread async)    : "obstacle level 4 → STOP !"
     │  path_InterruptTrajectory()
     │
DecisionMakerIA                 : motion_Goto() retourne TRAJ_INTERRUPTED
                                  mais ne sait pas POURQUOI ni OÙ est l'obstacle
```

**État dispersé dans 4 classes** :

```
SensorsDriver.vadv_                    ← positions brutes adversaire (mutex)
Sensors.adv_is_detected_front_right_   ← flags détection
SensorsThread.nb_ensurefront4          ← compteurs debounce
Asserv.temp_ignoreFrontDetection_      ← flag pour contourner le système
```

**Pas de coordination** :

- Stop immédiat level 4, pas de ralentissement progressif
- Flags `temp_ignoreFrontDetection_` pour contourner → fragile
- L'IA ne peut pas décider de contourner plutôt que stopper

### Solution cible : séparer en couches + détection par trajectoire

#### Étape 1 — Séparer les responsabilités

```
Couche 1 — LECTURE (driver pur, aucune décision)
┌──────────────────────────────────┐
│  BeaconSensors → I2C ToF brut   │
│  SensorsDriver → normalise,     │
│                  calibre         │
│  Résultat : positions table     │
│    (x_adv, y_adv) en mm         │
└──────────────┬───────────────────┘
               │ données brutes
               ▼
Couche 2 — DÉTECTION (logique pure, AUCUN side-effect)
┌──────────────────────────────────┐
│  ObstacleDetector                │
│    entrée : pos adversaire       │
│           + pos robot            │
│           + trajectoire en cours │
│    sortie : ObstacleEvent {      │
│      onPath, x, y, dist, time } │
│    debounce intégré              │
│    testable unitairement         │
│    N'APPELLE RIEN                │
└──────────────┬───────────────────┘
               │ événement
               ▼
Couche 3 — DÉCISION (un seul décideur)
┌──────────────────────────────────┐
│  DecisionMakerIA                 │
│    reçoit ObstacleEvent          │
│    décide : ralentir / stopper / │
│      contourner / ignorer        │
│    LUI SEUL appelle l'asserv     │
└──────────────┬───────────────────┘
               │ ordres mouvement
               ▼
Couche 4 — EXÉCUTION
┌──────────────────────────────────┐
│  AAsserv                         │
│    exécute les ordres            │
│    path_InterruptTrajectory()    │
│    motion_setLowSpeed()          │
└──────────────────────────────────┘
```

#### Étape 2 — Remplacer "devant/derrière" par "sur le chemin"

L'ancien système utilise la position de l'adversaire **relative au robot** (devant/derrière/gauche/droite)
avec `filtre_levelInFront/Back()`. Problème : "devant" ne veut rien dire si l'adversaire n'est pas
sur la trajectoire que le robot va emprunter.

La beacon ToF fournit déjà les **coordonnées table (x, y)** de l'adversaire. On connaît aussi
la trajectoire en cours (point départ → point destination). La vraie question est :

```
L'adversaire est-il SUR LE CHEMIN du robot ?

        Adversaire (800,450)
              ●
             /
            /  ← distance au segment < seuil ?
           /
        ● ────────────────── ●
      robot                destination
    (200,300)              (1000,500)
```

```cpp
// Logique pure, testable, pas de flags
bool isOnPath(Point adv, Point from, Point to, float safetyRadius) {
    float dist = distancePointToSegment(adv, from, to);
    return dist < safetyRadius;  // ex: 200mm (rayon robot + marge)
}
```

#### Ce que ça résout à terme (piste, non figée)

L'approche `isOnPath` pourrait à terme remplacer `filtre_levelInFront/Back`, mais ce n'est pas
encore décidé. Le tableau ci-dessous illustre les cas que ça résoudrait :


| Situation                                        | Avant (devant/derrière + flags)               | Avec isOnPath                            |
| -------------------------------------------------- | ------------------------------------------------ | ------------------------------------------ |
| Rotation sur place, adversaire devant            | STOP (faux positif) →`temp_ignoreFront=true`  | Pas sur le chemin → OK                  |
| Goto, adversaire sur le côté                   | STOP si dans le seuil frontal                  | Pas sur le chemin → OK                  |
| Goto, adversaire pile sur la route               | STOP                                           | Sur le chemin → STOP (vrai positif)     |
| Pathfinding chaîné                             | Flags oubliés entre segments                  | Chaque segment vérifié indépendamment |
| Goto composite (carte ST : rotation+translation) | Ne sait pas quelle phase → mauvaise décision | Vérifie le segment final, pas la phase  |

### Problème identifié : la reprise après un STOP

Les zones de sécurité autour du robot (ralentissement + emergency stop) restent pertinentes
pour la **protection physique immédiate**. Le vrai problème est la **reprise** :

```
1. Robot avance vers (500, 300)
2. Adversaire entre dans zone STOP → emergency stop   ← OK, correct
3. L'IA décide de repartir ailleurs → motion_Goto(800, 100)
4. L'adversaire est toujours dans la zone "devant" du robot
5. → re-STOP immédiat !                                ← PROBLÈME
   La nouvelle trajectoire évite l'adversaire,
   mais le système de zones ne le sait pas
```

Le robot est bloqué : il ne peut pas repartir tant que l'adversaire est "devant",
même si la nouvelle destination passe ailleurs.

#### Piste : deux niveaux complémentaires

```
Niveau 1 — Zones autour du robot (existant, à conserver)
  │  Protection physique immédiate
  │  Ralentissement progressif + emergency stop
  │  Indépendant de la trajectoire
  │  → sécurité anti-collision
  │
Niveau 2 — Vérification trajectoire (nouveau, pour la reprise)
  │  Utilisé UNIQUEMENT pour autoriser une reprise après STOP
  │  "Est-ce que ma NOUVELLE trajectoire passe par l'adversaire ?"
  │
  │  isOnPath(adv, robot, new_destination, safetyRadius)
  │    → oui : rester stoppé ou recalculer un chemin
  │    → non : autoriser le départ, les zones niveau 1 protègent pendant le mouvement
```

Cette approche reste à affiner. Questions ouvertes :

- Quel rayon de sécurité pour isOnPath ?
- Faut-il aussi vérifier les segments suivants du pathfinding ?
- Comment gérer un adversaire qui bouge pendant la reprise ?

### Ce que ça change (résumé)


| Avant                                             | Après                                                        |
| --------------------------------------------------- | --------------------------------------------------------------- |
| SensorsThread lit ET filtre ET stoppe le robot    | SensorsThread lit, publie les coordonnées adversaire         |
| L'IA ne sait pas pourquoi le robot s'est arrêté | L'IA reçoit l'événement + coordonnées, décide elle-même |
| Reprise impossible si adversaire "devant"         | Reprise autorisée si nouvelle trajectoire est libre          |
| Debounce dans le timer (état mélangé)          | Debounce dans ObstacleDetector (logique pure, testable)       |
| Patch`+/-50mm` hardcodé dans le filtre           | Calibration dans config.txt                                   |
| Impossible à tester sans hardware                | ObstacleDetector = fonction pure, testable en SIMU            |

### Plan de migration

1. **Phase 1** : migrer tel quel (SensorsThread avec zones devant/derrière)
2. **Phase 2** : extraire ObstacleDetector (logique pure) depuis Sensors::filtre_levelInFront/Back
3. **Phase 3** : remplacer les appels directs à l'asserv par des ObstacleEvent
4. **Phase 4** : intégrer la réception d'ObstacleEvent dans DecisionMakerIA
5. **Phase 5** : ajouter la vérification trajectoire (isOnPath) pour la logique de reprise

## Ordre de migration — classe par classe avec tests unitaires

Chaque classe est migrée avec son test. Pour les drivers, on migre toujours le couple
SIMU + ARM + test unitaire via l'interface abstraite.

### Étape 1 — Briques de base (aucune dépendance) ✅


| Classe                 | Dossier cible    | Test            | Statut |
| ------------------------ | ------------------ | ----------------- | -------- |
| Thread / Mutex         | `common/thread/` | ThreadTest      | ✅     |
| Chronometer            | `common/utils/`  | ChronometerTest | ✅     |
| Logger / LoggerFactory | `common/log/`    | LoggerTest      | ✅     |

### Étape 2 — Framework de test ✅


| Classe           | Dossier cible | Statut                   |
| ------------------ | --------------- | -------------------------- |
| UnitTest         | `test/suite/` | ✅                       |
| UnitTestSuite    | `test/suite/` | ✅                       |
| UnitTestAppender | `test/suite/` | ✅ (+ colorisation ANSI) |

### Étape 3 — Interfaces abstraites drivers ✅


| Classe                 | Dossier cible       | Statut |
| ------------------------ | --------------------- | -------- |
| AAsservDriver          | `common/interface/` | ✅     |
| ASensorsDriver         | `common/interface/` | ✅     |
| ALedDriver             | `common/interface/` | ✅     |
| AButtonDriver          | `common/interface/` | ✅     |
| ASwitchDriver          | `common/interface/` | ✅     |
| AServoDriver           | `common/interface/` | ✅     |
| ALcdShieldDriver       | `common/interface/` | ✅     |
| ASoundDriver           | `common/interface/` | ✅     |
| AColorDriver           | `common/interface/` | ✅     |
| AServoUsingMotorDriver | `common/interface/` | ✅     |
| AActionDriver          | `common/interface/` | ✅     |
| ARobotPositionShared   | `common/interface/` | ✅     |

### Étape 4 — Drivers (SIMU + ARM + test) ✅


| Driver                | SIMU | ARM | Test                                              | Statut |
| ----------------------- | ------ | ----- | --------------------------------------------------- | -------- |
| LedDriver             | ✅   | ✅  | LedDriverTest                                     | ✅     |
| ButtonDriver          | ✅   | ✅  | ButtonDriverManualTest                            | ✅     |
| SwitchDriver          | ✅   | ✅  | SwitchDriverTest                                  | ✅     |
| SoundDriver           | ✅   | ✅  | —                                                | ✅     |
| LcdShieldDriver       | ✅   | ✅  | LcdShieldDriverManualTest                         | ✅     |
| ServoDriver           | ✅   | ✅  | ServoDriverManualTest                             | ✅     |
| ServoUsingMotorDriver | ✅   | ✅  | —                                                | ✅     |
| SensorsDriver         | ✅   | ✅  | SensorsDriverTest (9 UT) + SensorDriverManualTest | ✅     |
| RobotPositionShared   | ✅   | ✅  | —                                                | ✅     |
| ColorDriver           | ✅   | ✅  | ColorDriverManualTest                             | ✅     |
| AsservDriver          | ✅   | ✅  | AsservDriverManualTest                            | ✅     |
| ActionDriver          | ✅   | ✅  | —                                                | ✅     |

### Étape 5 — Actions et timers ✅


| Classe                                       | Dossier cible    | Test                                   | Statut                                     |
| ---------------------------------------------- | ------------------ | ---------------------------------------- | -------------------------------------------- |
| IAction, ITimerListener, ITimerPosixListener | `common/timer/`  | —                                     | ✅                                         |
| ActionManagerTimer                           | `common/timer/`  | ActionManagerTimerTest (13 UT) + bench | ✅                                         |
| AActionsElement                              | `common/action/` | —                                     | ✅                                         |
| Actions                                      | `common/action/` | —                                     | ✅                                         |
| LedBar                                       | `common/action/` | O_LedBarTest                           | ✅                                         |
| ButtonBar                                    | `common/action/` | O_ButtonBarTest                        | ✅                                         |
| Sensors                                      | `common/action/` | O_SensorsTest (fonctionnel)            | ✅ (logique pure extraite → ObstacleZone) |
| ServoObjectsSystem                           | `common/action/` | O_ServoObjectsTest                     | ✅ (test non migré)                       |
| ServoUsingMotor                              | `common/action/` | —                                     | ✅                                         |
| Tirette                                      | `common/action/` | O_TiretteTest                          | ✅                                         |
| LcdShield                                    | `common/action/` | O_LcdBoardTest                         | ✅                                         |
| SoundBar                                     | `common/action/` | —                                     | ✅                                         |

### Étape 6 — Asserv, Robot et mouvement


| Classe                                     | Dossier cible          | Test                      | Statut                        |
| -------------------------------------------- | ------------------------ | --------------------------- | ------------------------------- |
| TableGeometry                              | `common/geometry/`     | TableGeometryTest (12 UT) | ✅ (nouveau)                  |
| ObstacleZone                               | `common/geometry/`     | ObstacleZoneTest (12 UT)  | ✅ (extrait de Sensors)       |
| Asserv                                     | `common/asserv/`       | —                        | ✅                            |
| AsservEsialR (+ sous-modules)              | `common/asserv.esial/` | O_AsservEsialTest         | ✅ (test non migré)          |
| Automate / AAutomateState                  | `common/state/`        | —                        | ✅                            |
| Arguments / ConsoleManager                 | `common/utils/`        | —                        | ✅                            |
| Robot                                      | `common/`              | —                        | ✅                            |
| MotorControl / EncoderControl / MovingBase | —                     | —                        | ⬜ Non utilisés, non migrés |
| IA / IAbyPath / IAbyZone                   | `common/ia/`           | O_IAbyPathTest            | ⬜ À migrer                  |

### Étape 7 — Bot OPOS6UL


| Classe                                 | Dossier cible  | Test                   | Statut               |
| ---------------------------------------- | ---------------- | ------------------------ | ---------------------- |
| OPOS6UL_RobotExtended                  | `bot-opos6ul/` | —                     | ✅                   |
| OPOS6UL_ActionsExtended                | `bot-opos6ul/` | —                     | ✅                   |
| OPOS6UL_AsservExtended                 | `bot-opos6ul/` | O_AsservLineRotateTest | ✅ (test non migré) |
| OPOS6UL_IAExtended                     | `bot-opos6ul/` | —                     | ✅                   |
| OPOS6UL_SvgWriterExtended              | `bot-opos6ul/` | —                     | ✅                   |
| O_State_Init / DecisionMaker / WaitEnd | `bot-opos6ul/` | —                     | ✅                   |
| O_LedBarTest                           | `bot-opos6ul/` | —                     | ✅                   |
| O_TiretteTest                          | `bot-opos6ul/` | —                     | ✅                   |
| O_ButtonBarTest                        | `bot-opos6ul/` | —                     | ✅                   |
| O_LcdBoardTest                         | `bot-opos6ul/` | —                     | ✅                   |
| O_ActionManagerTimerTest               | `bot-opos6ul/` | —                     | ✅                   |
| O_IAByPathTest                         | `bot-opos6ul/` | —                     | ✅                   |
| O_ServoStepTest                        | `bot-opos6ul/` | —                     | ✅                   |
| O_ServoObjectsTest                     | `bot-opos6ul/` | —                     | ✅                   |
| O_SensorsTest                          | `bot-opos6ul/` | —                     | ✅                   |
| Tests fonctionnels asserv (6)          | `bot-opos6ul/` | —                     | ⬜ À migrer         |

## Organisation des tests

Règle : **chaque test unitaire doit avoir au moins un `assert()` qui peut échouer**.
Sinon c'est un benchmark ou un test manuel, pas un test unitaire.

### Types de tests

```
test/
├── suite/                    # Framework UnitTest (maison, migré de PMX)
│   ├── UnitTest.cpp/hpp         assertions : assert(), fail(), expectedError()
│   ├── UnitTestSuite.cpp/hpp    orchestration : addTest(), run()
│   └── UnitTestAppender.cpp/hpp capture des erreurs via Logger
│
├── common/                   # Tests unitaires code commun
│   ├── MutexTest                lock/unlock/tryLock               (NOUVEAU)
│   ├── ThreadTest               start/state/waitForEnd + priorité
│   ├── ChronometerTest          elapsed > 0, précision ±1ms      (+ assertions)
│   ├── TimerPosixTest           onTimer appelé N fois en T sec    (+ assertions)
│   └── Main.cpp              → target : common-test
│
├── driver/                   # Tests driver via interface abstraite
│   ├── AsservDriverTest         position après mouvement          (+ assertions)
│   ├── SensorDriverTest         valeurs retournées                (+ assertions)
│   └── Main.cpp              → target : driver-test
│
├── bench/                    # Benchmarks (métriques, pas d'assertion)
│   ├── ChronometerBench         résolution timer, min/max/avg
│   ├── ReadWriteBench           comparaison I/O C/POSIX/C++
│   └── Main.cpp              → target : bench
│
└── manual/                   # Tests manuels hardware (sur la carte)
    ├── LedTest                  "je vois les LEDs clignoter"
    ├── ServoTest                "le servo bouge"
    └── Main.cpp              → target : manual-test
```

### Targets CMake


| Target        | Type                 | CI ?                        | Quand l'utiliser         |
| --------------- | ---------------------- | ----------------------------- | -------------------------- |
| `common-test` | Unit test            | Oui, doit passer            | À chaque build          |
| `driver-test` | Unit test            | Oui (SIMU), sur carte (ARM) | À chaque build          |
| `bench`       | Benchmark            | Non                         | Quand on optimise        |
| `manual-test` | Test visuel/physique | Non                         | Sur la carte, en atelier |

### Analyse qualité des tests PMX existants


| Test ancien            | Assertions réelles ?            | Migration vers                                         |
| ------------------------ | ---------------------------------- | -------------------------------------------------------- |
| ActionManagerTimerTest | Oui — vérifie compteurs        | `common/` (unit test)                                  |
| LoggerTest             | Oui — vérifie`expectedError()` | `common/` (unit test)                                  |
| ThreadTest             | Partiel — vérifie calculs      | `common/` (+ renforcer assertions)                     |
| ChronometerTest        | Non — log des timings           | `bench/` + nouveau test dans `common/` avec assertions |
| TimerFactoryTest       | Non — observe les logs          | `bench/` + nouveau test dans `common/` avec assertions |
| ReadWriteTest          | Non — benchmark I/O             | `bench/`                                               |
| LedDriverTest (ARM)    | Non —`assert(true)`             | `manual/`                                              |
| ServoDriverTest (ARM)  | Non —`assert(true)`             | `manual/`                                              |
| AsservDriverTest       | Non —`assert(true)`             | `driver/` (+ assertions réelles)                      |
| SensorDriverTest       | Non —`assert(true)`             | `driver/` (+ assertions réelles)                      |

## Déploiement ARM (OPOS6UL)

### Principe

Le binaire est compilé en cross-compilation (preset `arm-release`), strippé pour réduire
sa taille, puis envoyé sur la carte OPOS6UL via `scp`. L'authentification utilise `sshpass`.

### Prérequis

```bash
sudo apt install sshpass
```

### Adresses IP de la carte


| Interface            | IP              | Usage                        |
| ---------------------- | ----------------- | ------------------------------ |
| eth0 (câble direct) | `192.168.2.105` | En atelier, connexion fiable |
| WiFi 5GHz AP         | `192.168.3.103` | Sur le terrain, sans câble  |
| WiFi PMX             | `192.168.2.107` | Réseau local PMX            |
| WiFi maison          | `192.168.0.205` | Dev à domicile              |

### Utilisation

#### Via VSCode (recommandé)

Raccourci : **Ctrl+Shift+T** → menu des tasks :


| Task                | Description                               |
| --------------------- | ------------------------------------------- |
| `ARM: Deploy`       | Build ARM + strip + scp vers`/root/pmx/`  |
| `ARM: Deploy + Run` | Idem + exécute sur la carte via SSH      |
| `ARM: Connect SSH`  | Terminal SSH interactif dans`/root/pmx/`  |
| `ARM: Recup SVG`    | Rapatrie les fichiers SVG depuis la carte |

Chaque task demande l'IP et le binaire via des menus déroulants.

#### Via ligne de commande

```bash
cd robot/

# Deploy seulement
bash sh/deploy.sh 192.168.2.105 driver-test

# Deploy + exécute
bash sh/deploy.sh 192.168.2.105 driver-test run

# Autres cibles : common-test, manual-test, bench, opos6ul
bash sh/deploy.sh 192.168.3.103 opos6ul run
```

### Script `sh/deploy.sh`

Étapes exécutées :

1. `cmake --preset arm-release` + `cmake --build` sur la target demandée
2. Strip du binaire (toolchain ARM)
3. `scp` vers `root@IP:/root/pmx/`
4. (optionnel) `ssh` pour exécuter le binaire

## Documentation Doxygen

Le code utilise des commentaires Doxygen (`\file`, `\brief`, `\param`, `\return`) pour documenter
les classes et méthodes. Un `Doxyfile` est configuré dans `robot/`.

### Installation

```bash
sudo apt install doxygen graphviz
```

### Génération

```bash
cd robot/
doxygen
```

La doc HTML est générée dans `robot/docs/html/index.html`.

### Conventions de documentation

Chaque fichier `.hpp` doit avoir :

```cpp
/*!
 * \file
 * \brief Description courte du fichier.
 */
```

Chaque classe doit avoir un `\brief` :

```cpp
/*!
 * \brief Description de la classe et son rôle.
 */
class MaClasse {
```

Chaque méthode publique doit être documentée :

```cpp
/*!
 * \brief Ce que fait la méthode.
 * \param nom Description du paramètre.
 * \return Ce que retourne la méthode.
 */
```

### Configuration


| Paramètre        | Valeur     | Raison                                        |
| ------------------- | ------------ | ----------------------------------------------- |
| `EXTRACT_ALL`     | YES        | Documente aussi les méthodes non commentées |
| `EXTRACT_PRIVATE` | YES        | Utile pour comprendre l'architecture interne  |
| `HAVE_DOT`        | YES        | Diagrammes de classes et d'appels (graphviz)  |
| `CLASS_GRAPH`     | YES        | Hiérarchie d'héritage                       |
| `CALL_GRAPH`      | YES        | Graphe d'appels de fonctions                  |
| `EXCLUDE`         | `json.hpp` | nlohmann/json (24000 lignes, pas utile)       |
| `GENERATE_LATEX`  | NO         | HTML uniquement                               |

## Télémétrie réseau (UDP)

Le robot envoie ses données de télémétrie en JSON via UDP (port 9870) vers un récepteur externe.

### Architecture

```
OPOS6UL (robot)                              RPI (récepteur)
┌──────────────────┐       UDP 9870        ┌──────────────────┐
│ TelemetryAppender│ ──── WiFi 5GHz ────→  │ Réception JSON   │
│ (flush 300ms)    │    192.168.3.101      │                  │
└──────────────────┘                       └──────────────────┘
```

### Format des paquets JSON

Chaque paquet UDP contient un objet JSON avec l'ID du robot, un timestamp (`t`), le temps écoulé (`dt`) et les données du logger.

**Enveloppe commune** (toutes les trames) :

| Clé | Signification | Unité |
|-----|---------------|-------|
| `t` | timestamp horloge système | secondes (3 décimales) |
| `dt` | delta time depuis démarrage | millisecondes |

**Position du robot** (logger `Pos`, ~10-20 Hz, depuis AsservCborDriver) :

```json
{"OPOS6UL":{"t":1718193600.123,"dt":4567.890,"Pos":{"x":150.0,"y":300.0,"a":90.0,"s":1,"q":2}}}
```

| Clé | Signification | Unité |
|-----|---------------|-------|
| `x` | position X robot | mm |
| `y` | position Y robot | mm |
| `a` | angle robot | degrés |
| `s` | asserv status | 0=idle, 1=running, 2=emergency, 3=blocked |
| `q` | queue size | nombre de commandes en file |

**Positions adversaires** (logger `Adv`, envoyé seulement si détection, depuis SensorsDriver) :

```json
{"OPOS6UL":{"t":1718193600.123,"dt":4567.890,"Adv":{"n":2,"x1":150,"y1":200,"a1":45.0,"d1":500,"x2":300,"y2":400,"a2":120.0,"d2":700}}}
```

| Clé | Signification | Unité |
|-----|---------------|-------|
| `n` | nombre de robots adverses détectés | 0..4 |
| `x1`..`x4` | position X adversaire (repère beacon) | mm |
| `y1`..`y4` | position Y adversaire (repère beacon) | mm |
| `a1`..`a4` | angle adversaire | degrés |
| `d1`..`d4` | distance centre-à-centre | mm |

**LEDs** (logger `LedBar`) :

```json
{"OPOS6UL":{"t":1718193600.123,"dt":4567.890,"LedBar":{"pos":0,"color":6}}}
```

### Configuration réseau


| Destination      | IP              | Port |
| ------------------ | ----------------- | ------ |
| RPI (récepteur) | `192.168.3.101` | 9870 |

La même IP est utilisée en SIMU et en ARM. La configuration est dans `src/bot-opos6ul/LoggerInitialize.cpp`.

Reconfigurable au lancement : `./bot-opos6ul /i 192.168.3.50 /p 10000`

### Réception des trames UDP

Depuis le récepteur (RPI ou PC) :

```bash
# Affichage temps réel
nc -lu 9870

# Ou avec socat
socat -u UDP-LISTEN:9870,reuseaddr STDOUT

# Sauvegarde dans un fichier
socat -u UDP-LISTEN:9870,reuseaddr OPEN:./telemetry.json,creat,append &
```

### Loggers branchés sur la télémétrie

Pour envoyer de la télémétrie depuis un logger, le configurer sur l'appender `"net"` dans `LoggerInitialize.cpp` :

```cpp
add(logs::Level::INFO, "MonLogger", "net");
```

Puis dans le code, utiliser `logger().telemetry(json)` pour envoyer des données JSON.

## Bug historique EV3 — deadlock SIGALRM (rétrospective)

### Contexte

Le robot EV3 (LEGO Mindstorms, projet PMX) subissait des **arrêts intempestifs aléatoires**
pendant les matchs : le programme se figeait sans message d'erreur, de façon non reproductible.
Le bug n'a jamais été identifié à l'époque.

### Cause identifiée (avril 2026)

En corrigeant le même bug sur PMX-CORTEX/OPOS6UL, le mécanisme a été compris :
le fichier `PMX/src/Common/Action/ITimerPosixListener.hpp` contient **exactement le même code**
que PMX-CORTEX avant correction — `SIGEV_SIGNAL` + `SIGALRM` avec `pthread_mutex_lock()`
dans le signal handler `alarmFunction()`.

```
Thread principal:  mutex.lock()          ← tient le verrou
                       ↑ SIGALRM interrompt ici
Signal handler:    mAlarm_.lock()        → DEADLOCK (même thread, mutex non-récursif)
```

`pthread_mutex_lock()` n'est **pas async-signal-safe** selon POSIX
([signal-safety(7)](https://man7.org/linux/man-pages/man7/signal-safety.7.html)).
L'appeler dans un signal handler est un comportement indéfini.

### Pourquoi c'était pire sur l'EV3

| | EV3 | OPOS6UL |
|---|---|---|
| Processeur | ARM926EJ-S (ARMv5TE) | i.MX6ULL Cortex-A7 |
| Cœurs | **1 seul** | 1 |
| RAM | 64 MB | 512 MB |
| Toolchain | Sourcery CodeBench arm-2013.11-33 | Armadeus 7.0 (Buildroot) |
| GCC | 4.8.1 | 10.3 |
| glibc | 2.18 | 2.33 |
| Kernel | 3.x (ev3dev, Debian) | 5.10 |

Le processeur **mono-cœur** de l'EV3 rendait le deadlock **plus probable** : le signal
SIGALRM est toujours délivré au thread qui tient le mutex (pas d'autre cœur disponible).
Sur un multi-cœur, le signal peut être délivré à un autre thread qui ne tient pas le mutex.

### SIGEV_THREAD était disponible

`SIGEV_THREAD` est supporté par glibc depuis la version 2.3.2 (2003). La glibc 2.18
de la toolchain arm-2013.11-33 le supportait pleinement. Le fix aurait pu être appliqué
à l'époque.

### Symptômes connus sur ev3dev cohérents avec ce bug

- [ev3dev #243](https://github.com/ev3dev/ev3dev/issues/243) — freezes aléatoires avec périphériques I2C
- [ev3dev #324](https://github.com/ev3dev/ev3dev/issues/324) — instabilité de timing des boucles
- [LTP #925](https://github.com/linux-test-project/ltp/issues/925) — tests POSIX timer échouent sur ARM926EJ-S

### C'est un bug classique en embarqué Linux

Ce pattern (signal handler + mutex = deadlock) est une erreur connue dans les projets
qui portent du code bare-metal/RTOS vers Linux (où les interruptions timer fonctionnent
différemment) :

- [IBM : "Mutex Bad In Signal Context"](https://www.ibm.com/support/pages/mutex-bad-signal-context)
- ["Why you should avoid using SIGALRM for timer"](https://nativeguru.wordpress.com/2015/02/19/why-you-should-avoid-using-sigalrm-for-timer/) — scénario identique en embarqué
- [iceoryx #179](https://github.com/eclipse/iceoryx/issues/179) — deadlock timer POSIX dans un middleware robotique
- [grpc #24884](https://github.com/grpc/grpc/issues/24884) — deadlock mutex + signal handler dans gRPC
- [GCC Bug #21240](https://gcc.gnu.org/bugzilla/show_bug.cgi?id=21240) — "Deadlock (pthread) in signal handler"

### Correction appliquée (PMX-CORTEX uniquement)

Migration `SIGEV_SIGNAL` → `SIGEV_THREAD` dans `ITimerPosixListener.hpp`.
Voir [section migration détaillée](#migration-sigev_signal--sigev_thread-avril-2026-).

Le code PMX original (`/home/pmx/git/PMX/src/Common/Action/ITimerPosixListener.hpp`)
n'a **pas** été corrigé (projet archivé, plus maintenu).

### Caveat SIGEV_THREAD : accumulation de threads

Avec `SIGEV_THREAD`, glibc crée un **nouveau pthread pour chaque expiration** du timer.
Si le callback (`onTimer()`) dure plus longtemps que la période du timer, les threads
s'accumulent en attente sur `mAlarm_`, chacun allouant ~8MB de stack → OOM kill
sur système embarqué à RAM limitée.

**Solution** : `tryLock()` au lieu de `lock()` sur `mAlarm_`. Si le callback précédent
est encore en cours, le nouveau thread sort immédiatement sans s'accumuler.
Comportement identique à l'ancien SIGALRM (le signal était naturellement bloqué
pendant l'exécution du handler).

## TODO

### Migration PMX → PMX-CORTEX

- ✅ Module IA : migrer IAbyPath, IAbyZone, IACommon, IA (+ débloque O_IAByPathTest et O_State_DecisionMakerIA)
- ⬜ Tests fonctionnels asserv : O_AsservEsialTest, O_AsservLineRotateTest, O_AsservXYRotateTest, O_AsservTest, O_Asserv_SquareTest, O_Asserv_CalageTest
- ✅ Tests fonctionnels autres : O_ServoStepTest, O_ServoObjectsTest, O_SensorsTest
- ✅ O_IAByPathTest : activé, module IA migré
- ⬜ SlowMotionServo : tester la lib de motion progressive des servos (portage Arduino → Linux)

### Communication PAMIs (micro:bit)

- ⬜ Evaluer la communication avec les PAMIs micro:bit :
  - Option 1 : BLE via Broadcom CYW43430 integre (BT 4.1) — risque en milieu bruite (fiabilite < 5m en competition)
  - Option 2 (recommandee) : radio micro:bit native (Nordic ESB) avec un micro:bit passerelle sur USB/serie de l'OPOS6UL — plus robuste, reconnexion rapide
- ⬜ Definir le besoin : signal "top depart" uniquement ou communication bidirectionnelle pendant le match
- ⬜ Si bidirectionnel : thread dediee dans le brain (lecture/ecriture serie ou stack BLE), file de messages, timeout, fallback autonome PAMI
- ⬜ Tester la portee et fiabilite en conditions reelles (bruit 2.4 GHz)

### Refactoring architecture

- ⬜ Renommer `interface/` → `driver_interface/` et `timer/` → `action_interface/` (+ déplacer IAction.hpp dans action_interface/)
- ⬜ Renommer `Actions.hpp/cpp` → `TaskManager.hpp/cpp` (classe Actions → TaskManager)
- ⬜ Découpler Sensors de Asserv : Sensors ne devrait pas dépendre directement de Asserv.hpp, passer par une interface de position
- ⬜ Revoir l'architecture detection/capteurs : séparer la logique de filtrage des capteurs (seuils, niveaux) de la stratégie d'évitement
- ⬜ Supprimer la dépendance circulaire Robot ↔ Asserv (Asserv.cpp inclut Robot.hpp et Robot.hpp forward-declare Asserv)
- ⬜ Remplacer les `exit(-1)` dans les drivers par des exceptions ou des codes retour (pas de exit() dans du code embarqué)
- ⬜ Uniformiser les botId (OPOS6UL_Robot vs PMX) — un seul identifiant par robot
- ⬜ Changer le SVG pour celui de 2026

### Précision synchronisation beacon (position adv)

Erreur résiduelle observée : ~80mm sur la position adversaire à 1.5m, robot tournant à 90°/s.
Causes cumulées d'erreur sur t_mesure :

- ✅ **Latence I2C `getData()`** (~5-10ms) — résolu : `t_sync_ms` capturé par le driver dans `SensorsDriver::sync()` juste après `readFlag()`, avant le long `getData()`. Getter `getLastSyncMs()` ajouté à l'interface `ASensorsDriver`.
- ✅ **Buffer history non uniforme** — résolu : `AsservCborDriver` (thread CBOR) appelle `setRobotPosition()` à chaque trame reçue de la Nucleo, alimente `pushHistory()` en continu. `Asserv::pos_getPosition()` devient une lecture pure sans effet de bord.
- ✅ **Formule `t_mesure_ms` inversée** — résolu : `t_mesure = t_sync - TEENSY_CYCLE_MS + beacon_delay_us` (au lieu de `t_sync - beacon_delay_us`).
- ⬜ **Cycle Teensy variable** (40-60ms observés) : actuellement constante hardcodée `TEENSY_CYCLE_MS=60` dans `Sensors.cpp`. Solution : ajouter un registre I2C côté Teensy contenant la durée réelle du cycle, lecture par OPOS6UL au sync.
- ⬜ **Jitter polling SensorsThread** : période 20ms → la lecture du flag I2C arrive 0-20ms après la fin du cycle Teensy (aléatoire). Solution simple : réduire la période à 5ms.
- ⬜ **Solution propre (long terme)** : GPIO interrupt Teensy → OPOS6UL à chaque fin de cycle, capture timestamp exact côté OPOS6UL → jitter quasi nul. Nécessite cablage GPIO + handler interrupt.

À 1.5m de l'adversaire, 1° d'erreur de theta_robot = 26mm d'erreur de position. À 90°/s, 11ms d'erreur timing = 1° → 26mm. Réduire les 15-30ms à <5ms ramènerait l'erreur de 80mm à <15mm.

## Démarrage asserv : ordre `setPositionAndColor` → `startMotionTimerAndOdo`

### Règle impérative

`setPositionAndColor()` n'est **pas juste** une init de coordonnées, c'est le **point de
départ logique du match**. Il doit être appelé **AVANT** `startMotionTimerAndOdo()` car :

- Il applique la **couleur de match** (symétrie miroir des coordonnées pour blue/yellow)
- Il fait un **reset complet côté Nucleo** (position + odométrie)
- Avant ce setPos, la Nucleo peut encore avoir des positions résiduelles de la session précédente
- La couleur doit être définie avant que l'asserv ne commence à traiter des trajectoires

### Sémantique

```cpp
// 1. Couleur de match (normalement definie par parseConsoleArgs /y ou menu)
//    Defaut CLI = BLEU (color0 primaire). /y = JAUNE (miroir applique).

// 2. setPositionAndColor = RESET de match (Nucleo + couleur + origine)
//    Convention PMX : coords ecrites en repere BLEU ; isMatchColor() = true
//    si JAUNE -> applique le miroir x -> x_ground_table_ - x.
robot.asserv().setPositionAndColor(300, 600, 0, robot.isMatchColor());

// 3. Démarrage du thread CBOR (inclut un sleep 50ms pour laisser la Nucleo appliquer)
robot.asserv().startMotionTimerAndOdo(false);

// 4. Reste du programme (capteurs, IA, ...)
```

### Filet de sécurité : `positionInitialized_`

Dans `AsservCborDriver`, un flag `std::atomic<bool> positionInitialized_` filtre les
positions reçues tant que `odo_SetPosition()` n'a pas été appelé au moins une fois.
La boucle de réception CBOR fait :

```cpp
if (!positionInitialized_) {
    continue;  // skip history + skip SVG
}
```

Cela protège le SVG et l'historique de positions contre les données résiduelles de la
Nucleo, même si un test oublie de respecter l'ordre.

### Source de vérité unique pour `pos_getPosition()`

`Asserv::pos_getPosition()` lit maintenant directement depuis `sharedPosition` au lieu
de `asservdriver_->p_`. Motivation :

- `setPositionReal()` pousse immédiatement dans `sharedPosition` (pas d'attente)
- Le thread CBOR pousse aussi dans `sharedPosition` à chaque trame Nucleo reçue
- `sharedPosition` est **la** source de vérité, alimentée par les deux writers

Avant ce fix, juste après `setPositionAndColor(200, 200, 0)`, `pos_getPosition()` retournait
encore `(0, 0, 0)` pendant ~50ms (le temps que la Nucleo renvoie sa première trame et que
le thread CBOR mette à jour son `p_` local).

### Tests migrés

Tous les tests qui utilisent `startMotionTimerAndOdo` ont été mis à jour pour respecter
le nouvel ordre : `O_SensorsTest`, `O_AsservLineRotateTest`, `O_AsservTest`,
`O_Asserv_SquareTest`, `O_AsservXYRotateTest`,
`O_IAbyPathTest`, `O_AsservWaypointTest`, `O_NavigatorBackTest`, `O_NavigatorMovementTest`,
`O_State_Init`, `O_Asserv_CalageTest` (5 cas), `O_AsservCalibrationTest` (déjà correct).

`O_LedBarTest` nettoyé : ne démarre plus l'asserv qu'il n'utilisait pas.

## Clôture propre : thread CBOR + SVG valide

### Problème initial : `<circle>` après `</svg>`

Le thread CBOR (`AsservCborDriver`) continuait à écrire des positions dans `svgAPF.svg`
**après** la balise `</svg>` de fermeture, produisant un **SVG invalide**. Scénario type :

```
1. Test appelle robot.svgPrintEndOfFile()  → écrit </g></svg>
2. Pendant ce temps, le thread CBOR reçoit encore des positions de la Nucleo
3. Le thread logue <circle cx="..." cy="..." /> dans svgAPF.svg → APRÈS </svg>
4. Le SVG résultant est invalide (éléments hors racine)
```

### Solution : arrêter les producteurs avant de fermer le SVG

1. **`AsservCborDriver::stopReceiveThread()`** : flag `stopRequested_` + `waitForEnd()` pour
   arrêter proprement le thread de réception CBOR.
2. **`AsservCborDriver::endWhatTodo()`** : appelle `emergencyStop()` puis `stopReceiveThread()`.
3. **`OPOS6UL_RobotExtended::stopExtraActions()`** : arrête `asserv().endWhatTodo()` puis
   `actions().stopExtra()`. C'est le point d'entrée pour tout arrêter avant la clôture SVG.
4. **`~Robot()`** : appelle `stopMotionTimerAndActionManager()` **AVANT** `svgPrintEndOfFile()`.
5. **`Main.cpp sigintHandler`** : appelle `stopExtraActions()` avant `svgPrintEndOfFile()`
   pour que Ctrl+C produise un SVG valide.

### Sémantique de `svgPrintEndOfFile()`

Comme dans PMX, `Robot::svgPrintEndOfFile()` **ne fait que** `svg_->endHeader()`
(écriture des balises `</g></svg>`). **C'est la responsabilité de l'appelant** d'arrêter
les threads producteurs avant. Documenté dans le header :

```cpp
/*!
 * \warning Avant d'appeler cette methode, l'appelant DOIT avoir arrete les
 *          threads producteurs SVG (asserv CBOR, scheduler des timers).
 *          Sequence type : robot.stopExtraActions(); robot.svgPrintEndOfFile();
 */
void svgPrintEndOfFile();
```

### Optimisations temps-réel 2026

Objectif : réduire la latence worst-case des threads critiques (serial, I2C) de ~10-50ms à ~200-500µs, **sans PREEMPT_RT**.

Prérequis kernel (en cours) : `CONFIG_PREEMPT=y`, `CONFIG_HZ=1000`, I2C déjà en 400 kHz.

- ✅ **1. `mlockall()` — Verrouiller la mémoire**

  - Appeler `mlockall(MCL_CURRENT | MCL_FUTURE)` au début de `main()`, avant de lancer les threads
  - Empêche les page faults (stalls de 1-10ms) en verrouillant toute la mémoire en RAM
  - Coût : quelques Mo de RAM supplémentaires (négligeable sur 512 Mo)
- ✅ **2. `SCHED_FIFO` par thread — Priorités temps-réel**

  - Assigner des priorités différenciées aux threads selon leur criticité
  - POSIX SCHED_FIFO : 99 = max, 1 = min, 0 = pas temps-réel (SCHED_OTHER)
  - Nécessite root ou `CAP_SYS_NICE`


  | Thread                        | Priorité avant | Priorité après | Mode     | Rôle              |
  | ------------------------------- | ----------------- | ------------------ | ---------- | -------------------- |
  | **AsservDriver** (driver-arm) | 3               | **80**           | ARM      | Comm série Teensy |
  | **AsservEsialR**              | 2               | **75**           | SIMU     | Boucle PID interne |
  | **ActionManagerTimer**        | 2               | **60**           | ARM+SIMU | Actions/servos     |
  | **Main**                      | 50              | **50**           | ARM+SIMU | Init/orchestration |
  | **DecisionMakerIA**           | 3               | **40**           | ARM+SIMU | Stratégie match   |
  | **LoggerFactory**             | 0               | **0**            | ARM+SIMU | Logs best-effort   |
- ✅ **3. UART `ASYNC_LOW_LATENCY` — Réduire la latence série**

  - Activer le flag `ASYNC_LOW_LATENCY` via `ioctl(TIOCSSERIAL)` après `open()` du port série
  - Réduit la latence de réveil du `read()` de ~10ms à <1ms
  - Critique pour la communication haute fréquence avec la Teensy
- ✅ **4. `clock_nanosleep` absolu — Boucles sans dérive**

  - Remplacer les `sleep_for()` / `usleep()` par `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ...)`
  - Cible un instant absolu : si le traitement prend 2ms sur une période de 5ms, le sleep ne dure que 3ms
  - Insensible aux changements d'horloge (NTP)
- ✅ **5. Priorités IRQ via `chrt` — Protéger serial/I2C du WiFi**

  - Script init qui monte la priorité des threads IRQ UART et I2C au-dessus du WiFi :
    - IRQ UART : `chrt -f -p 90`
    - IRQ I2C : `chrt -f -p 85`
    - WiFi : reste à la priorité par défaut (50)
  - Empêche un transfert WiFi de bloquer les threads critiques

## Navigator — Refactoring navigation (planifié)

### Problème

La logique de retry (boucle while + gestion obstacle/collision/recul) est dupliquée dans :

- `Robot::whileDoLine()` — retry sur `doLine()`
- `IAbyPath::whileMoveForwardTo()` — retry sur `doMoveForwardTo()` ou `doPathForwardTo()`
- `IAbyPath::whileMoveBackwardTo()` — retry sur `doMoveBackwardTo()` ou `doPathBackwardTo()`
- `IAbyPath::whileMoveRotateTo()` — retry sur rotation
- `IAbyPath::whileMoveForwardAndRotateTo()` — retry sur avance + rotation
- `IAbyPath::whileMoveBackwardAndRotateTo()` — retry sur recul + rotation

6 copies du même pattern while/retry/obstacle/collision/recul.

### Solution : classe Navigator

Nouvelle classe `src/common/navigator/Navigator` qui factorise toute la logique de retry en une seule méthode `executeWithRetry()`.

#### Hiérarchie des couches

```
┌───────────────────────────────────────────────────────────────────┐
│  NIVEAU 3 — Stratégie / Match                                     │
│  (O_State_DecisionMakerIA, actions de zone)                       │
│  Utilise Navigator + IAbyPath (zones, actions)                    │
└──────────────────────────────┬────────────────────────────────────┘
                               │
┌──────────────────────────────▼────────────────────────────────────┐
│  NIVEAU 2 — Navigator (NOUVEAU)                                   │
│                                                                   │
│  Retry unifié : executeWithRetry()                                │
│                                                                   │
│  Mouvements :    line, goTo, goBackTo                             │
│  Waypoints :     manualPath(wps, policy, PathMode)                │
│  Pathfinding :   pathTo, pathBackTo (A* via IAbyPath)             │
│  Rotations :     rotateDeg, rotateAbsDeg, faceTo, faceBackTo      │
│  Combinaisons :  goToAndRotateAbsDeg, goToAndRotateRelDeg,        │
│                  goToAndFaceTo, goToAndFaceBackTo,                 │
│                  moveForwardToAndFaceTo, moveForwardToAndFaceBackTo│
│                  pathToAndRotateAbsDeg, pathToAndRotateRelDeg,     │
│                  pathToAndFaceTo, pathToAndFaceBackTo              │
└──────────────────────────────┬────────────────────────────────────┘
                               │
                ┌──────────────┼──────────────┐
                ▼                             ▼
┌───────────────────────┐           ┌───────────────────────┐
│  NIVEAU 1 — Asserv    │           │  IAbyPath              │
│  (inchangé)           │           │  (inchangé)            │
│                       │           │                       │
│  doLine()             │           │  playgroundFindPath() │
│  doMoveForwardTo()    │           │  → FoundPath*         │
│  doMoveBackwardTo()   │           │                       │
│  gotoXY()             │           │  + zones, actions,    │
│  gotoReverse()        │           │    playground, toSVG  │
│  doFaceTo()           │           │                       │
│  doAbsoluteRotateTo() │           └───────────────────────┘
│  doRelativeRotateDeg()│
│  resetEmergencyOnTraj │
│  ()                   │
└───────────┬───────────┘
            │
┌───────────▼───────────┐
│  NIVEAU 0             │
│  AAsservDriver        │
│  (inchangé)           │
│                       │
│  motion_DoLine()      │
│  motion_Goto()        │
│  motion_GotoChain()   │
│  waitEndOfTraj()      │
│                       │
│  Implémentations :    │
│  ARM, SIMU, EsialR    │
└───────────────────────┘
```

#### Dépendances Navigator

```
Navigator
   ├──► Asserv (via Robot*) — mouvements directs (1 tentative)
   └──► IAbyPath — calcul chemin A* → liste de waypoints
```

#### API Navigator

```cpp
struct Waypoint { float x; float y; bool reverse = false; };

struct RetryPolicy {
    int waitTempoUs;            // attente entre tentatives (us)
    int maxObstacleRetries;     // nb max retry obstacle
    int maxCollisionRetries;    // nb max retry collision
    int reculObstacleMm;        // recul apres obstacle (0=aucun)
    int reculCollisionMm;       // recul apres collision (0=aucun)
    bool ignoreCollision;       // ne pas interrompre sur collision

    // Note : la detection adversaire est automatiquement ignoree pendant
    // toute rotation, via MovementType::ROTATION dans
    // Asserv::waitEndOfTrajWithDetection. Aucun flag dedie n'est necessaire.

    // Presets
    static RetryPolicy noRetry();      // 1 essai, pas de retry
    static RetryPolicy standard();     // 2 essais, pas de recul
    static RetryPolicy aggressive();   // 5 essais, recul 50mm
    static RetryPolicy patient();      // 20 essais obstacle
};

// Mode d'execution des waypoints
enum PathMode {
    STOP,            // envoi un par un, arret a chaque point (gotoXY + wait)
    CHAIN,           // envoi groupe, arret a chaque point (gotoXY sans wait intermediaire)
    CHAIN_NONSTOP    // envoi groupe, pas d'arret (gotoChain sans wait + gotoXY final)
};

class Navigator {
public:
    Navigator(Robot* robot, IAbyPath* iap);

    // --- Mouvements simples (defaut: pas de retry) ---
    TRAJ_STATE line(float distMm, RetryPolicy policy = RetryPolicy::noRetry());
    TRAJ_STATE goTo(float x, float y, RetryPolicy policy = RetryPolicy::noRetry());
    TRAJ_STATE goBackTo(float x, float y, RetryPolicy policy = RetryPolicy::noRetry());

    // --- Mouvements composes (rotation + ligne droite, defaut: pas de retry) ---
    TRAJ_STATE moveForwardTo(float x, float y, RetryPolicy policy = RetryPolicy::noRetry());
    TRAJ_STATE moveBackwardTo(float x, float y, RetryPolicy policy = RetryPolicy::noRetry());

    // --- Rotations (defaut: pas de retry) ---
    TRAJ_STATE rotateDeg(float degRelative, RetryPolicy policy = RetryPolicy::noRetry());
    TRAJ_STATE rotateAbsDeg(float thetaDeg, RetryPolicy policy = RetryPolicy::noRetry());
    TRAJ_STATE faceTo(float x, float y, RetryPolicy policy = RetryPolicy::noRetry());
    TRAJ_STATE faceBackTo(float x, float y, RetryPolicy policy = RetryPolicy::noRetry());

    // --- Suite de waypoints manuels ---
    TRAJ_STATE manualPath(const std::vector<Waypoint>& waypoints,
                          RetryPolicy policy = RetryPolicy::standard(),
                          PathMode mode = STOP);

    // --- Pathfinding A* (IAbyPath calcule le chemin) ---
    TRAJ_STATE pathTo(float x, float y,
                      RetryPolicy policy = RetryPolicy::standard(),
                      PathMode mode = STOP);
    TRAJ_STATE pathBackTo(float x, float y,
                          RetryPolicy policy = RetryPolicy::standard(),
                          PathMode mode = STOP);

    // --- Combinaisons mouvement + rotation finale ---
    // goTo combos
    TRAJ_STATE goToAndRotateAbsDeg(float x, float y, float thetaDeg, RetryPolicy policy = RetryPolicy::standard());
    TRAJ_STATE goToAndRotateRelDeg(float x, float y, float degRelative, RetryPolicy policy = RetryPolicy::standard());
    TRAJ_STATE goToAndFaceTo(float x, float y, float fx, float fy, RetryPolicy policy = RetryPolicy::standard());
    TRAJ_STATE goToAndFaceBackTo(float x, float y, float fx, float fy, RetryPolicy policy = RetryPolicy::standard());
    // moveForwardTo combos
    TRAJ_STATE moveForwardToAndRotateAbsDeg(float x, float y, float thetaDeg, RetryPolicy policy = RetryPolicy::standard());
    TRAJ_STATE moveForwardToAndRotateRelDeg(float x, float y, float degRelative, RetryPolicy policy = RetryPolicy::standard());
    TRAJ_STATE moveForwardToAndFaceTo(float x, float y, float fx, float fy, RetryPolicy policy = RetryPolicy::standard());
    TRAJ_STATE moveForwardToAndFaceBackTo(float x, float y, float fx, float fy, RetryPolicy policy = RetryPolicy::standard());
    // pathTo combos
    TRAJ_STATE pathToAndRotateAbsDeg(float x, float y, float thetaDeg, RetryPolicy policy = RetryPolicy::standard());
    TRAJ_STATE pathToAndRotateRelDeg(float x, float y, float degRelative, RetryPolicy policy = RetryPolicy::standard());
    TRAJ_STATE pathToAndFaceTo(float x, float y, float fx, float fy, RetryPolicy policy = RetryPolicy::standard());
    TRAJ_STATE pathToAndFaceBackTo(float x, float y, float fx, float fy, RetryPolicy policy = RetryPolicy::standard());

private:
    // Coeur unique : boucle while/retry/obstacle/collision/recul
    TRAJ_STATE executeWithRetry(std::function<TRAJ_STATE()> moveFunc,
                                 const RetryPolicy& policy, int reculDir = -1);

    // Calcule les waypoints via IAbyPath::playgroundFindPath()
    std::vector<Waypoint> computePath(float x, float y, bool reverse = false);

    Robot* robot_;
    IAbyPath* iap_;
};
```

#### PathMode — modes d'execution des waypoints

```
STOP :          ●──stop──●──stop──●──stop──●   envoi 1 par 1, arret chaque point
CHAIN :         ●──stop──●──stop──●──stop──●   envoi groupe, arret chaque point
CHAIN_NONSTOP : ●────────●────────●──stop──●   envoi groupe, pas d'arret (sauf dernier)
```


| PathMode        | Envoi            | Arret entre points | Commande intermediaire     | Commande finale            |
| ----------------- | ------------------ | -------------------- | ---------------------------- | ---------------------------- |
| `STOP`          | un par un + wait | oui                | `gotoXY` + `waitEndOfTraj` | `gotoXY` + `waitEndOfTraj` |
| `CHAIN`         | groupé          | oui                | `gotoXY` (sans wait)       | `gotoXY` + `waitEndOfTraj` |
| `CHAIN_NONSTOP` | groupé          | non                | `gotoChain` (sans wait)    | `gotoXY` + `waitEndOfTraj` |

La Nucleo (asserv_chibios) gère la queue de waypoints en interne.


| Driver       | STOP | CHAIN         | CHAIN_NONSTOP            |
| -------------- | ------ | --------------- | -------------------------- |
| ARM (série) | OK   | OK            | OK                       |
| SIMU         | OK   | fallback STOP | fallback STOP            |
| EsialR       | OK   | fallback STOP | OK (addGoToEnchainement) |

**Envoi sans attente (mode CHAIN/CHAIN_NONSTOP)** :

Navigator utilise `Asserv::gotoSend()` / `gotoChainSend()` (envoi sans `waitEndOfTraj`)
puis `Asserv::waitTraj()` une seule fois a la fin. Sur le driver ARM, les commandes
sont envoyées en série a la Nucleo qui les bufferise dans sa queue. Sur EsialR,
`motion_GotoChain` utilise `addGoToEnchainement` (non-bloquant, queue interne),
tandis que `motion_Goto` reste bloquant (fallback STOP pour le mode CHAIN).

**Note SIMU / EsialR** :

- Les 3 modes fonctionnent sans erreur.
- CHAIN_NONSTOP utilise `addGoToEnchainement` d'EsialR (enchainement réel, pas de fallback).
- CHAIN utilise `motion_Goto` qui est bloquant en EsialR (comportement identique a STOP).
- Sur le vrai robot (Nucleo), CHAIN_NONSTOP donne une trajectoire fluide sans arret.

#### Tracé SVG des trajectoires

Navigator trace automatiquement les segments sur le SVG via `SvgWriter::writeLine()`.


| Commande                 | Couleur SVG                           | Style                 |
| -------------------------- | --------------------------------------- | ----------------------- |
| manualPath STOP          | *(rien, points odométrie existants)* |                       |
| manualPath CHAIN         | *(rien, points odométrie existants)* |                       |
| manualPath CHAIN_NONSTOP | Vert                                  | Pointillés`- - - -`  |
| goTo / goBackTo (direct) | Bleu                                  | Continu`──────` |
| pathTo STOP              | Rouge                                 | Continu`──────` |
| pathTo CHAIN             | Rouge                                 | Continu`──────` |
| pathTo CHAIN_NONSTOP     | Rouge                                 | Pointillés`- - - -`  |

#### Fichiers


| Fichier                                | Contenu                                                   |
| ---------------------------------------- | ----------------------------------------------------------- |
| `src/common/navigator/RetryPolicy.hpp` | Struct RetryPolicy + presets                              |
| `src/common/navigator/Navigator.hpp`   | Header classe Navigator + struct Waypoint + enum PathMode |
| `src/common/navigator/Navigator.cpp`   | Implémentation (executeWithRetry + toutes les méthodes) |

#### Ce qui reste inchangé dans IAbyPath

- Zones : `ia_createZone()`, `ia_getZone()`, `goToZone()`
- Actions séquentielles : `ia_addAction()`, `ia_start()`
- Playground : `playgroundFindPath()`, `enable()`, `toSVG()`
- Les méthodes `doPath*` et `while*` existantes restent en place (deprecated progressivement)

#### Migration (incrémentale)

Les méthodes `while*` de IAbyPath sont supprimées (remplacées par Navigator avec RetryPolicy).
Les méthodes `doPath*` de IAbyPath sont deprecated mais encore utilisées en interne :

- `doPathForwardTo`, `doPathBackwardTo`, `doPathForwardAndFaceTo`, `doPathForwardAndRotateTo`
- Appelants restants : IAbyPath interne + `O_IAbyPathTest.cpp`
- À migrer vers Navigator::pathTo / pathBackTo / pathToAndFaceTo / pathToAndRotateAbsDeg

#### Refactoring nommage (done)

Convention : **verbe + complément + qualificateur**, pas de préfixe `do`.


| Action                     | Navigator (niveau 2) | Asserv (ancien → nouveau)                                | AAsservDriver (ancien → nouveau)                   |
| ---------------------------- | ---------------------- | ----------------------------------------------------------- | ----------------------------------------------------- |
| Ligne droite               | `line`               | `doLine` → `line`                                        | `motion_DoLine` → `motion_Line`                    |
| Aller à (x,y)             | `goTo`               | `gotoXY` → `goTo`                                        | `motion_Goto` → `motion_GoTo`                      |
| Reculer à (x,y)           | `goBackTo`           | `gotoReverse` → `goBackTo`                               | `motion_GotoReverse` → `motion_GoBackTo`           |
| Aller chaîné             | —                   | `gotoChain` → `goToChain`                                | `motion_GotoChain` → `motion_GoToChain`            |
| Reculer chaîné           | —                   | `gotoReverseChain` → `goBackToChain`                     | `motion_GotoReverseChain` → `motion_GoBackToChain` |
| Envoi goto sans wait       | —                   | `gotoSend` → `goToSend`                                  | —                                                  |
| Envoi chain sans wait      | —                   | `gotoChainSend` → `goToChainSend`                        | —                                                  |
| Envoi back sans wait       | —                   | `gotoReverseSend` → `goBackToSend`                       | —                                                  |
| Envoi back chain sans wait | —                   | `gotoReverseChainSend` → `goBackToChainSend`             | —                                                  |
| Attente fin traj           | —                   | `waitTraj` ✅                                             | `waitEndOfTraj` ✅                                  |
| Rotation relative deg      | `rotateDeg`          | `doRelativeRotateDeg` → `rotateDeg`                      | `motion_DoRotate` → `motion_RotateRad` (radians)   |
| Rotation relative rad      | —                   | `doRelativeRotateRad` → `rotateRad`                      | — (le driver est toujours en rad)                  |
| Rotation par couleur deg   | —                   | `doRelativeRotateByMatchColor` → `rotateByMatchColorDeg` | —                                                  |
| Rotation absolue deg       | `rotateAbsDeg`       | `doAbsoluteRotateTo` → `rotateAbsDeg`                    | —                                                  |
| Face vers (x,y)            | `faceTo`             | `doFaceTo` → `faceTo`                                    | `motion_DoFace` → `motion_FaceTo`                  |
| Dos vers (x,y)             | `faceBackTo`         | — →`faceBackTo`                                         | — →`motion_FaceBackTo`                            |
| Avancer vers (x,y)         | —                   | `doMoveForwardTo` → `moveForwardTo`                      | —                                                  |
| Reculer vers (x,y)         | —                   | `doMoveBackwardTo` → `moveBackwardTo`                    | —                                                  |
| Avancer+rotation           | —                   | `doMoveForwardAndRotateTo` → `moveForwardAndRotateTo`    | —                                                  |
| Reculer+rotation           | —                   | `doMoveBackwardAndRotateTo` → `moveBackwardAndRotateTo`  | —                                                  |
| Calage                     | —                   | `doCalage` → `calage`                                    | —                                                  |
| Calage2                    | —                   | `doCalage2` → `calage2`                                  | —                                                  |
| CalageNew                  | —                   | `doCalageNew` → `calageNew`                              | —                                                  |
| Pivot gauche               | —                   | `doRunPivotLeft` → `pivotLeft`                           | —                                                  |
| Pivot droit                | —                   | `doRunPivotRight` → `pivotRight`                         | —                                                  |
| Orbital turn deg           | `orbitalTurnDeg`     | `orbitalTurnDeg`                                          | `motion_DoOrbitalTurn` → `motion_OrbitalTurnRad`   |

**Convention unités dans les noms** :

- `Deg` dans le nom = paramètre en degrés (Navigator, Asserv)
- `Rad` dans le nom = paramètre en radians (Driver)
- La conversion deg→rad se fait **une seule fois** dans Asserv avant d'appeler le driver
- Le driver travaille **toujours en radians**
- Navigator travaille **toujours en degrés**

**TODO drivers ARM (commandes non supportées)** :

- `AsservDriver` (ASCII) : `motion_FaceBackTo` → à implémenter côté Nucleo (commande série)

Fichiers impactés : `AAsservDriver.hpp` (8), `Asserv.hpp/.cpp` (18),
`Navigator.hpp/.cpp` (1), drivers ARM/SIMU/EsialR (8 chacun),
appelants (tests, IA, match) (~50+ occurrences).

#### Exemple avant/après

```cpp
// AVANT (10 paramètres, illisible)
robot.ia().iAbyPath().whileMoveForwardAndRotateTo(
    zone.x, zone.y, radToDeg(zone.theta),
    ROTATION_WITH_DETECTION, 2000000, 20, 10, NO_PATHFINDING);

// APRÈS
nav.goToAndRotateAbsDeg(zone.x, zone.y, radToDeg(zone.theta),
    RetryPolicy::patient());

// AVANT (waypoints manuels, pas possible directement)
robot.asserv().doMoveForwardTo(300, 500);
robot.asserv().doMoveForwardTo(600, 800);
robot.asserv().doMoveForwardTo(900, 800);

// APRÈS — arret a chaque point (defaut)
nav.manualPath({{300, 500}, {600, 800}, {900, 800}},
    RetryPolicy::aggressive());

// APRÈS — envoi groupe, arret a chaque point
nav.manualPath({{300, 500}, {600, 800}, {900, 800}},
    RetryPolicy::aggressive(), CHAIN);

// APRÈS — envoi groupe, trajectoire fluide (le plus rapide)
nav.manualPath({{300, 500}, {600, 800}, {900, 800}},
    RetryPolicy::aggressive(), CHAIN_NONSTOP);

// AVANT (pathfinding)
robot.ia().iAbyPath().whileMoveForwardTo(
    x, y, true, 2000000, 5, 5, true, 50, 0, false);

// APRÈS — pathfinding avec arret a chaque waypoint (defaut)
nav.pathTo(x, y, RetryPolicy::aggressive());

// APRÈS — pathfinding en trajectoire fluide
nav.pathTo(x, y, RetryPolicy::aggressive(), CHAIN_NONSTOP);

// --- Rotations ---

// AVANT
robot.asserv().doRelativeRotateDeg(90);
robot.asserv().doAbsoluteRotateTo(180);
robot.asserv().doFaceTo(x, y);

// APRÈS — sans retry (1 seul essai, equivalent asserv direct)
nav.rotateDeg(90, RetryPolicy::noRetry());
nav.rotateAbsDeg(180, RetryPolicy::noRetry());
nav.faceTo(x, y, RetryPolicy::noRetry());
nav.faceBackTo(x, y, RetryPolicy::noRetry());

// APRÈS — avec retry
nav.rotateDeg(90, RetryPolicy::aggressive());
nav.rotateAbsDeg(180, RetryPolicy::patient());
nav.faceTo(x, y, RetryPolicy::standard());
```

#### Évolution future : DecisionMaker + interruptions

Toutes les commandes Navigator sont **bloquantes** : elles retournent un `TRAJ_STATE` final
après avoir épuisé les retries de la RetryPolicy. Navigator ne prend **aucune décision
stratégique** — il exécute et remonte le résultat.

C'est le **DecisionMaker** (niveau 3) qui décide quoi faire après un échec :
changer de cible, contourner, abandonner, retenter avec une autre policy, etc.

```
┌─────────────────────────────────────────────────────────┐
│  DecisionMaker (niveau 3) — DÉCIDE                       │
│                                                         │
│  ts = nav.goTo(x, y, RetryPolicy::aggressive());       │
│                                                         │
│  if (ts == TRAJ_NEAR_OBSTACLE)                          │
│      → stratégie : contourner ? changer de cible ?      │
│  if (ts == TRAJ_COLLISION)                              │
│      → stratégie : reculer ? essayer un autre chemin ?  │
│  if (ts == TRAJ_IMPOSSIBLE)                             │
│      → stratégie : abandonner cette action ?            │
│  if (ts == TRAJ_FINISHED)                               │
│      → action suivante                                  │
└────────────────────────┬────────────────────────────────┘
                         │
┌────────────────────────▼────────────────────────────────┐
│  Navigator (niveau 2) — EXÉCUTE                          │
│                                                         │
│  Retry mécanique selon RetryPolicy                      │
│  Aucune décision stratégique                            │
│  Retourne TRAJ_STATE final au DecisionMaker             │
└─────────────────────────────────────────────────────────┘
```

Cette séparation permet de :

- Tester Navigator indépendamment (retry pur, pas de logique métier)
- Faire évoluer la stratégie du DecisionMaker sans toucher à Navigator
- Gérer les interruptions (fin de match 90s, changement de priorité) au niveau DecisionMaker

## Options post-compétition 2027

Idées d'extraction de modules indépendants dans `libs/`, pour réutilisation sur de futurs robots ou projets. Non prioritaire pour 2026 — à considérer une fois la compétition passée.

### Option 1 — Beacon autonome (`libs/beacon/`)

Extraire le driver beacon comme librairie indépendante. C'est le candidat le plus naturel : sous-système fonctionnel quasi-autonome, zéro dépendance robot.

**Fichiers concernés :**

| Fichier | Rôle | Dépendances externes |
|---|---|---|
| `BeaconSensors.hpp/cpp` | Driver I2C beacon (lecture registres, écriture settings) | AsI2cAtomic, Logger (optionnel) |
| `AsI2cAtomic.hpp` | Wrapper I2C Linux avec repeated start + bus recovery | Linux `<linux/i2c-dev.h>` uniquement |
| Structs `Registers`, `Settings` | Structures de données partagées | Aucune (plain structs) |

**Pourquoi c'est facile :**
- ~500 lignes, pur hardware driver
- Seule dépendance réelle : Linux I2C (ioctl)
- Le Logger est injectable/supprimable
- Interface claire : `begin()`, `getDataFull()`, `readSettings()`, `writeXxx()`
- Déjà isolé derrière l'interface abstraite `ASensorsDriver`

**Structure cible :**
```
libs/beacon/                          # Git submodule indépendant
├── CMakeLists.txt                    # target STATIC : pmx-beacon
├── include/
│   ├── BeaconSensors.hpp             # Driver + structs Settings/Registers
│   └── AsI2cAtomic.hpp              # Wrapper I2C Linux (header-only)
├── src/
│   └── BeaconSensors.cpp
└── test/
    ├── BeaconSensorsTest.cpp         # Tests unitaires driver (mock I2C ou loopback)
    └── AsI2cAtomicTest.cpp           # Tests wrapper I2C
```

**Séparation lib / robot — ce qui est exporté vs ce qui reste :**

```
libs/beacon/ (LIB)                         robot/ (PMX-CORTEX)
─────────────────────                      ──────────────────────────────
BeaconSensors          ◄── utilisé par ──► SensorsDriver (orchestration)
  begin()                                    agrège beacon + capteurs proximité
  getDataFull()                              synchronisation positions
  readSettings()                             buffer circulaire timestamps
  writeXxx()
                                           Sensors (threading + détection)
AsI2cAtomic                                  SensorsThread, DetectionEvent
  readReg() / writeReg()                     ObstacleZone, filtrage niveaux 0-4
  open() / recover()                         arrêt moteurs si obstacle

Settings / Registers                       MenuBeaconLCDTouch (UI)
  plain structs partagées                    menu LCD tactile pré-match
                                             configuration couleur/stratégie
```

**Intégration CMake dans PMX-CORTEX :**

```cmake
# Dans le CMakeLists.txt racine de PMX-CORTEX :
add_subdirectory(libs/beacon)

# Dans robot/CMakeLists.txt :
target_link_libraries(pmx-driver-arm pmx-common pmx-beacon ...)
target_include_directories(pmx-driver-arm PRIVATE ${CMAKE_SOURCE_DIR}/libs/beacon/include)
```

```
PMX-CORTEX/
├── libs/
│   ├── beacon/          ← git submodule (réutilisable seul)
│   ├── simple-svg/      ← git submodule (existant)
│   ├── PathFinding/     ← git submodule (existant)
│   └── ...
└── robot/
    └── src/driver-arm/
        └── SensorsDriver.cpp   ← #include <BeaconSensors.hpp> depuis libs/beacon
```

**Contraintes identifiées (TODO avant extraction) :**
- Logger : BeaconSensors et AsI2cAtomic utilisent `log/LoggerFactory.hpp`. Rendre optionnel (`#ifdef` ou callback) pour que la lib compile sans le système de log PMX.
- Bus recovery : voir section ci-dessous.

**Bus recovery : séparation générique / spécifique SoC**

Aujourd'hui `AsI2cAtomic` mélange deux choses :
- **Partie générique Linux** : `open()`, `readReg()`, `writeReg()` via `ioctl(I2C_RDWR)` — fonctionne sur tout SoC Linux (RPi, EV3, BeagleBone, etc.)
- **Partie spécifique i.MX6ULL** : `recover()` et `resetBusOnce()` font un unbind/rebind sysfs avec des device names hardcodés (`21a0000.i2c`, `/sys/bus/platform/drivers/imx-i2c/`)

Sur un autre SoC, seule la stratégie de recovery change. La solution : un **callback optionnel** passé au constructeur.

```cpp
// Dans la lib beacon (générique) :
using RecoveryFunc = std::function<void(int bus)>;

AsI2cAtomic(int bus, uint8_t addr, RecoveryFunc recovery = nullptr);
//                                  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//                     nullptr = pas de recovery (mode lib pure)
//                     callback = appelé après N erreurs consécutives
```

```cpp
// Dans PMX-CORTEX robot/ (spécifique OPOS6UL) :
auto imxRecovery = [](int bus) {
    const char* dev = (bus == 0) ? "21a0000.i2c" : "21a4000.i2c";
    // unbind/rebind /sys/bus/platform/drivers/imx-i2c/
};
AsI2cAtomic i2c(0, 0x2D, imxRecovery);
```

```
AsI2cAtomic (libs/beacon/ — générique Linux)
  ├── open(), readReg(), writeReg()    ← ioctl I2C_RDWR, partout pareil
  └── recover()                        ← appelle le callback si fourni

Stratégies de recovery (dans robot/ — spécifique plateforme) :
  ├── imxRecovery       → unbind/rebind sysfs imx-i2c (OPOS6UL i.MX6ULL)
  ├── rpiRecovery       → dtoverlay ou gpio bitbang (Raspberry Pi)
  ├── ev3Recovery       → /sys/class/lego-port/ (EV3)
  └── nullptr           → pas de recovery (fallback)
```

### Option 2 — Socle technique (thread / log / utils / timer)

Extraire les briques d'infrastructure comme librairies empilées. Plus gros chantier, mais permettrait de réutiliser le runtime complet sur d'autres projets embarqués Linux.

**Ordre d'extraction (dépendances ascendantes) :**

```
                    ┌──────────┐
                    │  timer   │  ActionTimerScheduler
                    └────┬─────┘  (dépend de thread + log + utils + action/IAction)
                         │
              ┌──────────┴──────────┐
              │                     │
         ┌────┴────┐          ┌────┴────┐
         │   log   │          │  utils  │  Time, Chronometer, Arguments
         └────┬────┘          └─────────┘  (POSIX uniquement)
              │  (Logger, LoggerFactory,
              │   Level, appenders)
         ┌────┴────┐
         │ thread  │  Thread, Mutex
         └─────────┘  (POSIX pthreads, zéro dépendance)
```

**Phase 1 — `libs/thread/`** (zéro dépendance, extractible immédiatement)
- `Thread.hpp/cpp`, `Mutex.hpp/cpp`
- Pur POSIX pthreads, utilisé par 30+ fichiers

**Phase 2 — `libs/log/`** (dépend de thread)
- Logger, LoggerFactory, Level, appenders
- LoggerFactory hérite de Thread → nécessite libs/thread
- SvgWriter dépend de simple-svg → optionnel, peut rester dans robot/

**Phase 3 — `libs/utils/`** (composants indépendants entre eux)
- Time, Chronometer → POSIX uniquement
- Arguments → stdlib uniquement
- json.hpp → déjà une lib tierce header-only

**Phase 4 — `libs/timer/`** (dépend de tout le reste + IAction)
- ActionTimerScheduler couplé à `action/IAction` → nécessite une interface abstraite
- Le plus complexe à extraire, gain discutable

**Difficulté :** les dépendances croisées rendent l'extraction progressive — chaque couche dépend des précédentes. Le timer est le plus problématique (couplage avec IAction).
