# Architecture robot/ — PMX-CORTEX

## Migrations en cours

- [Configuration hardware dynamique](HARDWARE_CONFIG.md) — Activation/désactivation des drivers via fichier `hardware.conf` pour l'intégration progressive
- [Communication asserv](ASSERV_MIGRATION_COMMUNICATION.md) — Migration communication série ancien raspIO → SerialIO/RaspIO

### Règles de migration PMX → PMX-CORTEX

1. **Validation obligatoire** — Chaque modification doit être validée par l'utilisateur avant d'être appliquée. Aucun changement sans confirmation et explication pertinente.
2. **Conserver la configuration robot PMX** — Les paramètres, réglages et configurations du robot PMX doivent être préservés tels quels. Aucune modification de configuration sans validation et justification claire.
3. **Dossier `old/`** — Les fichiers remplacés ne sont pas supprimés : ils sont déplacés dans un dossier `old/` au même niveau. Cela permet de comparer et de revenir en arrière. On nettoiera une fois que tout fonctionne.
4. **Comparaison PMX2025** — En cas de doute, comparer avec la version PMX2025 (ancien projet `/home/pmx/git/PMX/`).
5. **Vérification PlotJuggler** — Après chaque étape qui compile et fonctionne, vérifier que les courbes d'asservissement sont toujours disponibles dans PlotJuggler (`plotjuggler_asservstream_plugin`). La communication série et le format des données doivent rester compatibles.

## Hardware status LEDs

Au démarrage, les 8 LEDs vertes s'allument (lamp test 500ms), puis chacune s'éteint si le composant est OK. **LED allumée = erreur.**

| LED | Composant | Bus | Classe driver |
|---|---|---|---|
| 0 | LcdShield | MCP23017 I2C | `LcdShieldDriver` |
| 1 | Tirette/Switch | PCA9555 I2C | `SwitchDriver` |
| 2 | BeaconSensors | Teensy I2C | `SensorsDriver` |
| 3 | GroveColorSensor | TCS3414 I2C | `GroveColorSensor` |
| 4 | Servos AX12 | Teensy CCAx12 I2C | `ServoDriver` |
| 5 | *(réserve)* | | |
| 6 | *(réserve)* | | |
| 7 | AsservDriver | Nucleo série USB | `AsservDriver` |

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
│   │   │   ├── Sensors.cpp/hpp           # ◆ TIMER onTimer (variable ms) — lecture beacon + détection obstacle
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
│   ├── common/                         # Common-UnitTest
│   │   ├── Main.cpp                    #   point d'entrée
│   │   ├── LoggerInitialize.cpp
│   │   ├── DriverStubs.cpp             #   stubs interfaces driver pour linker
│   │   ├── ActionManagerTimerTest.cpp/hpp
│   │   ├── ChronometerTest.cpp/hpp
│   │   ├── LoggerTest.cpp/hpp
│   │   ├── ReadWriteTest.cpp/hpp
│   │   ├── ThreadTest.cpp/hpp
│   │   └── TimerFactoryTest.cpp/hpp
│   │
│   ├── driver/                         # Driver-UnitTest (tests partagés ARM+SIMU)
│   │   ├── Main.cpp
│   │   ├── LoggerInitialize.cpp
│   │   ├── AsservDriverTest.cpp/hpp
│   │   ├── SensorDriverTest.cpp/hpp
│   │   ├── LedDriverTest.cpp/hpp
│   │   ├── ServoDriverTest.cpp/hpp
│   │   ├── SwitchDriverTest.cpp/hpp
│   │   ├── ColorDriverTest.cpp/hpp     #   (ARM seulement)
│   │   ├── LcdShieldDriverTest.cpp/hpp #   (ARM seulement)
│   │   └── ButtonDriverTest.cpp/hpp    #   (SIMU seulement)
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

| Target | Type | Sources | Dépendances |
|--------|------|---------|-------------|
| `pmx-common` | STATIC | `src/common/**` | pthread |
| `pmx-driver-arm` | STATIC | `src/driver-arm/**` | pmx-common, as_devices |
| `pmx-driver-simu` | STATIC | `src/driver-simu/**` | pmx-common |
| `pmx-suite` | STATIC | `test/suite/**` | pmx-common |
| `common-test` | EXE | `test/common/**` | pmx-suite + **driver auto** |
| `driver-test` | EXE | `test/driver/**` | pmx-suite + **driver auto** |
| `opos6ul` | EXE | `src/bot/opos6ul/**` | pmx-common + **driver auto** |
| `bot2` | EXE | `src/bot/bot2/**` | pmx-common + **driver auto** |

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

| Ancien projet Eclipse | Nouveau dans robot/ |
|---|---|
| `Common-UnitTest_SIMU` | `cmake --preset simu-debug && cmake --build --preset simu-debug --target common-test` |
| `Common-UnitTest_OPOS6UL_ARM` | `cmake --preset arm-debug && cmake --build --preset arm-debug --target common-test` |
| `Driver-UnitTest_SIMU` | `cmake --preset simu-debug && cmake --build --preset simu-debug --target driver-test` |
| `Driver-UnitTest_OPOS6UL_ARM` | `cmake --preset arm-debug && cmake --build --preset arm-debug --target driver-test` |
| `Bot_ArmadeusOPOS6UL_SIMU` | `cmake --build --preset simu-debug --target opos6ul` |
| `Bot_ArmadeusOPOS6UL_ARM` | `cmake --build --preset arm-debug --target opos6ul` |
| Virtual folders (linked resources) | `target_link_libraries()` dans CMake |
| Sélection du projet à builder | `--preset` + `--target` |

## Correspondance fichiers ancien PMX → nouveau

| Ancien (PMX/src/) | Nouveau (robot/src/) |
|---|---|
| `Common/Interface.Driver/` | `common/interface/` |
| `Common/Action/` | `common/action/` |
| `Common/Asserv/` | `common/asserv/` |
| `Common/State/` | `common/state/` |
| `Common/IA/` | `common/ia/` |
| `Common/Utils/` | `common/utils/` |
| `Common/*.cpp/hpp` | `common/` (racine) |
| `Log/` | `common/log/` |
| `Thread/` | `common/thread/` |
| `Asserv.Esial/` | `common/asserv-esial/` |
| `Driver-OPOS6UL_ARM/` | `driver-arm/` |
| `Driver-SIMU/` | `driver-simu/` |
| `Bot-OPOS6UL/` | `bot/opos6ul/` |
| `Bot-OPOS6UL.Main/` | `bot/opos6ul/` (Main.cpp + LoggerInitialize.cpp) |

| Ancien (PMX/test/) | Nouveau (robot/test/) |
|---|---|
| `Suite/` | `suite/` |
| `Common-Test.Main/` | `common/` |
| `Driver-Test_OPOS6UL_ARM.Main/` | `driver/` |
| `Driver-Test_SIMU.Main/` | `driver/` |

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
  │     ├── SensorsTimer         TIMER  variable ms       onTimer → lecture beacon + détection obstacle
  │     ├── ServoObjectsTimer    TIMER  50ms              onTimer → interpolation position servos
  │     └── LedBarTimer          TIMER  variable µs       onTimer → animation LEDs
  │
  └─→ O_State_DecisionMakerIA   THREAD prio 3    exécution unique (stratégie match)
```

### Détail des threads (THREAD + LOOP)

| Fichier | Classe | Prio | Loop | Période | Rôle |
|---------|--------|------|------|---------|------|
| `common/asserv-esial/AsservEsialR` | AsservEsialR | 2 | `while(1)` + chronoTimer | **50ms** (20Hz) configurable | Odométrie, PID, commandes mouvement |
| `driver-arm/AsservDriver` | AsservDriver | 2 | `while(1)` + sleep | **100ms** (~10Hz) | Lecture série carte ST, parse position |
| `driver-arm/AsservDriver_mbed_i2c` | AsservDriver | 2 | `while(1)` + chronoTimer | variable | Variante I2C (ancien mbed) |
| `driver-simu/AsservDriver` | AsservDriver | 2 | `while(1)` + timer | variable | Simulation moteurs (math pure) |
| `common/action/ActionManagerTimer` | ActionManagerTimer | 2 | `while(!stop)` + sem_wait | event-driven | Queue d'actions async, gère les TIMER |
| `common/log/LoggerFactory` | LoggerFactory | 0 | `while(!stop)` | non borné | Flush buffers log vers appenders |
| `bot/opos6ul/states/O_State_DecisionMakerIA` | O_State_DecisionMakerIA | 3 | exécution unique | **une fois** | Stratégie match, appels mouvement bloquants |

### Détail des timers (TIMER onTimer)

Tous gérés par `ActionManagerTimer`, enregistrés via `actions().addTimer()`.

| Fichier | Classe | Période | onTimer() fait quoi | Décide ? |
|---------|--------|---------|---------------------|----------|
| `common/action/Sensors` | SensorsTimer | **variable ms** | Lit beacon I2C, filtre niveaux 0-4, stoppe le robot | **OUI — problème (voir refactoring)** |
| `common/action/ServoObjectsSystem` | ServoObjectsTimer | **50ms** | Calcule position interpolée, envoie commande servo | Oui (contrôle moteur servo) |
| `common/action/LedBar` | LedBarTimer | **variable µs** | Anime LEDs (alternate, K2000, blink) | Non (affichage) |
| `bot/opos6ul/tests/O_ActionManagerTimerTest` | TestTimer | **100-500ms** | Log messages | Non (test seulement) |

### Notes

- **AsservEsialR vs AsservDriver** : mutuellement exclusifs. AsservEsialR = asserv interne (PID sur OPOS6UL). AsservDriver = asserv externe (carte ST via série).
- **Priorités SCHED_FIFO** : prio 2 pour le temps réel (asserv, actions), prio 3 pour la stratégie (moins critique que le mouvement).
- **ActionManagerTimer** : ne boucle pas à fréquence fixe, il dort sur un sémaphore et se réveille quand une action est ajoutée (`sem_post`). Il gère aussi tous les TIMER (SIGALRM).
- **SensorsTimer** : le seul timer qui prend des décisions de mouvement — voir section refactoring plus bas.

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

| Groupe | Nb | ARM (carte ST) | SIMU | EsialR | Constat |
|--------|----|----------------|------|--------|---------|
| Moteurs directs (setPower, setPosition, stop) | 7 | STUB | FULL | STUB | SIMU seulement |
| Encodeurs (getEncoder, getCounts, reset) | 11 | STUB | FULL | STUB | SIMU seulement |
| Odométrie (setPosition, getPosition) | 2 | FULL | FULL | FULL | **Commun** |
| Mouvements (doLine, doRotate, goto...) | 8 | FULL | FULL | FULL* | **Commun** |
| Vitesse (setLowSpeed, setMaxSpeed) | 3 | FULL | FULL | PARTIEL | **Commun** |
| Contrôle (freeMotion, activateManager...) | 4 | FULL | FULL | FULL | **Commun** |
| Interruption (interrupt, resetEmergency) | 2 | FULL | FULL | FULL | **Commun** |
| Régulation (activateReguDist/Angle) | 2 | FULL | STUB | FULL | ARM+EsialR |
| Courant moteur (getMotorCurrent) | 2 | STUB | STUB | STUB | **Code mort** |
| Deprecated (path_GetLastCommandStatus) | 1 | returns -1 | returns -1 | returns -1 | **Code mort** |
| **Total** | **~47** | | | | |

*EsialR : GotoChain non implémenté.

### Solution cible : deux interfaces

#### Répartition des 47 méthodes

| Destination | Méthodes | Nb |
|---|---|---|
| **→ AAsserv** (haut niveau) | odo_SetPosition, odo_GetPosition, motion_DoLine, motion_DoRotate, motion_DoFace, motion_Goto, motion_GotoReverse, motion_GotoChain, motion_GotoReverseChain, motion_FreeMotion, motion_DisablePID, motion_AssistedHandling, motion_ActivateManager, motion_setLowSpeedForward, motion_setLowSpeedBackward, motion_setMaxSpeed, path_InterruptTrajectory, path_ResetEmergencyStop, endWhatTodo | **19** |
| **→ AMotorDriver** (bas niveau) | setMotorLeftPower, setMotorRightPower, setMotorLeftPosition, setMotorRightPosition, stopMotorLeft, stopMotorRight, stopMotors, getLeftExternalEncoder, getRightExternalEncoder, getLeftInternalEncoder, getRightInternalEncoder, getCountsExternal, getDeltaCountsExternal, resetEncoders | **14** |
| **→ À décider** (AAsserv ou spécifique) | motion_ActivateReguDist, motion_ActivateReguAngle, motion_DoArcRotate | **3** |
| **→ Fusionnées** (doublons inutiles) | getCountsInternal, resetInternalEncoders, resetExternalEncoders → fusionnés dans AMotorDriver | **3** |
| **→ Supprimées** (code mort) | getMotorLeftCurrent, getMotorRightCurrent, path_GetLastCommandStatus | **3** |
| | **Total ancien** | **~47** |
| | dont réellement utiles | **~36** |

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

## Timers POSIX (ITimerPosixListener)

Le système de timers utilise des signaux POSIX (SIGALRM) gérés par `ActionManagerTimer`.
Chaque timer implémente `ITimerPosixListener` avec un callback `onTimer()` périodique.

### Liste des timers

| Timer | Période | Fichier | onTimer() fait quoi | Décide ? |
|-------|---------|---------|---------------------|----------|
| **SensorsTimer** | variable (ms) | `common/action/Sensors.cpp/hpp` | Lit beacon I2C, filtre niveaux 0-4, **stoppe le robot** | **OUI — problème** |
| **ServoObjectsTimer** | 50ms | `common/action/ServoObjectsSystem.cpp/hpp` | Interpole position servo, envoie commande PWM | Oui (contrôle moteur servo) |
| **LedBarTimer** | variable (µs) | `common/action/LedBar.cpp/hpp` | Anime LEDs (alternate, K2000, blink) | Non (affichage) |
| **TestTimer** | 100-500ms | `bot/opos6ul/tests/O_ActionManagerTimerTest` | Log messages (test seulement) | Non |

### Mécanisme

```
ActionManagerTimer (◆ THREAD, event-driven)
  │  gère une liste de ITimerPosixListener
  │  chaque timer émet un SIGALRM à sa période
  │
  ├── SensorsTimer ──→ onTimer() toutes les N ms
  ├── ServoObjectsTimer ──→ onTimer() toutes les 50ms
  └── LedBarTimer ──→ onTimer() toutes les N µs
```

Enregistrement via : `actions().addTimer(ITimerPosixListener* timer)`

## Refactoring prévu : détection d'obstacles (SensorsTimer)

### Problème actuel (ancien PMX)

Le `SensorsTimer::onTimer()` mélange **3 responsabilités** dans un seul callback :

```
SensorsTimer::onTimer()          ← callback timer périodique
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
SensorsTimer (timer async)      : "obstacle level 4 → STOP !"
     │  path_InterruptTrajectory()
     │
DecisionMakerIA                 : motion_Goto() retourne TRAJ_INTERRUPTED
                                  mais ne sait pas POURQUOI ni OÙ est l'obstacle
```

**État dispersé dans 4 classes** :
```
SensorsDriver.vadv_                    ← positions brutes adversaire (mutex)
Sensors.adv_is_detected_front_right_   ← flags détection
SensorsTimer.nb_ensurefront4           ← compteurs debounce
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

| Situation | Avant (devant/derrière + flags) | Avec isOnPath |
|---|---|---|
| Rotation sur place, adversaire devant | STOP (faux positif) → `temp_ignoreFront=true` | Pas sur le chemin → OK |
| Goto, adversaire sur le côté | STOP si dans le seuil frontal | Pas sur le chemin → OK |
| Goto, adversaire pile sur la route | STOP | Sur le chemin → STOP (vrai positif) |
| Pathfinding chaîné | Flags oubliés entre segments | Chaque segment vérifié indépendamment |
| Goto composite (carte ST : rotation+translation) | Ne sait pas quelle phase → mauvaise décision | Vérifie le segment final, pas la phase |

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

| Avant | Après |
|---|---|
| SensorsTimer lit ET filtre ET stoppe le robot | SensorsTimer lit, publie les coordonnées adversaire |
| L'IA ne sait pas pourquoi le robot s'est arrêté | L'IA reçoit l'événement + coordonnées, décide elle-même |
| Reprise impossible si adversaire "devant" | Reprise autorisée si nouvelle trajectoire est libre |
| Debounce dans le timer (état mélangé) | Debounce dans ObstacleDetector (logique pure, testable) |
| Patch `+/-50mm` hardcodé dans le filtre | Calibration dans config.txt |
| Impossible à tester sans hardware | ObstacleDetector = fonction pure, testable en SIMU |

### Plan de migration

1. **Phase 1** : migrer tel quel (SensorsTimer avec zones devant/derrière)
2. **Phase 2** : extraire ObstacleDetector (logique pure) depuis Sensors::filtre_levelInFront/Back
3. **Phase 3** : remplacer les appels directs à l'asserv par des ObstacleEvent
4. **Phase 4** : intégrer la réception d'ObstacleEvent dans DecisionMakerIA
5. **Phase 5** : ajouter la vérification trajectoire (isOnPath) pour la logique de reprise

## Ordre de migration — classe par classe avec tests unitaires

Chaque classe est migrée avec son test. Pour les drivers, on migre toujours le couple
SIMU + ARM + test unitaire via l'interface abstraite.

### Étape 1 — Briques de base (aucune dépendance) ✅

| Classe | Dossier cible | Test | Statut |
|---|---|---|---|
| Thread / Mutex | `common/thread/` | ThreadTest | ✅ |
| Chronometer | `common/utils/` | ChronometerTest | ✅ |
| Logger / LoggerFactory | `common/log/` | LoggerTest | ✅ |

### Étape 2 — Framework de test ✅

| Classe | Dossier cible | Statut |
|---|---|---|
| UnitTest | `test/suite/` | ✅ |
| UnitTestSuite | `test/suite/` | ✅ |
| UnitTestAppender | `test/suite/` | ✅ (+ colorisation ANSI) |

### Étape 3 — Interfaces abstraites drivers ✅

| Classe | Dossier cible | Statut |
|---|---|---|
| AAsservDriver | `common/interface/` | ✅ |
| ASensorsDriver | `common/interface/` | ✅ |
| ALedDriver | `common/interface/` | ✅ |
| AButtonDriver | `common/interface/` | ✅ |
| ASwitchDriver | `common/interface/` | ✅ |
| AServoDriver | `common/interface/` | ✅ |
| ALcdShieldDriver | `common/interface/` | ✅ |
| ASoundDriver | `common/interface/` | ✅ |
| AColorDriver | `common/interface/` | ✅ |
| AServoUsingMotorDriver | `common/interface/` | ✅ |
| AActionDriver | `common/interface/` | ✅ |
| ARobotPositionShared | `common/interface/` | ✅ |

### Étape 4 — Drivers (SIMU + ARM + test) ✅

| Driver | SIMU | ARM | Test | Statut |
|---|---|---|---|---|
| LedDriver | ✅ | ✅ | LedDriverTest | ✅ |
| ButtonDriver | ✅ | ✅ | ButtonDriverManualTest | ✅ |
| SwitchDriver | ✅ | ✅ | SwitchDriverTest | ✅ |
| SoundDriver | ✅ | ✅ | — | ✅ |
| LcdShieldDriver | ✅ | ✅ | LcdShieldDriverManualTest | ✅ |
| ServoDriver | ✅ | ✅ | ServoDriverManualTest | ✅ |
| ServoUsingMotorDriver | ✅ | ✅ | — | ✅ |
| SensorsDriver | ✅ | ✅ | SensorDriverManualTest | ✅ |
| RobotPositionShared | ✅ | ✅ | — | ✅ |
| ColorDriver | ✅ | ✅ | ColorDriverManualTest | ✅ |
| AsservDriver | ✅ | ✅ | AsservDriverManualTest | ✅ |
| ActionDriver | ✅ | ✅ | — | ✅ |

### Étape 5 — Actions et timers ✅

| Classe | Dossier cible | Test | Statut |
|---|---|---|---|
| IAction, ITimerListener, ITimerPosixListener | `common/timer/` | — | ✅ |
| ActionManagerTimer | `common/timer/` | ActionManagerTimerTest (13 UT) + bench | ✅ |
| AActionsElement | `common/action/` | — | ✅ |
| Actions | `common/action/` | — | ✅ |
| LedBar | `common/action/` | O_LedBarTest | ✅ |
| ButtonBar | `common/action/` | O_ButtonBarTest | ✅ |
| Sensors | `common/action/` | O_SensorsTest | ✅ (test non migré) |
| ServoObjectsSystem | `common/action/` | O_ServoObjectsTest | ✅ (test non migré) |
| ServoUsingMotor | `common/action/` | — | ✅ |
| Tirette | `common/action/` | O_TiretteTest | ✅ |
| LcdShield | `common/action/` | O_LcdBoardTest | ✅ |
| SoundBar | `common/action/` | — | ✅ |

### Étape 6 — Asserv, Robot et mouvement

| Classe | Dossier cible | Test | Statut |
|---|---|---|---|
| Asserv | `common/asserv/` | — | ✅ |
| AsservEsialR (+ sous-modules) | `common/asserv.esial/` | O_AsservEsialTest | ✅ (test non migré) |
| Automate / AAutomateState | `common/state/` | — | ✅ |
| Arguments / ConsoleManager | `common/utils/` | — | ✅ |
| Robot | `common/` | — | ✅ |
| MotorControl / EncoderControl / MovingBase | — | — | ⬜ Non utilisés, non migrés |
| IA / IAbyPath / IAbyZone | `common/ia/` | O_IAbyPathTest | ⬜ À migrer |

### Étape 7 — Bot OPOS6UL

| Classe | Dossier cible | Test | Statut |
|---|---|---|---|
| OPOS6UL_RobotExtended | `bot-opos6ul/` | — | ✅ |
| OPOS6UL_ActionsExtended | `bot-opos6ul/` | — | ✅ |
| OPOS6UL_AsservExtended | `bot-opos6ul/` | O_AsservLineRotateTest | ✅ (test non migré) |
| OPOS6UL_IAExtended | `bot-opos6ul/` | — | ✅ |
| OPOS6UL_SvgWriterExtended | `bot-opos6ul/` | — | ✅ |
| O_State_Init / DecisionMaker / WaitEnd | `bot-opos6ul/` | — | ✅ |
| O_LedBarTest | `bot-opos6ul/` | — | ✅ |
| O_TiretteTest | `bot-opos6ul/` | — | ✅ |
| O_ButtonBarTest | `bot-opos6ul/` | — | ✅ |
| O_LcdBoardTest | `bot-opos6ul/` | — | ✅ |
| O_ActionManagerTimerTest | `bot-opos6ul/` | — | ✅ |
| O_IAByPathTest | `bot-opos6ul/` | — | ✅ |
| O_ServoStepTest | `bot-opos6ul/` | — | ✅ |
| O_ServoObjectsTest | `bot-opos6ul/` | — | ✅ |
| O_SensorsTest | `bot-opos6ul/` | — | ✅ |
| Tests fonctionnels asserv (6) | `bot-opos6ul/` | — | ⬜ À migrer |

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

| Target | Type | CI ? | Quand l'utiliser |
|---|---|---|---|
| `common-test` | Unit test | Oui, doit passer | À chaque build |
| `driver-test` | Unit test | Oui (SIMU), sur carte (ARM) | À chaque build |
| `bench` | Benchmark | Non | Quand on optimise |
| `manual-test` | Test visuel/physique | Non | Sur la carte, en atelier |

### Analyse qualité des tests PMX existants

| Test ancien | Assertions réelles ? | Migration vers |
|---|---|---|
| ActionManagerTimerTest | Oui — vérifie compteurs | `common/` (unit test) |
| LoggerTest | Oui — vérifie `expectedError()` | `common/` (unit test) |
| ThreadTest | Partiel — vérifie calculs | `common/` (+ renforcer assertions) |
| ChronometerTest | Non — log des timings | `bench/` + nouveau test dans `common/` avec assertions |
| TimerFactoryTest | Non — observe les logs | `bench/` + nouveau test dans `common/` avec assertions |
| ReadWriteTest | Non — benchmark I/O | `bench/` |
| LedDriverTest (ARM) | Non — `assert(true)` | `manual/` |
| ServoDriverTest (ARM) | Non — `assert(true)` | `manual/` |
| AsservDriverTest | Non — `assert(true)` | `driver/` (+ assertions réelles) |
| SensorDriverTest | Non — `assert(true)` | `driver/` (+ assertions réelles) |

## Déploiement ARM (OPOS6UL)

### Principe

Le binaire est compilé en cross-compilation (preset `arm-release`), strippé pour réduire
sa taille, puis envoyé sur la carte OPOS6UL via `scp`. L'authentification utilise `sshpass`.

### Prérequis

```bash
sudo apt install sshpass
```

### Adresses IP de la carte

| Interface | IP | Usage |
|---|---|---|
| eth0 (câble direct) | `192.168.2.105` | En atelier, connexion fiable |
| WiFi 5GHz AP | `192.168.3.103` | Sur le terrain, sans câble |
| WiFi PMX | `192.168.2.107` | Réseau local PMX |
| WiFi maison | `192.168.0.205` | Dev à domicile |

### Utilisation

#### Via VSCode (recommandé)

Raccourci : **Ctrl+Shift+T** → menu des tasks :

| Task | Description |
|---|---|
| `ARM: Deploy` | Build ARM + strip + scp vers `/root/pmx/` |
| `ARM: Deploy + Run` | Idem + exécute sur la carte via SSH |
| `ARM: Connect SSH` | Terminal SSH interactif dans `/root/pmx/` |
| `ARM: Recup SVG` | Rapatrie les fichiers SVG depuis la carte |

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

| Paramètre | Valeur | Raison |
|---|---|---|
| `EXTRACT_ALL` | YES | Documente aussi les méthodes non commentées |
| `EXTRACT_PRIVATE` | YES | Utile pour comprendre l'architecture interne |
| `HAVE_DOT` | YES | Diagrammes de classes et d'appels (graphviz) |
| `CLASS_GRAPH` | YES | Hiérarchie d'héritage |
| `CALL_GRAPH` | YES | Graphe d'appels de fonctions |
| `EXCLUDE` | `json.hpp` | nlohmann/json (24000 lignes, pas utile) |
| `GENERATE_LATEX` | NO | HTML uniquement |

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

Chaque paquet UDP contient un objet JSON avec l'ID du robot, un timestamp, le temps écoulé et les données du logger :

```json
{
  "OPOS6UL": {
    "timestamp": 1774733396.268,
    "elapsedtime_ms": 10009.463,
    "LedBar": { "pos": 0, "color": 6 }
  }
}
```

### Configuration réseau

| Destination | IP | Port |
|---|---|---|
| RPI (récepteur) | `192.168.3.101` | 9870 |

La même IP est utilisée en SIMU et en ARM. La configuration est dans `src/bot-opos6ul/LoggerInitialize.cpp`.

### Test local en simulation

```bash
cd robot/build-simu-release
socat -u UDP-RECV:9870,reuseaddr OPEN:./telemetry.json,creat,trunc &
./bot-opos6ul t /n 1
cat telemetry.json
```

### Loggers branchés sur la télémétrie

Pour envoyer de la télémétrie depuis un logger, le configurer sur l'appender `"net"` dans `LoggerInitialize.cpp` :

```cpp
add(logs::Level::INFO, "MonLogger", "net");
```

Puis dans le code, utiliser `logger().telemetry(json)` pour envoyer des données JSON.

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

  | Thread | Priorité avant | Priorité après | Mode | Rôle |
  |---|---|---|---|---|
  | **AsservDriver** (driver-arm) | 3 | **80** | ARM | Comm série Teensy |
  | **AsservEsialR** | 2 | **75** | SIMU | Boucle PID interne |
  | **ActionManagerTimer** | 2 | **60** | ARM+SIMU | Actions/servos |
  | **Main** | 50 | **50** | ARM+SIMU | Init/orchestration |
  | **DecisionMakerIA** | 3 | **40** | ARM+SIMU | Stratégie match |
  | **LoggerFactory** | 0 | **0** | ARM+SIMU | Logs best-effort |

- ✅ **3. UART `ASYNC_LOW_LATENCY` — Réduire la latence série**
  - Activer le flag `ASYNC_LOW_LATENCY` via `ioctl(TIOCSSERIAL)` après `open()` du port série
  - Réduit la latence de réveil du `read()` de ~10ms à <1ms
  - Critique pour la communication haute fréquence avec la Teensy

- ⬜ **4. `clock_nanosleep` absolu — Boucles sans dérive**
  - Remplacer les `sleep_for()` / `usleep()` par `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ...)`
  - Cible un instant absolu : si le traitement prend 2ms sur une période de 5ms, le sleep ne dure que 3ms
  - Insensible aux changements d'horloge (NTP)

- ⬜ **5. Priorités IRQ via `chrt` — Protéger serial/I2C du WiFi**
  - Script init qui monte la priorité des threads IRQ UART et I2C au-dessus du WiFi :
    - IRQ UART : `chrt -f -p 90`
    - IRQ I2C : `chrt -f -p 85`
    - WiFi : reste à la priorité par défaut (50)
  - Empêche un transfert WiFi de bloquer les threads critiques
