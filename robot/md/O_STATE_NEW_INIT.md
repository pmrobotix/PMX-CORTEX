# O_State_NewInit — Refactor du menu d'init multi-sources

Document de specification pour remplacer [src/bot-opos6ul/O_State_Init.cpp](../src/bot-opos6ul/O_State_Init.cpp) par une version qui supporte plusieurs interfaces de configuration en parallele (LCD shield 2x16, LCD tactile balise, et eventuelles futures sources) avec une source de verite unique cote Robot.

Contexte et decisions architecturales discutees avant redaction : voir l'historique du refactor. Les choix retenus sont resumes ici et font foi.

## 1. Objectifs

- **Plusieurs sources d'input/output simultanees** pour configurer le robot avant match : LCD shield 2x16 + boutons (existant), LCD tactile balise via I2C 0x2D (existant cote Teensy, voir [ARCHITECTURE_BEACON.md](../../teensy/IO_t41_ToF_DetectionBeacon/ARCHITECTURE_BEACON.md)), et possibilite d'ajouter un 3e systeme plus tard sans reecriture.
- **Degradation gracieuse** : si une des deux interfaces tombe ou est debranchee (en compet ou en preparation), l'autre continue de fonctionner.
- **Source de verite unique** : `Robot` detient l'etat de configuration. Aucune source n'a sa propre copie "faisant foi". Les sources sont de simples plugins input/output.
- **Anti-misclick couleur** : couleur de match confirmee par hold 2s (erreur frequente sur le shield actuel).
- **Couleur verrouillee apres setPos** : la couleur conditionne la position physique du robot sur la table, donc on ne peut pas la changer apres que `setPos` a positionne le robot.
- **Autres parametres editables jusqu'a la tirette** : strategie, diametre adversaire, luminosite LED, tests mecaniques.
- **Support `/k` skipSetup** inchange : court-circuite le menu, params venant du parseur CLI ou defauts.

## 2. Vue d'ensemble

```
             +------------------------------+
             | Robot (source de verite)     |
             |                              |
             | phase_                       |
             | myColor_ proposedColor_      |
             | strategy_ configVRR_         |
             | advDiameter_ ledLuminosity_  |
             | testMode_                    |
             | validateReq_ resetReq_       |
             |                              |
             | API publique :               |
             |   setXxx()  (phase-aware)    |
             |   getXxx()                   |
             +------^-------------------+---+
                    |                   |
             (2) robot.setXxx()  (3) robot.getXxx()
                    |                   |
             +------+-------------------+---+
             | MenuController               |
             |                              |
             | void tick() {                |
             |   for s in sources:          |
             |     s->pollInputs(robot)     | --- appel (1)
             |   for s in sources:          |
             |     s->refreshDisplay(robot) | --- appel (4)
             | }                            |
             +----+--------------------+----+
                  |                    |
            (1) pollInputs       (4) refreshDisplay
                  |                    |
       +----------+--------+-----------+---------+
       v                   v                     v
  +----------------------+           +----------------------+
  | MenuShieldLCD        |           | MenuBeaconLCDTouch   |
  |                      |           |                      |
  | pollInputs()         |           | pollInputs()         |
  |  lit boutons         |           |  lit I2C Settings    |
  |  + robot.setXxx      |           |  + robot.setXxx      |
  |                      |           |                      |
  | refreshDisplay()     |           | refreshDisplay()     |
  |  robot.getXxx()      |           |  robot.getXxx()      |
  |  + ecrit LCD2x16     |           |  + push I2C          |
  +----------+-----------+           +----------+-----------+
             |                                  |
             v                                  v
  +----------------------+           +----------------------+
  | HW: LCD 2x16         |           | HW: Teensy via I2C   |
  |     + boutons        |           |     LCD tactile      |
  +----------------------+           +----------------------+
```

Les 4 fleches numerotees sur un tick :

1. **Controller -> source** : `tick()` appelle `source->pollInputs(robot)` sur chaque source a tour de role. C'est l'invocation du plugin.
2. **Source -> Robot (ecriture)** : a l'interieur de `pollInputs`, la source lit ses propres inputs hardware (boutons, I2C, ...) et appelle `robot.setXxx(...)` pour chaque changement detecte. **Seule ecriture vers Robot.**
3. **Robot -> Source (lecture)** : ensuite, le controller appelle `source->refreshDisplay(robot)`. La source fait `robot.getXxx()` pour obtenir l'etat courant.
4. **Source -> HW (affichage)** : toujours dans `refreshDisplay`, la source pousse cet etat vers son hardware (ecriture LCD2x16, write I2C Settings, ...). **Aucun setter Robot appele dans cette phase.**

Les setters/getters appartiennent a `Robot`. Les methodes `pollInputs`/`refreshDisplay` appartiennent a `IMenuSource` et a chacune de ses implementations. Le controller ne fait aucun acces direct a Robot : c'est un simple orchestrateur qui appelle les 2 methodes dans le bon ordre sur toutes les sources.

Regle d'or : **une source ne parle jamais a une autre source**. Elles passent toujours par Robot. Si le shield veut refleter un changement fait via le touch, il relit Robot au prochain tick (via son `refreshDisplay`).

## 3. Nouveautes cote `Robot`

### 3.1 Enum de phase

```cpp
// Robot.hpp
enum MatchPhase {
    PHASE_CONFIG    = 0,  // Menu ouvert, tout editable
    PHASE_COMMITTED = 1,  // Couleur commit + setPos fait, strat/diam/test editables
    PHASE_PRIMED    = 2,  // Tirette inseree, strat/diam/test toujours editables
    PHASE_MATCH     = 3,  // Tirette retiree, match en cours
    PHASE_END       = 4,  // Fin match
};
```

**Pourquoi 2 phases COMMITTED et PRIMED malgre des permissions identiques** : la tirette est detectee par edge. Sans attendre explicitement l'insertion (COMMITTED -> PRIMED), un etat "tirette jamais inseree" serait interprete comme "tirette retiree" et ferait partir le match directement. PRIMED garantit qu'on a bien vu le rising edge avant d'accepter le falling edge.

### 3.2 Champs ajoutes

```cpp
protected:
    MatchPhase  phase_         = PHASE_CONFIG;
    RobotColor  proposedColor_ = PMXNOCOLOR;   // Toggle libre en CONFIG, non utilise par setPos
    // myColor_ existe deja, sert de couleur commit (utilisee par setPos)

    uint8_t     advDiameter_   = 40;   // cm
    uint8_t     ledLuminosity_ = 5;    // 0..100, defaut aligne avec Settings Teensy
    uint8_t     testMode_      = 0;    // 0=aucun, 1..5
    std::atomic<bool> validateReq_{false};
    std::atomic<bool> resetReq_{false};
    std::atomic<bool> testModeReq_{false};
```

### 3.3 Setters phase-aware

Les setters centralisent les regles de verrouillage. Une source qui tente une modif interdite recoit `false` en retour, libre a elle de feedback visuellement.

```cpp
// PHASE_CONFIG uniquement
void proposeColor(RobotColor c);
bool commitColor();        // proposedColor_ -> myColor_, transition CONFIG -> COMMITTED
void uncommitColor();      // COMMITTED -> CONFIG, liberation couleur

// PHASE_CONFIG + PHASE_COMMITTED + PHASE_PRIMED (jusqu'avant MATCH)
bool setStrategy(const std::string& s);
bool setConfigVRR(const std::string& s);
bool setAdvDiameter(uint8_t d);
bool setLedLuminosity(uint8_t l);
bool triggerTestMode(uint8_t t);   // positionne testMode_ + testModeReq_

// Drapeaux d'intention
void requestValidate();    // non utilise dans cette version (commit couleur = validation)
bool validateRequested() const;
void clearValidate();

void requestReset();       // appele par une source si l'operateur veut revenir en CONFIG
bool resetRequested() const;
void clearReset();

// Gestion de phase (appelee uniquement par O_State_NewInit)
MatchPhase phase() const;
void setPhase(MatchPhase p);
```

## 4. Interface `IMenuSource`

Nouveau fichier `src/common/IMenuSource.hpp`.

```cpp
class IMenuSource {
public:
    virtual ~IMenuSource() = default;

    // Lit les inputs (clics, touch, I2C) et appelle les setters Robot.
    // Appele a chaque tick (~10 Hz) du MenuController.
    virtual void pollInputs(Robot& robot) = 0;

    // Rafraichit l'affichage (LCD, ecran tactile, I2C push) a partir de
    // l'etat courant de Robot. Appele a chaque tick, apres pollInputs.
    virtual void refreshDisplay(const Robot& robot) = 0;

    // Indique si la source est encore vivante (hardware repond).
    // Le controller peut decider de ne pas l'appeler si false.
    virtual bool isAlive() const = 0;

    // Nom pour les logs.
    virtual const char* name() const = 0;
};
```

### 4.1 `MenuController`

Nouveau fichier `src/common/MenuController.hpp/cpp`. Coordonne les sources.

```cpp
class MenuController {
    std::vector<std::unique_ptr<IMenuSource>> sources_;
    Robot& robot_;
public:
    explicit MenuController(Robot& r);
    void add(std::unique_ptr<IMenuSource> s);

    // Appele par O_State_NewInit a 100 Hz.
    // Ordre critique : pollInputs d'abord sur toutes les sources, PUIS refreshDisplay.
    // Cet ordre permet a une modif touch d'etre adoptee par Robot avant que
    // le shield (ou une autre source) ne pousse son etat vers son support.
    void tick();

    bool anyAlive() const;
    size_t aliveCount() const;
};
```

## 5. `MenuShieldLCD` — shield 2x16 + boutons

### 5.1 Layout LCD 2 lignes, toutes les infos en permanence

Les 2 lignes affichent en continu **tout l'etat de configuration**. Un curseur blink HD44780 indique le champ en edition (pour les phases COMMITTED/PRIMED).

```
Ligne 0:  Y*S2 VRR  D:40
Ligne 1:  L:50 T:- B:OK
```

**Contenu ligne 0** (16 chars max) :

| Position | Champ | Format | Largeur |
|---|---|---|---|
| 0-1 | Couleur + commit | `Y*`/`B*`/`Y?`/`B?`/`__` | 2 |
| 2-3 | Strategie courte | `S1`/`S2`/`S3` | 2 |
| 4 | espace | ` ` | 1 |
| 5-7 | Config VRR | `VRR`/`RVR`/`RRV` | 3 |
| 8-9 | padding | `  ` | 2 |
| 10-13 | Diametre adv | `D:40` | 4 |
| 14-15 | padding | `  ` | 2 |

**Contenu ligne 1** (16 chars max) :

| Position | Champ | Format | Largeur |
|---|---|---|---|
| 0-3 | Luminosite LED | `L:50` | 4 |
| 4 | espace | ` ` | 1 |
| 5-7 | Dernier test lance | `T:-`/`T:1`..`T:5` | 3 |
| 8 | espace | ` ` | 1 |
| 9-12 | Beacon alive | `B:OK`/`B:--` | 4 |
| 13-15 | padding | `   ` | 3 |

**Cursor blink** : en phase COMMITTED/PRIMED, le curseur HD44780 clignote sur le champ en cours d'edition (couleur, strat, VRR, diam, led, test). En phase CONFIG, le curseur est positionne sur `Y*`/`B*` (champ couleur).

**Correspondance courte <-> longue**. Mapping a conserver dans une fonction utilitaire du source :

| Long (Robot) | Court (LCD) |
|---|---|
| `tabletest` | `S1` |
| `strat2` | `S2` |
| `strat3` | `S3` |
| `VRR` | `VRR` |
| `RVR` | `RVR` |
| `RRV` | `RRV` |

### 5.2 Interaction boutons

**Principe** : un "curseur logique" qui designe le champ en edition. UP/DOWN deplace le curseur, LEFT/RIGHT modifie la valeur du champ selectionne.

Ordre des champs navigables via UP/DOWN (cycle) :

```
COULEUR -> STRAT -> VRR -> DIAM -> LED -> TEST -> (retour COULEUR)
```

#### PHASE_CONFIG

Le curseur est bloque sur COULEUR. Les champs non-couleur ne sont pas editables via shield en CONFIG (ils sont visibles sur la ligne 0/1 et peuvent etre modifies via la balise touch si presente).

| Bouton | Clic court | Hold 2s | Hold 5s |
|---|---|---|---|
| LEFT | `proposeColor(YELLOW)` | `commitColor()` (si proposed == YELLOW) | - |
| RIGHT | `proposeColor(BLUE)` | `commitColor()` (si proposed == BLUE) | - |
| UP | (inutilise) | - | - |
| DOWN | (inutilise) | - | - |
| ENTER | (inutilise) | - | - |
| BACK | (inutilise) | - | `exit(0)` |

**Feedback hold 2s** : pendant le hold, la ligne 1 affiche `HOLD 2.0 -> 0.0` en countdown. Au commit reussi, flash `COMMITTED!` 500 ms puis retour affichage standard.

#### PHASE_COMMITTED / PHASE_PRIMED

Le curseur peut naviguer sur tous les champs sauf COULEUR (qui reste visible mais non editable).

| Bouton | Clic court | Hold 5s |
|---|---|---|
| UP | curseur champ precedent | - |
| DOWN | curseur champ suivant | - |
| LEFT | valeur precedente du champ selectionne | - |
| RIGHT | valeur suivante du champ selectionne | - |
| ENTER | si curseur sur TEST, `triggerTestMode(selected)` | - |
| BACK | `requestReset()` (retour CONFIG) | `exit(0)` |

**Wrapping des valeurs** : LEFT sur `S1` -> `S3`, RIGHT sur `D:250` -> `D:5`, etc.

**Tentative de bouger le curseur sur COULEUR en COMMITTED** : le curseur passe par dessus sans s'arreter, affichage d'un flash `COLOR LOCKED` 500 ms sur la ligne 1.

### 5.3 Detection de presence shield

`MenuShieldLCD::isAlive()` = resultat du dernier acces I2C au shield. Si un `writeReg` echoue, on bascule alive=false, et le controller arrete d'appeler `refreshDisplay` sur cette source jusqu'au prochain check.

Au boot de `O_State_NewInit`, un probe explicite est fait via `LcdShield::isConnected()` (a ajouter si pas present).

### 5.4 Fichiers a creer

- `src/common/MenuShieldLCD.hpp`
- `src/common/MenuShieldLCD.cpp`

Dependances : `LcdShield`, `ButtonBar`, `Robot`.

## 6. `MenuBeaconLCDTouch` — LCD tactile balise

Premiere etape : lecture uniquement (OPOS6UL recupere ce que l'operateur tape sur le LCD tactile). Modifications cote Teensy listees en 6.3.

### 6.1 Comportement

**`pollInputs`** :
1. Lit le bloc Settings (reg 5-8) + `seq_touch` de la balise via `BeaconSensors::readSettings()`.
2. Si `seq_touch != last_seq_seen_` : un clic a eu lieu depuis la derniere lecture.
3. Pour chaque champ change par rapport a `shadow_`, appelle le setter Robot correspondant (`setMyColor`/`setStrategy`/`setAdvDiameter`/`triggerTestMode`).
4. Met a jour `last_seq_seen_ = seq_touch` et `shadow_ = current`.
5. Si regression de `seq_touch` (< `last_seq_seen_`) : reboot Teensy detecte -> reset `last_seq_seen_ = 0`, ne rien adopter ce tick, le prochain refreshDisplay re-pushera Robot.

**`refreshDisplay`** :
1. Push Robot vers Settings Teensy via `BeaconSensors::writeXxx()` :
   - `matchColor` (0/1 depuis `robot.getMyColor()`)
   - `strategy` (1..3 depuis mapping)
   - `advDiameter`
   - `ledLuminosity`
   - `matchState` (= `robot.phase() >= PHASE_MATCH ? 1 : 0`, evolue plus tard)
2. Met a jour `shadow_` avec les valeurs poussees (pour ne pas re-declencher un "delta" au prochain pollInputs).
3. **Ne touche jamais `seq_touch`** : c'est la Teensy qui l'incremente.

**`isAlive`** : derniere operation I2C OK + `Registers.seq` (cycle ToF) doit avoir avance entre deux polls, sinon Teensy consideree morte.

### 6.2 Fichiers OPOS6UL a creer/modifier

- `src/common/MenuBeaconLCDTouch.hpp` (nouveau)
- `src/common/MenuBeaconLCDTouch.cpp` (nouveau)
- `src/driver-arm/BeaconSensors.hpp/cpp` : ajouter
  - `bool readSettings(Settings& out)` (lit les 10 bytes du bloc Settings d'un coup)
  - `bool writeMatchColor(uint8_t)`
  - `bool writeStrategy(uint8_t)`
  - `bool writeAdvDiameter(uint8_t)`
  - `bool writeMatchState(uint8_t)`
  - `bool writeMatchPoints(uint8_t)`
  - `bool writeNumOfBots(int8_t)`
  - (le `writeLedLuminosity` existant est OK)

### 6.3 Modifications cote Teensy (2e etape)

A faire dans un 2e temps, apres avoir valide le framework OPOS6UL. Ces modifications conditionnent la capacite du touch a piloter la config :

- **[teensy/IO_t41_ToF_DetectionBeacon/src/TofSensors.h](../../teensy/IO_t41_ToF_DetectionBeacon/src/TofSensors.h)** : ajouter dans la struct `Settings` un champ `uint8_t seq_touch = 0` au Reg 9 (apres `advDiameter`). Mettre a jour le `static_assert(sizeof(Settings) == 10)`.
- **[teensy/IO_t41_ToF_DetectionBeacon/src/LCDScreen.cpp](../../teensy/IO_t41_ToF_DetectionBeacon/src/LCDScreen.cpp)** : dans CHAQUE callback LVGL qui modifie un champ de Settings (matchColor, strategy, testMode, advDiameter, ledLuminosity), ajouter `settings.seq_touch++` apres la modif. C'est la seule facon pour l'OPOS6UL de distinguer "valeur qu'il vient d'ecrire et relit" de "valeur que l'operateur a cliquee".
- **Affichage Robot-originated des champs** : le LCD tactile doit afficher les valeurs courantes des Settings en continu (c'est deja le cas via les labels LVGL rafraichis a 5 Hz dans `screen_loop()` selon [ARCHITECTURE_BEACON.md#L378](../../teensy/IO_t41_ToF_DetectionBeacon/ARCHITECTURE_BEACON.md)). Verifier que TOUS les champs modifies par OPOS6UL sont bien reflechis sur l'ecran LVGL : matchColor, strategy, advDiameter, ledLuminosity. Sinon ajouter les bindings manquants.
- **Mise a jour de la doc** : modifier [ARCHITECTURE_BEACON.md](../../teensy/IO_t41_ToF_DetectionBeacon/ARCHITECTURE_BEACON.md) section "Struct Settings" pour documenter `seq_touch` + `static_assert` + role dans la reconciliation.
- **Reset au boot Teensy** : `seq_touch` demarre a 0 apres reset -> c'est exactement le signal que l'OPOS6UL utilise pour detecter le reboot Teensy (voir 6.1 point 5). Ne pas persister `seq_touch` en EEPROM.
- **BeaconSensors.hpp cote OPOS6UL** : mettre a jour `SETTINGS_SIZE_BeaconSensors = 10` et `DATA_BeaconSensors = 10` pour garder l'alignement avec la nouvelle taille Settings.
- **Revoir tous les offsets** de `getData()` dans [BeaconSensors.cpp:126](../src/driver-arm/BeaconSensors.cpp#L126) : chaque `DATA_BeaconSensors + N` doit etre decale de +1 byte (Settings passe de 9 a 10).

## 7. `O_State_NewInit::execute()` — pseudo-code

Nouveau fichier `src/bot-opos6ul/O_State_NewInit.hpp/cpp`, remplace `O_State_Init`. Le nom de registration dans l'automate reste probablement `Init` pour ne pas casser le cablage, a decider.

```cpp
IAutomateState* O_State_NewInit::execute(Robot&)
{
    logger().info() << "O_State_NewInit executing..." << logs::end;
    auto& robot = OPOS6UL_RobotExtended::instance();
    robot.actions().start();

    robot.actions().lcd2x16().setBacklightOn();
    robot.actions().lcd2x16().clear();

    //========================================================
    // /k SKIP SETUP : params deja dans Robot via CLI
    //========================================================
    if (robot.skipSetup()) {
        logger().info() << "SKIP SETUP (/k)" << logs::end;
        if (robot.getMyColor() == PMXNOCOLOR) {
            logger().error() << "No color (CLI or default), EXIT" << logs::end;
            exit(1);
        }
        robot.setPhase(PHASE_COMMITTED);
        setPos();
        robot.setPhase(PHASE_PRIMED);
        robot.waitForInit(true);
        robot.setPhase(PHASE_MATCH);
    }
    //========================================================
    // MENU NORMAL (multi-sources)
    //========================================================
    else {
        // 1. Construction controller + sources
        MenuController ctrl(robot);
        if (robot.actions().lcd2x16().isConnected()) {
            ctrl.add(std::make_unique<MenuShieldLCD>(
                robot.actions().lcd2x16(), robot.actions().buttonBar()));
        }
        if (robot.actions().sensors().beaconConnected()) {
            ctrl.add(std::make_unique<MenuBeaconLCDTouch>(
                robot.actions().sensors().beacon()));
        }
        if (!ctrl.anyAlive()) {
            logger().error() << "No menu source alive, EXIT" << logs::end;
            exit(1);
        }

restart_menu:
        robot.setPhase(PHASE_CONFIG);
        robot.setMyColor(PMXNOCOLOR);
        robot.clearReset();
        robot.clearValidate();

        // 2. PHASE_CONFIG : attente commit couleur
        logger().info() << "PHASE_CONFIG, waiting color commit..." << logs::end;
        while (robot.phase() == PHASE_CONFIG) {
            ctrl.tick();
            if (!ctrl.anyAlive()) {
                logger().error() << "All sources lost, EXIT" << logs::end;
                exit(1);
            }
            utils::sleep_for_micros(10000);   // 100 Hz
        }
        // Sortie : commitColor() a fait passer la phase a COMMITTED

        // 3. setPos : le robot rejoint sa position initiale
        // Couleur maintenant LOCKED par phase_ >= PHASE_COMMITTED
        setPos();
        // Luminosite LED pour le match (override possible touch en CONFIG)
        robot.actions().sensors().writeLedLuminosity(50);

        // 4. PHASE_COMMITTED : attend l'insertion de la tirette
        //    Strat/diam/led/tests restent editables via sources
        logger().info() << "PHASE_COMMITTED, insert tirette..." << logs::end;
        while (!robot.actions().tirette().pressed()) {
            ctrl.tick();
            handleTestModeRequest(robot);   // voir section 7.1
            if (robot.resetRequested()) {
                robot.clearReset();
                robot.uncommitColor();
                robot.asserv().freeMotion();
                goto restart_menu;
            }
            if (!ctrl.anyAlive()) {
                logger().error() << "All sources lost, EXIT" << logs::end;
                exit(1);
            }
            utils::sleep_for_micros(10000);
        }

        // 5. PHASE_PRIMED : tirette inseree, attend retrait
        robot.setPhase(PHASE_PRIMED);
        logger().info() << "PHASE_PRIMED, waiting tirette release..." << logs::end;
        while (robot.actions().tirette().pressed()) {
            ctrl.tick();
            handleTestModeRequest(robot);   // toujours autorise en PRIMED
            if (robot.resetRequested()) {
                robot.clearReset();
                robot.uncommitColor();
                robot.asserv().freeMotion();
                goto restart_menu;
            }
            utils::sleep_for_micros(10000);
        }

        robot.setPhase(PHASE_MATCH);
    }

    //========================================================
    // Depart match (commun /k + menu) — inchange
    //========================================================
    robot.actions().ledBar().stop(true);
    robot.actions().ledBar().resetAll();
    robot.actions().lcd2x16().clear();
    robot.actions().lcd2x16().print("GO...");
    robot.displayPoints();

    logger().info() << "O_State_NewInit executed" << logs::end;
    return this->getState("WaitEndOfMatch");
}
```

### 7.1 Consommation du `testMode`

Quand une source (shield ENTER sur TEST, ou touch LVGL bouton T1..T5) pousse `triggerTestMode(n)`, Robot met `testModeReq_ = true`. O_State_NewInit detecte le flag dans `handleTestModeRequest()` et dispatch sur les routines numerotees :

```cpp
void handleTestModeRequest(Robot& robot) {
    if (!robot.testModeRequested()) return;
    uint8_t t = robot.testMode();
    robot.clearTestMode();

    switch (t) {
        case 1: runTestAx12Init(); break;
        case 2: runTestServosObjects(); break;
        case 3: runTestAspiration(); break;
        case 4: runTestLedBar(); break;
        case 5: runTestBeaconComm(); break;
        default: break;
    }
    // Convention : chaque routine se "self-cleans" = remet la meca en position
    // initiale avant de retourner, pour permettre des lancements repetes.
}
```

Les routines elles-memes sont a coder progressivement. Le framework est pret.

### 7.2 `setPos()`

La methode existante de `O_State_Init::setPos()` est copiee telle quelle dans `O_State_NewInit::setPos()`. Aucune modif fonctionnelle.

## 8. Interaction avec la memoire `myColor_` et strategy existantes

`Robot` a deja `myColor_`, `strategy_`, `configVRR_`. Ces champs restent et sont reutilises. La seule addition est `proposedColor_` + les phase-aware setters + les nouveaux champs `advDiameter_`/`ledLuminosity_`/`testMode_`.

**Retrocompatibilite** : le parseur CLI (Arguments class) continue d'ecrire dans `myColor_`/`strategy_`/`configVRR_` via les setters existants. En mode `/k`, ces setters doivent accepter l'ecriture meme si `phase_ == PHASE_CONFIG` (cas normal post-construction). A verifier qu'aucun ordonnancement ne casse : l'ordre est `construct Robot (phase=CONFIG) -> parseCLI -> execute O_State_NewInit`, donc CLI ecrit en phase CONFIG, OK.

## 9. Ordre d'implementation

Chaque etape doit etre **compilable + testable en simu et sur robot reel** avant de passer a la suivante.

### Etape 1 : phase enum + champs Robot

- Ajouter `MatchPhase`, `phase_`, `proposedColor_`, `advDiameter_`, `ledLuminosity_`, `testMode_`, drapeaux dans Robot
- Ajouter getters/setters phase-aware
- Aucun changement de comportement runtime (O_State_Init existant continue d'etre utilise). La phase est toujours `PHASE_CONFIG` car personne ne la change.
- **Test** : compile, simu OK, match OK via O_State_Init existant.

### Etape 2 : BeaconSensors read/write Settings

- Ajouter `readSettings(Settings&)` + writeXxx() par champ
- **Test** : un petit programme ad-hoc (ou un bouton de debug) qui lit + reecrit les Settings, verifier via logs et via affichage LCD tactile que les valeurs sont bien vues des deux cotes.

### Etape 3 : IMenuSource + MenuController

- Interface + controller + squelette vide
- Aucun source encore, juste la plomberie
- **Test** : compile.

### Etape 4 : MenuShieldLCD

- Implementation complete (status line 0+1, cursor, hold 2s commit, pages COMMITTED, reset, exit)
- **Test en isolation** : un petit binaire de test qui instancie `Robot + MenuController + MenuShieldLCD` et tourne la boucle, verification manuelle de tous les boutons et des affichages. Sans le reste du robot.

### Etape 5 : MenuBeaconLCDTouch cote OPOS6UL (lecture seule, sans seq_touch Teensy)

- Implementation qui lit les Settings actuels (sans seq) et pousse a chaque refreshDisplay
- Detection de delta via comparaison simple avec shadow
- **Test** : shield inactif, beacon presente. On modifie les settings sur le LCD tactile, on verifie que Robot les reflete (via logs ou ligne 0 du shield si on le laisse branche pour observer).
- **Limitation connue** : sans seq_touch, les cas de concurrence shield+touch ne sont pas parfaitement gerables. Acceptable pour cette etape car single-operator.

### Etape 6 : O_State_NewInit.cpp

- Nouveau fichier qui remplace l'usage de O_State_Init dans l'automate (ou cree en parallele avec un flag de selection)
- Branchement du controller, flow CONFIG/COMMITTED/PRIMED/MATCH, /k
- **Test** : parcours complet sur robot reel. Tester toutes les transitions, reset, exit.

### Etape 7 : modifs Teensy `seq_touch`

- Ajout `seq_touch` dans Settings + incrementation dans callbacks LVGL
- Flash Teensy
- **Test** : repeter le test etape 5 + ajouter des cas de concurrence shield+touch dans un intervalle < 100 ms, verifier que seq_touch evite les pertes.
- Decaler les offsets cote `BeaconSensors::getData()`.

### Etape 8 : `testMode` dispatch + routines

- `handleTestModeRequest()` + coder les routines 1..5 progressivement
- Chaque routine self-cleans (remet la meca en position de depart)
- **Test** : chaque routine lancable depuis shield (page TEST) et depuis touch (boutons T1..T5).

### Etape 9 : pages shield secondaires completes

- Curseur multi-champs, navigation UP/DOWN, edition LEFT/RIGHT, wrapping
- **Test** : parcours de tous les champs, verification des limites, cursor blink.

### Etape 10 : cleanup

- Supprimer `O_State_Init.cpp` si `O_State_NewInit` est stable
- Mettre a jour [ARCHITECTURE.md](ARCHITECTURE.md) pour pointer sur ce document
- Mettre a jour [HARDWARE_CONFIG.md](HARDWARE_CONFIG.md) si les flags d'activation doivent evoluer

## 10. Tests manuels de resilience

Scenarios a valider avant de considerer le refactor complet :

1. **Shield seul, beacon debranchee** : menu fonctionnel, commit couleur OK, tirette OK, match OK.
2. **Beacon seule, shield debranche** : toute la config via touch, commit couleur via touch (bouton OUI 2s), tirette OK, match OK.
3. **Les deux sources presentes, user utilise les deux** : changements shield visibles sur touch et inverse, pas de perte de config.
4. **Beacon reboot en plein menu (phase CONFIG)** : reset de `seq_touch` cote OPOS6UL, rien d'adopte, refreshDisplay re-pousse Robot -> Teensy reaffiche la config courante.
5. **Beacon reboot en plein match (phase MATCH)** : `matchState` + score re-pousses automatiquement, LCD tactile se met a jour.
6. **Reset shield (BACK click) en PHASE_COMMITTED** : retour CONFIG, robot en freeMotion, operateur peut re-committer.
7. **Exit shield (BACK hold 5s)** en toutes phases : `exit(0)` propre, logs OK.
8. **`/k` avec tous les params CLI** : court-circuit complet du menu.
9. **Test meca en PRIMED** : lance un test, verifie que la meca revient en position de depart, tirette retiree -> match OK.

## 11. Points ouverts / evolutions futures

- **Persistance EEPROM cote Teensy** (evoque dans ARCHITECTURE_BEACON.md section "Evolutions prevues") : rappeler les derniers choix operateur au reboot. Ne change rien cote OPOS6UL tant que `seq_touch` repart a 0 (ce qui est le comportement souhaite meme avec EEPROM, car on veut detecter le reboot comme un signal).
- **3e source potentielle** (web UI, telecommande, ...) : creer une classe qui herite de `IMenuSource`, l'ajouter au controller au boot si detectee. Zero impact sur le reste.
- **`matchNumber`** : non gere dans ce refactor (non pertinent pour la Coupe 2026 selon [ARCHITECTURE_BEACON.md](../../teensy/IO_t41_ToF_DetectionBeacon/ARCHITECTURE_BEACON.md)). A ajouter si besoin en reutilisant le meme pattern.
- **Synchronisation horloge entre OPOS6UL et Teensy** : pas necessaire pour la config. Si besoin pour les timestamps ToF, utiliser le champ `seq` des Registers ToF deja present.

## 12. Fichiers impactes (recapitulatif)

| Fichier | Action |
|---|---|
| [src/common/Robot.hpp](../src/common/Robot.hpp) | Ajouts : enum, champs, setters phase-aware |
| [src/common/Robot.cpp](../src/common/Robot.cpp) | Impl des setters |
| `src/common/IMenuSource.hpp` | Creation |
| `src/common/MenuController.hpp` | Creation |
| `src/common/MenuController.cpp` | Creation |
| `src/common/MenuShieldLCD.hpp` | Creation |
| `src/common/MenuShieldLCD.cpp` | Creation |
| `src/common/MenuBeaconLCDTouch.hpp` | Creation |
| `src/common/MenuBeaconLCDTouch.cpp` | Creation |
| [src/driver-arm/BeaconSensors.hpp](../src/driver-arm/BeaconSensors.hpp) | readSettings + writeXxx |
| [src/driver-arm/BeaconSensors.cpp](../src/driver-arm/BeaconSensors.cpp) | Impl + decalage offsets getData |
| `src/bot-opos6ul/O_State_NewInit.hpp` | Creation |
| `src/bot-opos6ul/O_State_NewInit.cpp` | Creation (remplace O_State_Init) |
| [src/bot-opos6ul/O_State_Init.cpp](../src/bot-opos6ul/O_State_Init.cpp) | Suppression a l'etape 10 |
| [src/common/action/Sensors.hpp](../src/common/action/Sensors.hpp) | Ajouter accesseur `beacon()` + `beaconConnected()` |
| [src/common/action/LcdShield.hpp](../src/common/action/LcdShield.hpp) | Ajouter `isConnected()` si absent |
| [teensy/IO_t41_ToF_DetectionBeacon/src/TofSensors.h](../../teensy/IO_t41_ToF_DetectionBeacon/src/TofSensors.h) | Ajout `seq_touch`, static_assert 10 bytes (etape 7) |
| [teensy/IO_t41_ToF_DetectionBeacon/src/LCDScreen.cpp](../../teensy/IO_t41_ToF_DetectionBeacon/src/LCDScreen.cpp) | Incrementation `seq_touch` dans callbacks LVGL (etape 7) |
| [teensy/IO_t41_ToF_DetectionBeacon/ARCHITECTURE_BEACON.md](../../teensy/IO_t41_ToF_DetectionBeacon/ARCHITECTURE_BEACON.md) | Mise a jour Settings + seq_touch |
| [robot/ARCHITECTURE.md](ARCHITECTURE.md) | Lien vers ce document |

## 13. Hors scope

- Routines concretes des tests meca (chaque routine sera codee au besoin).
- Modifications UI LVGL balise (sera fait en 2e temps, liste des modifs en section 6.3).
- Support d'une 3e source d'input (extensible plus tard via le meme pattern).
- Gestion fine du scoring en match (existait deja, non touche).
