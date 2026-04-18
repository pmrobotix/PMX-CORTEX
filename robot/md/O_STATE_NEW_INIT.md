# O_State_NewInit — Refactor du menu d'init multi-sources

Document de specification pour remplacer [src/bot-opos6ul/O_State_Init.cpp](../src/bot-opos6ul/O_State_Init.cpp) par une version qui supporte plusieurs interfaces de configuration en parallele (LCD shield 2x16, LCD tactile balise, et eventuelles futures sources) avec une source de verite unique cote Robot.

Contexte et decisions architecturales discutees avant redaction : voir l'historique du refactor. Les choix retenus sont resumes ici et font foi.

## 1. Objectifs

- **Plusieurs sources d'input/output simultanees** pour configurer le robot avant match : LCD shield 2x16 + boutons (existant), LCD tactile balise via I2C 0x2D (existant cote Teensy, voir [ARCHITECTURE_BEACON.md](../../teensy/IO_t41_ToF_DetectionBeacon/ARCHITECTURE_BEACON.md)), et possibilite d'ajouter un 3e systeme plus tard sans reecriture.
- **Degradation gracieuse** : si une des deux interfaces tombe ou est debranchee (en compet ou en preparation), l'autre continue de fonctionner.
- **Source de verite unique** : `Robot` detient l'etat de configuration. Aucune source n'a sa propre copie "faisant foi". Les sources sont de simples plugins input/output.
- **Modele flexible a 3 phases** : CONFIG -> ARMED -> MATCH. La couleur est un parametre comme les autres, editable en CONFIG. Le bouton SETPOS declenche la transition CONFIG -> ARMED (execution de setPos physique + verrou couleur). En ARMED, le bouton devient RESET : retour en CONFIG avec freeMotion pour permettre un repositionnement manuel du robot.
- **Couleur verrouillee apres setPos** : la couleur conditionne la position physique du robot sur la table, donc on ne peut pas la changer apres que `setPos` a positionne le robot. Si erreur de couleur : RESET, repositionner manuellement, refaire SETPOS.
- **Autres parametres editables jusqu'au match** : strategie, diametre adversaire, luminosite LED, tests mecaniques. Toujours modifiables en CONFIG et en ARMED.
- **Support `/k` skipSetup** inchange : court-circuite le menu, params venant du parseur CLI ou defauts (BLEU par defaut).

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
    PHASE_CONFIG = 0,  // Menu ouvert, tout editable (couleur incluse)
    PHASE_ARMED  = 1,  // setPos fait. Couleur LOCKED. Strat/diam/LED/test editables.
    PHASE_MATCH  = 2,  // Tirette retiree, match en cours
    PHASE_END    = 3,  // Fin match
};
```

**Transitions** :
- `CONFIG -> ARMED` : bouton SETPOS clique (touch) ou BACK click (shield). setPos() est execute, robot place, couleur verrouillee.
- `ARMED -> CONFIG` : bouton RESET (touch) ou BACK click (shield). freeMotion + couleur de nouveau editable, pour repositionnement manuel si erreur.
- `ARMED -> MATCH` : edge "tirette inseree PUIS retiree" (detecte en interne dans O_State_NewInit, pas une phase separee).
- `MATCH -> END` : fin des 90s.

**Note sur la detection tirette** : on ne cree pas de phase intermediaire PRIMED (comme dans l'ancien modele). La sequence "insertion puis retrait" est trackee par un `bool tiretteWasInserted` local a la boucle ARMED, qui evite qu'un etat "tirette jamais inseree" soit interprete comme "tirette retiree = start match".

### 3.2 Champs ajoutes

```cpp
protected:
    MatchPhase phase_         = PHASE_CONFIG;
    // myColor_ existe deja (RobotColor, defaut PMXBLUE).
    // Plus de proposedColor_ : myColor_ est directement editable en CONFIG.

    uint8_t    advDiameter_   = 40;   // cm
    uint8_t    ledLuminosity_ = 10;   // 0..100, defaut aligne avec Settings Teensy
    uint8_t    testMode_      = 0;    // 0=aucun, 1..5
    std::atomic<bool> testModeReq_{false};   // test meca a declencher
    std::atomic<bool> setPosReq_{false};     // CONFIG -> ARMED demande
    std::atomic<bool> resetReq_{false};      // ARMED -> CONFIG demande
```

### 3.3 Setters phase-aware

Les setters centralisent les regles de verrouillage. Une source qui tente une modif interdite recoit `false` en retour.

```cpp
// PHASE_CONFIG uniquement (la couleur verrouille le placement physique du robot).
bool setMyColorChecked(RobotColor c);

// PHASE_CONFIG + PHASE_ARMED (editables tant que match pas lance).
bool setStrategyChecked(const std::string& s);
bool setAdvDiameter(uint8_t d);
bool setLedLuminosity(uint8_t l);
bool triggerTestMode(uint8_t t);   // positionne testMode_ + testModeReq_

// Drapeaux d'intention poses par les sources (boutons).
void requestSetPos();     // CONFIG -> ARMED (touch: bouton SETPOS, shield: BACK click)
bool setPosRequested() const;
void clearSetPos();

void requestReset();      // ARMED -> CONFIG (touch: bouton RESET, shield: BACK click)
bool resetRequested() const;
void clearReset();

// Gestion de phase (appelee uniquement par O_State_NewInit)
MatchPhase phase() const;
void setPhase(MatchPhase p);
```

**Plus de `proposedColor_`/`commitColor()`** : dans le nouveau modele, la couleur est un parametre comme les autres. `setMyColorChecked` ecrit directement dans `myColor_` en CONFIG, et refuse en ARMED+. Le "commit" implicite est la transition CONFIG -> ARMED via `requestSetPos()`.

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

### 5.1 Layout LCD 2 lignes

**Ligne 0** (toujours) : synthese de la config.
```
Y* S2  D:40 T:-
```
- pos 0-1 : couleur + verrou (`Y*`/`B*` en ARMED, `Y `/`B ` en CONFIG)
- pos 3-4 : strat courte (S1/S2/S3)
- pos 7-10 : diametre `D:40`
- pos 12-14 : dernier test `T:-` / `T:1`..`T:5`

**Ligne 1** (contextuelle) : rubrique en cours d'edition + valeur.
```
>COLOR:YELLOW
>DIAM :40 cm
>STRAT:S2
>TEST :T3
```
Pendant un hold BACK, la ligne 1 est remplacee par le countdown `EXIT IN  8.5s`.

**VRR et LED ne sont PAS editables sur le shield** (gardes pour le touch LVGL uniquement). Cote shield, LED luminosity est ignoree (Teensy ou CLI fixent la valeur). VRR est supprime du menu 2026 (strategies 1/2/3 suffisent).

**Correspondance strat longue <-> courte** :

| Long (Robot) | Court (LCD) |
|---|---|
| `tabletest` | `S1` |
| `strat2` | `S2` |
| `strat3` | `S3` |

### 5.2 Interaction boutons

**Principe simple** : LEFT/RIGHT = rubrique precedente/suivante ; UP/DOWN = valeur suivante/precedente ; ENTER = trigger test ; BACK = setPos (CONFIG) ou reset (ARMED) ou exit (hold 10s).

Cycle des rubriques selon la phase :
- **PHASE_CONFIG** : `COLOR -> DIAM -> STRAT -> TEST -> (COLOR)`
- **PHASE_ARMED**  : `DIAM -> STRAT -> TEST -> (DIAM)` (COLOR skippee automatiquement)

| Bouton | Clic court | Hold 10s |
|---|---|---|
| LEFT  | rubrique precedente | - |
| RIGHT | rubrique suivante | - |
| UP    | valeur "haut" (pour COLOR: `setMyColorChecked(BLUE)`, pour DIAM/STRAT/TEST: valeur +1) | - |
| DOWN  | valeur "bas" (pour COLOR: `setMyColorChecked(YELLOW)`, autre: -1) | - |
| ENTER | si rubrique=TEST : `triggerTestMode(testSelected_)` | - |
| BACK  | CONFIG : `requestSetPos()` ; ARMED : `requestReset()` | `exit(0)` |

**Plus de hold 5s pour commit couleur** : le commit implicite se fait via `requestSetPos()` qui transitionne CONFIG -> ARMED.

**Wrapping des valeurs** : UP sur `DIAM:250` -> `DIAM:5`, DOWN sur `STRAT:S1` -> `STRAT:S3`, etc.

**Convention UP/DOWN pour couleur** : UP = BLEU, DOWN = JAUNE (cohere avec la localisation physique sur la table).

### 5.3 Detection de presence shield

`MenuShieldLCD::isAlive()` = resultat du dernier acces I2C au shield. Si un `writeReg` echoue, on bascule alive=false, et le controller arrete d'appeler `refreshDisplay` sur cette source jusqu'au prochain check.

Au boot de `O_State_NewInit`, un probe explicite est fait via `LcdShield::isConnected()` (a ajouter si pas present).

### 5.4 Fichiers a creer

- `src/common/MenuShieldLCD.hpp`
- `src/common/MenuShieldLCD.cpp`

Dependances : `LcdShield`, `ButtonBar`, `Robot`.

## 6. `MenuBeaconLCDTouch` — LCD tactile balise

Premiere etape : lecture uniquement (OPOS6UL recupere ce que l'operateur tape sur le LCD tactile). Modifications cote Teensy listees en 6.3.

### 6.1 Architecture I2C — syncFull (init) vs sync (match)

Pour eviter les problemes de synchronisation I2C (crash du bus sous stress
lors de clics rapides sur le LCD tactile), tout l'I2C beacon est regroupe
dans **une seule methode atomique** :

- **Init (O_State_NewInit)** : `sensors.syncFull()` appele dans les boucles
  CONFIG et ARMED, **avant** `ctrl.tick()`. Fait sous un seul lock :
  1. Ecrit les Settings pending vers la Teensy (si `pending_dirty_`)
  2. Lit les Settings depuis la Teensy -> `cached_settings_`
  3. Lit flag + getData (donnees beacon adv) si nouvelle donnee disponible
- **Match (SensorsThread)** : `sync()` inchange, appele par le timer a 20ms.
  Ne fait que flag + getData (pas de Settings).

`MenuBeaconLCDTouch` ne fait **aucun I2C** directement :
- `readMatchSettings()` retourne `cached_settings_` (zero I2C)
- `writeXxx()` ecrit dans `cached_settings_` + `pending_dirty_ = true` (zero I2C)
- L'I2C effectif est fait au prochain `syncFull()`.

```
Boucle O_State_NewInit (10ms) :
  sensors.syncFull()    <-- tout l'I2C ici, 1 seul lock
  ctrl.tick()           <-- pollInputs + refreshDisplay sur cache, zero I2C
```

### 6.2 Comportement MenuBeaconLCDTouch

**`pollInputs`** :
1. Lit le bloc Settings cache via `Sensors::readMatchSettings()` (retourne `cached_settings_`, pas d'I2C).
2. Si `seq_touch` a change depuis le dernier poll : nouveau clic touch detecte.
3. Pour chaque champ different de `shadow_`, appelle le setter Robot approprie :
   - `matchColor` -> `setMyColorChecked(PMXBLUE/PMXYELLOW)` (accepte en CONFIG uniquement)
   - `strategy` -> `setStrategyChecked("tabletest"/"strat2"/"strat3")`
   - `advDiameter` -> `setAdvDiameter(...)`
   - `ledLuminosity` -> `setLedLuminosity(...)`
   - `testMode` -> `triggerTestMode(...)` si != 0
4. Si `actionReq == 1` (bouton SETPOS/RESET clique sur le touch) :
   - phase == CONFIG -> `robot.requestSetPos()`
   - phase == ARMED -> `robot.requestReset()`
   - Ecrit `actionReq = 0` via `writeActionReq(0)` (marque pending, ecrit au prochain syncFull).
5. Si regression de `seq_touch` : reboot Teensy detecte, reset du compteur local, pas d'adoption ce tick.

**`refreshDisplay`** :
1. Push Robot vers Settings Teensy via les `Sensors::writeXxx()` (marque pending, zero I2C) :
   - `matchColor` (0/1 depuis `robot.getMyColor()`)
   - `strategy` (1..3 via mapping inverse)
   - `advDiameter`
   - `ledLuminosity` (via `writeLedLuminosity`)
   - `matchState` = `(uint8_t)robot.phase()` directement (0=CONFIG, 1=ARMED, 2=MATCH, 3=END)
2. Met a jour `shadow_` avec les valeurs poussees pour ne pas re-declencher un faux "delta" au prochain pollInputs.
3. **Ne touche jamais `seq_touch`** (c'est la Teensy qui l'incremente).
4. **Pas de push de `matchColor` en ARMED+** : la couleur est lockee cote Robot, et le bouton LVGL est verrouille cote Teensy via `matchState >= 1`.

**`isAlive`** : basee sur `settings_valid_` (derniere lecture Settings OK dans syncFull). Initialise a `true` au constructeur pour permettre un premier `pollInputs` (c'est lui qui determine l'etat reel).

### 6.3 Fichiers OPOS6UL

- `src/common/menu/MenuBeaconLCDTouch.hpp/cpp` : inchange (travaille sur cache/pending de facon transparente).
- `src/driver-arm/BeaconSensors.hpp/cpp` : `readSettings(Settings&)` lit 11 bytes + `writeMatchColor/Strategy/...` (I2C bas niveau, appele par syncFull).
- `src/driver-arm/SensorsDriver.hpp/cpp` : `syncFull()` regroupe tout l'I2C. `readMatchSettings()` retourne cache. `writeXxx()` marque pending.
- `src/common/action/Sensors.hpp` : wrappers `readMatchSettings` + `writeXxx` + `syncFull()`.
- `src/common/interface/ASensorsDriver.hpp` : struct `MatchSettingsData` (11 bytes) + virtuelles `readMatchSettings` + `writeXxx` + `syncFull()` (defaults no-op pour SIMU).

### 6.4 Cote Teensy — etat actuel (fait)

- `TofSensors.h` struct `Settings` : 11 bytes (5 bloc 1 + 5 bloc 2 + 1 seq_touch).
- Champ `actionReq` (reg 9) ajoute au bloc 2 : le bouton SETPOS/RESET du LCD tactile ecrit `actionReq=1` + `seq_touch++`.
- Champ `seq_touch` (reg 10) : incremente par CHAQUE callback LVGL qui modifie un champ du bloc 2 (matchColor, strategy, testMode, advDiameter, ledLuminosity, actionReq).
- `static_assert(sizeof(Settings) == 11)` verrouille l'ABI I2C.
- `LCDScreen.cpp` :
  - Bouton SETPOS/RESET (`btn_setpos_handle`) : 152x50 pixels a droite du bouton couleur (152x50 a gauche), partageant la largeur de l'ecran. Label/couleur change selon `settings.matchState` : vert "SETPOS" en 0 (CONFIG), rouge "RESET" en 1 (ARMED), gris "MATCH" en >=2.
  - `screen_loop()` @5 Hz compare chaque champ a sa valeur precedente mise en cache et rafraichit les widgets LVGL (bouton couleur, strategy radio, labels diam/led, bouton setpos, lock couleur en ARMED+).
  - `updateColorButtonLock()` met le bouton couleur en `LV_STATE_DISABLED` quand `matchState >= 1` (empeche le toggle tactile apres setPos).

### 6.4 Migration I2C (flash + deploiement coordonnes)

Flasher la Teensy **ET** redeployer le binaire OPOS6UL (ARM) dans la meme session. Ordre important :

1. **Teensy** : flasher le firmware avec `Settings` a 11 bytes + `actionReq` + bouton SETPOS/RESET.
2. **OPOS6UL** : deployer `bot-opos6ul` build avec `SETTINGS_SIZE_BeaconSensors = 11` + struct miroir.

Entre les 2 flashs, la detection ToF sera cassee (offsets `Registers` decales). Faire les flashs en rapide sequence.

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
        // Plus de check NOCOLOR : myColor_ defaut = PMXBLUE, le CLI peut
        // basculer en JAUNE via /b. Toujours une valeur valide.
        robot.setPhase(PHASE_ARMED);
        setPos();
        robot.actions().sensors().writeLedLuminosity(50);
        robot.waitForInit(true);
        robot.setPhase(PHASE_MATCH);
    }
    //========================================================
    // MENU NORMAL (multi-sources)
    //========================================================
    else {
        MenuController ctrl(robot);
        if (robot.actions().lcd2x16().is_connected()) {
            ctrl.add(std::make_unique<MenuShieldLCD>(
                robot.actions().lcd2x16(), robot.actions().buttonBar()));
        }
        if (robot.actions().sensors().is_connected()) {
            ctrl.add(std::make_unique<MenuBeaconLCDTouch>(robot.actions().sensors()));
        }
        if (!ctrl.anyAlive()) {
            logger().error() << "No menu source alive, EXIT" << logs::end;
            std::exit(1);
        }

restart_menu:
        robot.setPhase(PHASE_CONFIG);
        robot.clearSetPos();
        robot.clearReset();
        robot.clearTestMode();

        // ===== PHASE CONFIG : attente setPos =====
        // Tous les parametres editables (couleur, strat, diam, LED, tests).
        // L'operateur clique SETPOS (touch) ou BACK click (shield) pour passer en ARMED.
        while (robot.phase() == PHASE_CONFIG) {
            ctrl.tick();
            handleTestModeRequest();
            if (robot.setPosRequested()) {
                robot.clearSetPos();
                robot.setPhase(PHASE_ARMED);
                break;
            }
            if (!ctrl.anyAlive()) std::exit(1);
            utils::sleep_for_micros(10000);
        }

        // setPos : robot rejoint sa position initiale. Inclus reset asserv
        // via startMotionTimerAndOdo(true) (reset Nucleo + match ref).
        setPos();
        robot.actions().sensors().writeLedLuminosity(50);

        // ===== PHASE ARMED : attente sequence tirette, reset possible =====
        // Couleur LOCKED. Strat/diam/LED/tests restent editables.
        // Etat interne : detecter l'edge "tirette inseree PUIS retiree".
        bool tiretteWasInserted = false;
        while (robot.phase() == PHASE_ARMED) {
            ctrl.tick();
            handleTestModeRequest();

            if (robot.resetRequested()) {
                robot.clearReset();
                robot.asserv().freeMotion();
                goto restart_menu;   // repositionnement manuel possible
            }

            bool tirettePressed = robot.actions().tirette().pressed();
            if (!tiretteWasInserted && tirettePressed)  tiretteWasInserted = true;
            if (tiretteWasInserted && !tirettePressed) {
                robot.setPhase(PHASE_MATCH);
                break;
            }

            if (!ctrl.anyAlive()) std::exit(1);
            utils::sleep_for_micros(10000);
        }
    }

    //========================================================
    // Depart match (commun /k + menu) — inchange
    //========================================================
    robot.actions().ledBar().stop(true);
    robot.actions().ledBar().resetAll();
    robot.actions().lcd2x16().clear();
    robot.actions().lcd2x16().print("GO...");
    robot.displayPoints();

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

## 8. Flux de donnees, sequences et scenarios de panne

Cette section detaille **comment chaque LCD / touch LCD / bouton reste independant** et comment les modifications se propagent entre les deux interfaces. Elle complete la vue d'ensemble de la section 2 avec les sequences concretes.

### 8.1 Architecture complete (hardware + logiciel)

```
+----------------------------------------------------------------------------+
|  OPOS6UL (Linux, process bot-opos6ul)                                      |
|                                                                            |
|                   +--------------------------------+                       |
|                   |  Robot (SOURCE DE VERITE)      |                       |
|                   |  myColor_, proposedColor_      |                       |
|                   |  strategy_, configVRR_         |                       |
|                   |  advDiameter_, ledLuminosity_  |                       |
|                   |  testMode_, phase_             |                       |
|                   +----------+------------+--------+                       |
|                              ^            |                                |
|                          setters()     getters()                           |
|                              |            v                                |
|                   +----------+---------------------+                       |
|                   |  MenuController::tick() @100Hz |                       |
|                   |                                |                       |
|                   |  for s in sources:             |                       |
|                   |    s->pollInputs(robot)   (A)  |                       |
|                   |  for s in sources:             |                       |
|                   |    if (alive) refreshDisplay(B)|                       |
|                   +------+-----------------+-------+                       |
|                          |                 |                               |
|              +-----------+-------+   +-----+----------------+              |
|              | MenuShieldLCD     |   | MenuBeaconLCDTouch   |              |
|              |                   |   |                      |              |
|              | state interne:    |   | state interne:       |              |
|              |  - lastButton_    |   |  - shadow_           |              |
|              |  - holdChrono_    |   |  - lastSeqTouchSeen_ |              |
|              |  - editField_     |   |  - alive_            |              |
|              +--+----------+-----+   +--+--------------+----+              |
|                 |          |            |              |                   |
|          pollInputs   refreshDisplay    |              |                   |
|                 |          |            |              |                   |
|                 v          v            v              v                   |
|           +----------+  +--------+  +-----------------------+              |
|           |ButtonBar |  |LcdShield| |Sensors::readMatchSettings/           |
|           |.check...()| |.print() | |       writeXxx / writeLedLum         |
|           +----+-----+  +---+----+  +-------+--------------+               |
|                |            |               |                              |
|                |  I2C bus 0 |               |  I2C bus 0                   |
|                |  shield    |               |  beacon 0x2D                 |
+----------------+------------+---------------+------------------------------+
                 |            |               |
                 v            v               v
+-----------------------------+   +-----------------------------------------+
|  Shield 2x16 (hardware)     |   |  Teensy 4.1 (slave I2C 0x2D)            |
|                             |   |                                         |
|  +--------+  +-----------+  |   |  +-------------------------------+      |
|  |boutons |  | LCD 2x16  |  |   |  |  Settings (10 bytes RAM)      |      |
|  |UP/DN/  |  | (HD44780) |  |   |  |   - bloc 1 (OPO->TNY)         |      |
|  |L/R/ENT/|  |           |  |   |  |   - bloc 2 (TNY->OPO)         |      |
|  |BACK    |  |           |  |   |  |   - seq_touch (TNY only)      |      |
|  +--------+  +-----------+  |   |  +--------+-----------+----------+      |
|                             |   |           ^           |                 |
+-----------------------------+   |    LVGL cb| (UI->RAM) |screen_loop @5Hz |
                                  |           |           |(RAM->UI)        |
                                  |  +--------+------+    v                 |
                                  |  | ecran tactile |   widgets LVGL       |
                                  |  | ILI9341+touch |   btn_color, strat,  |
                                  |  |               |   diam, lum          |
                                  |  +---------------+                      |
                                  +-----------------------------------------+
```

### 8.2 Regle d'or : "syncFull then poll all then refresh all"

Un tick complet dans O_State_NewInit :

```
+---------------- TICK (toutes les 10ms) ----------------+
|                                                        |
|  (0) SYNC FULL I2C                                     |
|   sensors.syncFull()                                   |
|     1. write pending Settings -> Teensy (si dirty)     |
|     2. read Settings <- Teensy -> cached_settings_     |
|     3. read flag + getData (si new data adv)           |
|   ==> tout l'I2C est fait ici, 1 seul lock             |
|                                                        |
|  (A) POLL INPUTS (zero I2C)                            |
|   +---------------+      lit boutons                   |
|   | MenuShieldLCD | -->  shield -> robot.setXxx()      |
|   +---------------+                                    |
|   +-------------------+  lit Settings depuis cache     |
|   |MenuBeaconLCDTouch |-> compare seq_touch + shadow   |
|   |                   |  si nouveau clic -> robot.set* |
|   +-------------------+                                |
|                                                        |
|  vvv Robot a integre tous les inputs vvv               |
|                                                        |
|  (B) REFRESH DISPLAY (zero I2C beacon)                 |
|   +---------------+      lit robot.getXxx()            |
|   | MenuShieldLCD | -->  print sur LCD 2x16            |
|   +---------------+                                    |
|   +-------------------+  lit robot.getXxx()            |
|   |MenuBeaconLCDTouch |-> writeXxx marque pending      |
|   |                   |  (ecrit au prochain syncFull)   |
|   +-------------------+                                |
+--------------------------------------------------------+
```

Cet ordre garantit qu'a la fin du tick, tous les displays voient **le meme etat Robot coherent**, celui qui a integre tous les inputs du tick. Voir section 2 pour les 4 fleches numerotees (controller/source/Robot/HW).

### 8.3 Sequence : clic UP sur shield en CONFIG -> affichage sur touch LVGL

```
Time    Composant               Action
------  ----------------------  --------------------------------------------
t=0ms   User                    appuie sur UP du shield
        ButtonBar (I2C)         lit l'etat physique des boutons

t=10ms  sensors.syncFull()          <- tout l'I2C ici (write pending + read settings + read flag)
        MenuController.tick()
        |- MenuShieldLCD.pollInputs
        |    btns_.check...() == BUTTON_UP_KEY
        |    lastButton_ = UP, holdChrono_.start()
        |    holdActive_ = true
        |
        |- MenuBeaconLCDTouch.pollInputs
        |    readMatchSettings(current)  <- retourne cached_settings_ (zero I2C)
        |    current.seq_touch == shadow_ (rien bouge cote touch)
        |    -> pas d'adoption
        |
        |- MenuShieldLCD.refreshDisplay
        |    (robot.proposedColor est encore NOCOLOR)
        |    buildLine0 -> "__ S- VRR  D:40"
        |    pas de changement vs lastLine0_, pas d'ecriture LCD
        |
        '- MenuBeaconLCDTouch.refreshDisplay
             colorByte = 0 (NOCOLOR), shadow.matchColor = 0 (default)
             pas de push (pending_dirty_ reste false)

...        user maintient UP, polls continuent, holdChrono avance

--- USER RELACHE UP a t=150ms ---------------------------------------------

t=150ms MenuShieldLCD.pollInputs
        btns_.check...() == BUTTON_NONE
        holdActive_ etait true, donc "click court"
        -> handleClick(b=UP)
        -> robot.proposeColor(PMXBLUE)    <- ROBOT MODIFIE ICI

t=150ms MenuBeaconLCDTouch.refreshDisplay (meme tick, phase B)
        colorByte = (BLUE == YELLOW) ? 1 : 0 = 0
        shadow_.matchColor = 0, donc PAS de push (pas un changement)
        [OK ici car Teensy default matchColor=0 = BLEU, deja affiche]

--- USER CLIQUE DOWN a t=500ms (propose YELLOW) ---------------------------

t=510ms sensors.syncFull()           <- ecrit pending (rien ici), lit settings
        MenuShieldLCD.pollInputs    -> proposeColor(PMXYELLOW)
t=510ms MenuBeaconLCDTouch.refreshDisplay
        colorByte = 1, shadow_.matchColor = 0 -> DIFFERENT
        -> writeMatchColor(1) marque pending_dirty_ = true
        shadow_.matchColor = 1
t=520ms sensors.syncFull()           <- ecrit matchColor=1 vers Teensy (pending)
        Teensy: settings.matchColor = 1

t=510-710ms Teensy screen_loop()    (toutes les 200ms)
        last_matchColor = 0 (cache) != settings.matchColor = 1
        updateColorButton(btn_color_handle)
        -> le bouton tactile passe visuellement a "JAUNE"
        last_matchColor = 1
```

**Latence totale shield -> touch : < 220 ms** (10 ms OPOS6UL tick + 200 ms max Teensy screen_loop cycle).

### 8.4 Sequence inverse : clic touch -> affichage shield

```
Time    Composant               Action
------  ----------------------  --------------------------------------------
t=0ms   User                    tape sur le bouton "BLEU/JAUNE" du touch
        LVGL dispatch l'event   matchColor_event_cb()
                                settings.matchColor = 1 - settings.matchColor
                                settings.seq_touch++     <- CLE DE LA SYNCHRO
                                updateColorButton(btn)   <- refresh local immediat

t=10ms  (prochain tick OPOS6UL)
        sensors.syncFull()          <- lit cached_settings_ avec seq_touch=N+1
        MenuBeaconLCDTouch.pollInputs
        readMatchSettings(current)  <- retourne cached_settings_ (zero I2C)
        current.seq_touch = N+1,  lastSeqTouchSeen_ = N
        -> nouveau clic detecte
        current.matchColor = 1, shadow_.matchColor = 0 (etait a 0)
        -> robot.proposeColor(PMXYELLOW)    <- ROBOT MODIFIE ICI
        shadow_.matchColor = 1
        lastSeqTouchSeen_ = N+1

t=10ms  MenuShieldLCD.refreshDisplay
        buildLine0 reflete robot.proposedColor == YELLOW
        -> "Y? S- VRR  D:40 "
        diff vs lastLine0_ -> ecrit sur LCD 2x16
        -> shield affiche "Y?"

t=10ms  MenuBeaconLCDTouch.refreshDisplay
        colorByte = 1, shadow_.matchColor = 1, same, PAS de push
        [c'est ca qui empeche une boucle infinie : on ne re-push pas ce
         qu'on vient de lire]
```

**Latence totale touch -> shield : ~ 10 ms** (1 tick OPOS6UL).

### 8.5 Scenarios de panne — independance des sources

**Scenario 1 : Shield LCD debranche (ou jamais connecte)**

```
- Au boot: lcd_.is_connected() = false
- MenuShieldLCD::alive_ = false
- controller.anyAlive() reste true grace a MenuBeaconLCDTouch

TICK :
  sensors.syncFull()            <- OK (lit beacon I2C)
  MenuShieldLCD.pollInputs    -> if(!alive_) return;  [no-op]
  MenuBeaconLCDTouch.pollInputs -> OK (lit cache, met a jour Robot)
  MenuShieldLCD.refreshDisplay  -> gate isAlive()=false, skip
  MenuBeaconLCDTouch.refreshDisplay -> OK (marque pending, zero I2C)

Resultat : seul le touch LVGL est utilisable. Match fonctionnel.
```

**Scenario 2 : Balise Teensy debranchee (ou cable I2C coupe)**

```
TICK :
  sensors.syncFull()            <- I2C err -> settings_valid_=false, return -1
  MenuShieldLCD.pollInputs     -> OK (boutons lus)
  MenuBeaconLCDTouch.pollInputs
    readMatchSettings() -> retourne settings_valid_=false -> alive_ = false, return
  MenuShieldLCD.refreshDisplay -> OK (LCD 2x16 mis a jour depuis Robot)
  MenuBeaconLCDTouch.refreshDisplay
    gate isAlive()=false -> skip [on n'ecrit pas vers un bus mort]

Resultat : seul le shield est utilisable. Match fonctionnel.

Recuperation si la balise revient :
  Prochain tick : syncFull() reussit -> settings_valid_=true.
  pollInputs lit le cache OK -> alive_ = true -> source reactivee automatiquement.
```

**Scenario 3 : Teensy reboot en plein menu (glitch alim / ESD)**

```
Avant reboot :  settings.matchColor=1, seq_touch=42
                OPOS6UL: lastSeqTouchSeen_=42, shadow.matchColor=1
                Robot: myColor_=YELLOW  (commit deja fait)

Teensy reboot -> settings = defaults (matchColor=0, seq_touch=0)

TICK apres reboot :
  MenuBeaconLCDTouch.pollInputs
    current.seq_touch = 0
    seqActive = true (lastSeqTouchSeen_=42 != 0)
    0 < 42 -> REGRESSION detectee -> Teensy a reboote
    -> logger().warn + lastSeqTouchSeen_ = 0
    -> return (on n'adopte RIEN cote Robot)

  MenuBeaconLCDTouch.refreshDisplay (meme tick)
    colorByte = 1 (Robot.myColor=YELLOW), shadow.matchColor = 1
    -> shadow matche Robot, pas de push immediat

Limite connue : la recuperation apres reboot Teensy est imparfaite
(le fallback delta peut re-adopter les defaults). Cas rare en competition
(alim stable). Contournement possible : flagger l'etat "post-reboot" pour
ignorer le delta fields pendant quelques ticks et forcer un push Robot.
```

### 8.6 Pourquoi les differents mecanismes sont-ils necessaires

| Mecanisme | Probleme qu'il resout |
|-----------|----------------------|
| Source de verite unique (`Robot`) | Conflits si chaque source avait sa propre copie. Ici, une seule valeur de color/strat/... a l'instant t, partagee par tous les getters. |
| "poll all then refresh all" | Empeche les inconsistances entre sources dans un meme tick. Ex: si poll+refresh etait entrelace, un clic touch adopte puis le shield re-pousse l'ancien Robot avant d'avoir vu la modif. |
| `seq_touch` (cote Teensy) | Dedup + anti-loop. Sans lui, OPOS6UL relit sa propre ecriture via I2C et croit que le touch a clique. Permet aussi de detecter un reboot Teensy (regression du compteur). |
| `shadow_` (cote MenuBeaconLCDTouch) | Fallback pour la detection si seq_touch inactif (ex: ancien firmware Teensy). Et mis a jour apres chaque push OPOS6UL pour eviter de re-adopter sa propre ecriture. |
| `screen_loop()` Teensy @5Hz | Diffusion cross-MCU : sans ce refresh periodique cote Teensy, les widgets LVGL ne voient JAMAIS les modifs ecrites en I2C par l'OPOS6UL (seul un clic tactile declenchait un redraw). |
| `alive_ = true` au ctor + pollInputs sans gate | Sans ca, une source dont le 1er poll echoue reste morte pour toujours (le controller la skipperait). Permet aussi la reconnexion a chaud. |

### 8.7 Paths I2C reels (recap)

```
            CLIC SHIELD                        CLIC TOUCH
                |                                  |
                v                                  v
   ButtonDriver/LCDShieldDriver           LVGL matchColor_event_cb
   (I2C bus 0 sur OPOS6UL)                (interne Teensy, pas d'I2C)
   lit boutons via MCP23017               modifie settings.matchColor
                                          + seq_touch++
                |                                  |
                v                                  v
      MenuShieldLCD::pollInputs      [OPOS6UL poll @100Hz via I2C:
      -> robot.setXxx()               MenuBeaconLCDTouch::pollInputs
                                      I2C read 10 bytes @0x2D]
                                                  |
                                                  v
                                      detecte delta seq_touch
                                      -> robot.setXxx()
                                                  |
                                                  v
                                  +---------------+----------------+
                                  |                                |
                         MenuShieldLCD                 MenuBeaconLCDTouch
                         .refreshDisplay               .refreshDisplay
                         (lit robot)                   (lit robot)
                                  |                                |
                                  v                                v
                         LcdShield::print()            sensors_.writeXxx()
                         (I2C bus 0, MCP23017)         (I2C bus 0 @0x2D,
                                                        writeReg par byte)
                                                                  |
                                                                  v
                                                      [Teensy i2c_register_slave
                                                       ecrit dans settings.X]
                                                                  |
                                                                  v
                                                      Teensy screen_loop() @5Hz
                                                      compare settings.X vs cache
                                                      -> lv_label_set_text ou
                                                         updateColorButton
                                                                  |
                                                                  v
                                                      redraw LVGL sur ILI9341
```

**Latences cote OPOS6UL** : 10 ms (1 tick @ 100 Hz).
**Latences cote Teensy** : jusqu'a 200 ms (1 cycle `screen_loop`). C'est pour ca que les changements shield mettent quelques dizaines a ~200 ms avant d'apparaitre sur le touch.

## 9. Interaction avec la memoire `myColor_` et strategy existantes

`Robot` a deja `myColor_`, `strategy_`, `configVRR_`. Ces champs restent et sont reutilises. La seule addition est `proposedColor_` + les phase-aware setters + les nouveaux champs `advDiameter_`/`ledLuminosity_`/`testMode_`.

**Retrocompatibilite** : le parseur CLI (Arguments class) continue d'ecrire dans `myColor_`/`strategy_`/`configVRR_` via les setters existants. En mode `/k`, ces setters doivent accepter l'ecriture meme si `phase_ == PHASE_CONFIG` (cas normal post-construction). A verifier qu'aucun ordonnancement ne casse : l'ordre est `construct Robot (phase=CONFIG) -> parseCLI -> execute O_State_NewInit`, donc CLI ecrit en phase CONFIG, OK.

## 10. Ordre d'implementation

Chaque etape doit etre **compilable + testable en simu et sur robot reel** avant de passer a la suivante.

### Etape 1 : phase enum + champs Robot ✅

- ~~Ajouter `MatchPhase`, `phase_`, `proposedColor_`, `advDiameter_`, `ledLuminosity_`, `testMode_`, drapeaux dans Robot~~
- ~~Ajouter getters/setters phase-aware~~
- Aucun changement de comportement runtime (O_State_Init existant continue d'etre utilise). La phase est toujours `PHASE_CONFIG` car personne ne la change.
- **Test** : compile, simu OK, match OK via O_State_Init existant.

### Etape 2 : BeaconSensors read/write Settings ✅

- ~~Ajouter `readSettings(Settings&)` + writeXxx() par champ~~
- **Test** : un petit programme ad-hoc (ou un bouton de debug) qui lit + reecrit les Settings, verifier via logs et via affichage LCD tactile que les valeurs sont bien vues des deux cotes.

### Etape 3 : IMenuSource + MenuController ✅

- ~~Interface + controller + squelette vide~~
- ~~Aucun source encore, juste la plomberie~~
- **Test** : compile.

### Etape 4 : MenuShieldLCD ✅

- ~~Implementation complete (status line 0+1, cursor, hold 2s commit, pages COMMITTED, reset, exit)~~
- **Test en isolation** : un petit binaire de test qui instancie `Robot + MenuController + MenuShieldLCD` et tourne la boucle, verification manuelle de tous les boutons et des affichages. Sans le reste du robot.

### Etape 5 : MenuBeaconLCDTouch cote OPOS6UL ✅

- ~~Implementation qui lit les Settings actuels et pousse a chaque refreshDisplay~~
- ~~Detection de delta via comparaison simple avec shadow~~
- ~~seq_touch integre (detection reboot Teensy, anti-loop, validation actionReq)~~
- **Test** : shield inactif, beacon presente. On modifie les settings sur le LCD tactile, on verifie que Robot les reflete (via logs ou ligne 0 du shield si on le laisse branche pour observer).

### Etape 6 : O_State_NewInit.cpp ✅

- ~~Nouveau fichier qui remplace l'usage de O_State_Init dans l'automate~~
- ~~Branchement du controller, flow CONFIG/ARMED/MATCH, /k~~
- ~~Enregistre dans OPOS6UL_RobotExtended.cpp~~
- **Test** : parcours complet sur robot reel. Tester toutes les transitions, reset, exit.

### Etape 7 : modifs Teensy `seq_touch` ✅

- ~~Ajout `seq_touch` dans Settings + incrementation dans callbacks LVGL~~
- ~~Flash Teensy~~
- ~~Decaler les offsets cote `BeaconSensors::getData()`~~
- ~~`static_assert(sizeof(Settings) == 11)` cote Teensy~~
- **Test** : repeter le test etape 5 + ajouter des cas de concurrence shield+touch dans un intervalle < 100 ms, verifier que seq_touch evite les pertes.

### Etape 8 : `testMode` dispatch + routines ⚠️ partiel

- ~~`handleTestModeRequest()` framework + dispatch switch~~
- ~~Case 1 : `ax12_init()` fonctionne~~
- [ ] Case 2 : ToF/beacon (TODO)
- [ ] Case 3 : aspiration (TODO)
- [ ] Case 4 : LED bar (TODO)
- [ ] Case 5 : servos (TODO)
- Chaque routine self-cleans (remet la meca en position de depart)
- **Test** : chaque routine lancable depuis shield (page TEST) et depuis touch (boutons T1..T5).

### Etape 9 : pages shield secondaires completes ✅

- ~~Curseur multi-champs (`EditField` enum), navigation LEFT/RIGHT, edition UP/DOWN, wrapping~~
- ~~Skip automatique de COLOR en ARMED~~
- **Test** : parcours de tous les champs, verification des limites, cursor blink.

### Etape 10 : cleanup ⬜

- [ ] Supprimer `O_State_Init.cpp` si `O_State_NewInit` est stable
- [ ] Mettre a jour [ARCHITECTURE.md](ARCHITECTURE.md) pour pointer sur ce document
- [ ] Mettre a jour [HARDWARE_CONFIG.md](HARDWARE_CONFIG.md) si les flags d'activation doivent evoluer

## 11. Tests manuels de resilience

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

## 12. Points ouverts / evolutions futures

- **Persistance EEPROM cote Teensy** (evoque dans ARCHITECTURE_BEACON.md section "Evolutions prevues") : rappeler les derniers choix operateur au reboot. Ne change rien cote OPOS6UL tant que `seq_touch` repart a 0 (ce qui est le comportement souhaite meme avec EEPROM, car on veut detecter le reboot comme un signal).
- **3e source potentielle** (web UI, telecommande, ...) : creer une classe qui herite de `IMenuSource`, l'ajouter au controller au boot si detectee. Zero impact sur le reste.
- **`matchNumber`** : non gere dans ce refactor (non pertinent pour la Coupe 2026 selon [ARCHITECTURE_BEACON.md](../../teensy/IO_t41_ToF_DetectionBeacon/ARCHITECTURE_BEACON.md)). A ajouter si besoin en reutilisant le meme pattern.
- **Synchronisation horloge entre OPOS6UL et Teensy** : pas necessaire pour la config. Si besoin pour les timestamps ToF, utiliser le champ `seq` des Registers ToF deja present.

## 13. Fichiers impactes (recapitulatif)

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

## 14. Hors scope

- Routines concretes des tests meca (chaque routine sera codee au besoin).
- Modifications UI LVGL balise (sera fait en 2e temps, liste des modifs en section 6.3).
- Support d'une 3e source d'input (extensible plus tard via le meme pattern).
- Gestion fine du scoring en match (existait deja, non touche).
