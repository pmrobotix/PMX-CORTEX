# Exécution de stratégie JSON côté PMX-CORTEX

> Document de référence — architecture et usage du système de stratégie JSON
> (simulateur JS et, à terme, runner C++).
>
> Pour la spécification détaillée du format, voir [STRATEGY_JSON_FORMAT.md](STRATEGY_JSON_FORMAT.md).

---

## 1. Vue d'ensemble

Le système repose sur **un format JSON unique** pour décrire une stratégie, utilisé dans deux sens :

```
       ┌─────────────────────┐
       │   Stratégie JSON    │
       │ (strategyPMX0.json) │
       └──────┬──────────────┘
              │
      ┌───────┴───────┐
      │               │
      ▼               ▼
  ┌───────┐     ┌──────────────┐
  │ Simu  │     │ Runner C++   │
  │ (JS)  │     │ (à venir)    │
  └───────┘     └──────────────┘
  tween +          vraie exec
  calcul auto      avec A*,
  position,        asserv,
  mirror jaune,    actuators,
  filtre zones     flags
```

**Format = EsialRobotik Ia-Python 2026 structuré** (array d'instructions →
tasks[] avec `type`/`subtype` explicites). Noms de subtypes alignés 1:1 sur
les méthodes `Navigator` C++ (camelCase → UPPER_SNAKE_CASE).

**Export C++ → JSON** :
- `ZoneJsonExporter` écrit `table.json` (zones) à partir des obstacles Playground + IA ✅
- À venir : `StrategyJsonExporter` écrira `strategyPMX0.json` à partir d'une stratégie C++

**Import JSON → C++** :
- À venir : `StrategyJsonRunner` lit un JSON et exécute chaque étape via les vraies
  fonctions C++ (`Navigator`, `Asserv`, `ActionRegistry`, `FlagManager`)

---

## 2. État actuel (2026-04-19)

| Composant | État |
|---|---|
| Fork simulateur `pmrobotix/Simulateur` | ✅ Fait, submodule dans `PMX-CORTEX/simulator/` |
| Convention coord PMX (X horiz, Y vert, bottom-left) | ✅ Adaptée dans `visualisator.js` |
| Robot dessiné (cercle 28 cm, arrière segmenté) | ✅ Fait |
| Toggle zones + slider opacité | ✅ Fait |
| Toggle quadrillage 100 mm + labels + slider opacité | ✅ Fait |
| Retrait PAMI du simulateur | ✅ Fait |
| Renommage Princess/Big → PMX | ✅ Fait |
| `ZoneJsonExporter` (obstacles Playground + zones IA) | ✅ Fait |
| CLI export `/e /d` avec dry-run | ✅ Fait |
| `table.json` régénéré depuis C++ | ✅ Fait |
| Format JSON structuré (`type`/`subtype` + flags) spécifié | ✅ Fait |
| Simulateur parse le nouveau format + calcul auto position | ✅ Fait |
| Bouton Strat Jaune = miroir auto bleu → jaune | ✅ Fait |
| Filtre zones par couleur (Strat Bleu/Jaune) | ✅ Fait |
| `strategyPMX0.json` généré depuis C++ | ⬜ **TODO** |
| Runner C++ qui exécute un JSON | ⬜ **TODO** |
| Système de flags + préconditions côté runner | ⬜ **TODO** |

---

## 3. Format JSON stratégie

Spec complète : [STRATEGY_JSON_FORMAT.md](STRATEGY_JSON_FORMAT.md). Résumé :

### 3.1 Structure

Array d'**instructions**. Chaque instruction a une liste de **tasks** séquentielles.

```json
[
  {
    "id": 1,
    "desc": "Ramassage caisse centre",
    "tasks": [
      { "type": "MOVEMENT", "subtype": "PATH_TO", "position_x": 1500, "position_y": 1000 },
      { "type": "MOVEMENT", "subtype": "FACE_TO", "position_x": 1500, "position_y": 800 },
      { "type": "MANIPULATION", "action_id": "ouvrir_pinces", "timeout": 2000 },
      { "type": "MOVEMENT", "subtype": "LINE", "dist": 100 },
      { "type": "MANIPULATION", "action_id": "fermer_pinces", "timeout": 2000 },
      { "type": "MOVEMENT", "subtype": "LINE", "dist": -150 }
    ]
  }
]
```

### 3.2 Champs d'une task

- `type` : `MOVEMENT`, `MANIPULATION`, `ELEMENT`, `SPEED`, `WAIT`
- `subtype` : valeur dépendante du `type` (voir §3.3)
- Champs spécifiques selon (type, subtype) : `position_x/y`, `dist`, `angle_deg`, `action_id`, etc.
- `desc`, `timeout`, `needed_flag` : optionnels

### 3.3 Subtypes MOVEMENT supportés (26)

**Primitives (13)** — alignés Navigator C++ :
`LINE`, `GO_TO`, `GO_BACK_TO`, `MOVE_FORWARD_TO`, `MOVE_BACKWARD_TO`,
`ROTATE_DEG`, `ROTATE_ABS_DEG`, `FACE_TO`, `FACE_BACK_TO`, `ORBITAL_TURN_DEG`,
`PATH_TO`, `PATH_BACK_TO`, `MANUAL_PATH`.

**Composites (12)** — enchaînent déplacement + rotation avec abort si échec :
`GO_TO_AND_ROTATE_ABS_DEG`, `GO_TO_AND_ROTATE_REL_DEG`, `GO_TO_AND_FACE_TO`,
`GO_TO_AND_FACE_BACK_TO`, `MOVE_FORWARD_TO_AND_*` (4), `PATH_TO_AND_*` (4).

### 3.4 Autres types

| Type | Champs | Action C++ cible |
|---|---|---|
| `MANIPULATION` | `action_id`, `timeout` | `ActionRegistry::call(action_id)` |
| `ELEMENT`, `subtype: ADD_ZONE/DELETE_ZONE` | `item_id` | `Playground::enable(item_id, true/false)` |
| `SPEED`, `subtype: SET_SPEED` | `speed_percent` | `Asserv::setMaxSpeed(N, N)` |
| `WAIT` | `duration_ms` | `sleep_for(duration_ms)` |

### 3.5 Flags et préconditions (optionnels)

Sur une instruction :
- `needed_flag: "X"` → skip l'instruction si flag `X` non actif
- `action_flag: "Y"` → lève `Y` après succès
- `clear_flags: ["A", "B"]` → efface après succès

Sur une task :
- `needed_flag: "X"` → skip juste cette task

### 3.6 Position initiale

**Pas de task `start`** — la pose initiale vient de `initBig.json`. Chargée
au démarrage par le simulateur et par le runner (via `Asserv::setPosAndColor()`).

---

## 4. Intégration du pathfinding A*

Le pathfinding reste identique. Le JSON décrit les **destinations**, jamais les
waypoints intermédiaires.

```
JSON:   { "type": "MOVEMENT", "subtype": "PATH_TO",
          "position_x": 1500, "position_y": 1000 }
                      │
                      ▼
Runner: Navigator::pathTo(1500, 1000)
                      │
                      ▼
IAbyPath::playgroundFindPath()  ← A* existant, inchangé
                      │
                      ▼
Liste de waypoints → Asserv (moteurs)
```

Les obstacles utilisés par l'A* viennent du `Playground` C++. Les zones
`forbiddenZones` / `dynamicZones` du simulateur sont une **visualisation** de
ces mêmes obstacles.

Quand une task JSON dit `ELEMENT / DELETE_ZONE / item_id: "caisse_centre_1"` :
- **Runner C++** : `Playground::enable("caisse_centre_1", false)` → l'A* ne contourne plus la zone
- **Simu** : la zone est masquée sur le canvas (cohérence visuelle)

---

## 5. Miroir couleur (jaune)

Le simulateur applique la même règle miroir que l'Asserv C++ quand l'utilisateur
clique sur **Strat Jaune** :

| Champ | Conversion |
|---|---|
| `position_x`, `face_x`, `waypoints[*].x` | `3000 - x` |
| `position_y`, `face_y`, `waypoints[*].y` | inchangé |
| `angle_deg` pour `ROTATE_ABS_DEG` | `180 - angle` |
| `angle_deg` pour `ROTATE_DEG`, `ORBITAL_TURN_DEG` | `-angle` (sens inversé) |
| `final_angle_deg` | `180 - angle` |
| `rotate_rel_deg` | `-angle` |
| `turn_right` (ORBITAL_TURN_DEG) | `!turn_right` |
| `dist` (LINE) | inchangé (scalaire) |
| Pose initiale (`initBig.json`) | `x → 3000-x`, `theta → π - theta` |

Ce miroir est géré **entièrement en JS côté simulateur**
(fonction `mirrorStrategy()`). Côté C++, le miroir est géré dans l'Asserv
(`Robot::changeMatchX()`, `Robot::changeMatchAngleRad()`) — aucune duplication
dans le JSON.

Le simulateur filtre également les zones par couleur
(`type: "bleu"` / `"jaune"` / `"all"`) quand une strat colorée est chargée.

---

## 6. Architecture C++ du runner (à implémenter)

### 6.1 Fichiers à créer

| Fichier | Rôle |
|---|---|
| `robot/src/common/ia/StrategyJsonRunner.hpp/.cpp` | Charge le JSON, boucle + dispatch par `type`/`subtype` |
| `robot/src/common/ia/ActionRegistry.hpp/.cpp` | Map `action_id → std::function<bool()>` pour `MANIPULATION` |
| `robot/src/common/ia/FlagManager.hpp/.cpp` | `std::set<std::string>` + tests `needed_flag` |
| `robot/src/common/ia/StrategyJsonExporter.hpp/.cpp` | Export C++ → JSON (miroir de `ZoneJsonExporter`) |

### 6.2 Squelettes

```cpp
struct Task {
    std::string type;             // "MOVEMENT", "MANIPULATION", ...
    std::string subtype;          // "GO_TO", "LINE", ...
    std::optional<std::string> desc;
    std::optional<std::string> needed_flag;
    int timeout_ms = -1;

    // Champs selon (type, subtype) :
    std::optional<float> position_x, position_y;
    std::optional<float> dist;
    std::optional<float> angle_deg;
    std::optional<float> final_angle_deg, rotate_rel_deg;
    std::optional<float> face_x, face_y;
    std::optional<bool> forward, turn_right;
    std::optional<std::string> action_id;
    std::optional<std::string> item_id;
    std::optional<int> speed_percent;
    std::optional<int> duration_ms;
    std::vector<std::array<float,2>> waypoints;
};

struct Instruction {
    int id = 0;
    std::string desc;
    std::vector<Task> tasks;
    std::optional<std::string> needed_flag;
    std::optional<std::string> action_flag;
    std::vector<std::string> clear_flags;
    std::optional<int> points;
    std::optional<float> priority;
    std::optional<float> estimatedDurationSec;
};

class StrategyJsonRunner {
    Robot* robot_;
    Navigator* nav_;
    ActionRegistry* actions_;
    FlagManager* flags_;
    std::vector<Instruction> instructions_;

public:
    bool loadFromFile(const std::string& path);
    void run();                               // boucle d'exécution
    bool executeInstruction(const Instruction&);
    TRAJ_STATE executeTask(const Task&);      // dispatch sur (type, subtype)
};
```

### 6.3 Dispatch des tasks (cœur du runner)

```cpp
TRAJ_STATE StrategyJsonRunner::executeTask(const Task& t) {
    if (t.needed_flag && !flags_->has(*t.needed_flag)) return TRAJ_SKIPPED;

    if (t.type == "MOVEMENT") {
        if (t.subtype == "LINE")           return nav_->line(*t.dist);
        if (t.subtype == "GO_TO")          return nav_->goTo(*t.position_x, *t.position_y);
        if (t.subtype == "GO_BACK_TO")     return nav_->goBackTo(*t.position_x, *t.position_y);
        if (t.subtype == "MOVE_FORWARD_TO")  return nav_->moveForwardTo(*t.position_x, *t.position_y);
        if (t.subtype == "MOVE_BACKWARD_TO") return nav_->moveBackwardTo(*t.position_x, *t.position_y);
        if (t.subtype == "ROTATE_DEG")     return nav_->rotateDeg(*t.angle_deg);
        if (t.subtype == "ROTATE_ABS_DEG") return nav_->rotateAbsDeg(*t.angle_deg);
        if (t.subtype == "FACE_TO")        return nav_->faceTo(*t.position_x, *t.position_y);
        if (t.subtype == "FACE_BACK_TO")   return nav_->faceBackTo(*t.position_x, *t.position_y);
        if (t.subtype == "PATH_TO")        return nav_->pathTo(*t.position_x, *t.position_y);
        if (t.subtype == "PATH_BACK_TO")   return nav_->pathBackTo(*t.position_x, *t.position_y);
        // ... autres primitives + composites
    }
    else if (t.type == "MANIPULATION") {
        return actions_->call(*t.action_id) ? TRAJ_FINISHED : TRAJ_ERROR;
    }
    else if (t.type == "ELEMENT") {
        bool active = (t.subtype == "ADD_ZONE");
        robot_->playground().enable(*t.item_id, active);
        return TRAJ_FINISHED;
    }
    else if (t.type == "SPEED" && t.subtype == "SET_SPEED") {
        robot_->asserv().setMaxSpeed(*t.speed_percent, *t.speed_percent);
        return TRAJ_FINISHED;
    }
    else if (t.type == "WAIT") {
        std::this_thread::sleep_for(std::chrono::milliseconds(*t.duration_ms));
        return TRAJ_FINISHED;
    }
    return TRAJ_ERROR;
}
```

### 6.4 Intégration dans `O_State_DecisionMakerIA::execute()`

```cpp
void O_State_DecisionMakerIA::execute() {
    // ... init playground, setup zones ...

    if (!robot.strategyJsonPath().empty()) {
        StrategyJsonRunner runner(&robot, &robot.nav(), &robot.actions(), &robot.flags());
        runner.loadFromFile(robot.strategyJsonPath());
        runner.run();
    } else {
        // Ancien : exec hardcodé (temporaire)
        robot.ia().iAbyPath().ia_start();
    }
}
```

### 6.5 Enregistrement des actions

Dans `OPOS6UL_RobotExtended` ou équivalent :

```cpp
actions_.register("ouvrir_pinces", [this]() { return this->actions().openPinces(); });
actions_.register("fermer_pinces", [this]() { return this->actions().closePinces(); });
actions_.register("banderole",     [this]() { this->actions().ax12_GO_banderole(); return true; });
// ...
```

---

## 7. Workflow d'utilisation

### 7.1 Aujourd'hui

```bash
# 1. Éditer la stratégie JSON à la main (nouveau format)
vim /home/pmx/git/PMX-CORTEX/simulator/resources/2026/strategyPMX0.json

# 2. Régénérer les zones depuis C++ si besoin (BLEU par défaut, /y pour JAUNE)
cd build-simu-debug/bin && echo "m" | ./bot-opos6ul /e /d /k

# 3. Lancer le simulateur
cd /home/pmx/git/PMX-CORTEX/simulator && python3 -m http.server 8080
#    → http://localhost:8080
#    → cliquer "Zones" puis "Strat Bleu" ou "Strat Jaune"
```

### 7.2 Demain (runner JSON C++)

```bash
# Workflow 1 : édition humaine du JSON
vim simulator/resources/2026/strategyPMX0.json
./bot-opos6ul /s strategyPMX0.json    # charge et exécute ce JSON en match

# Workflow 2 : strat définie en C++, exportée vers JSON pour le simu
./bot-opos6ul /e /d /k                # exporte table.json + strategyPMX0.json (BLEU défaut)

# Workflow 3 : itération rapide en compétition
# 1. Un ops modifie strategyPMX0.json sur le robot
# 2. Relance ./bot-opos6ul /s strategyPMX0.json
# 3. Pas besoin de recompiler
```

---

## 8. Exemple de stratégie minimale

Voir [/home/pmx/git/PMX-CORTEX/simulator/resources/2026/strategyPMX0.json](../../simulator/resources/2026/strategyPMX0.json)
pour un exemple test complet (reste dans 700 mm, actions simulées, retour zone départ).

```json
[
  {
    "id": 1,
    "desc": "Sortie zone de départ",
    "tasks": [
      { "type": "ELEMENT", "subtype": "DELETE_ZONE", "item_id": "zone_start" },
      { "type": "MOVEMENT", "subtype": "GO_TO", "position_x": 500, "position_y": 500 }
    ]
  },
  {
    "id": 2,
    "desc": "Action simulée",
    "tasks": [
      { "type": "MOVEMENT", "subtype": "FACE_TO", "position_x": 700, "position_y": 500 },
      { "type": "MANIPULATION", "action_id": "ouvrir_pinces", "timeout": 2000 },
      { "type": "MOVEMENT", "subtype": "LINE", "dist": 100 },
      { "type": "MANIPULATION", "action_id": "fermer_pinces", "timeout": 2000 },
      { "type": "MOVEMENT", "subtype": "LINE", "dist": -100 }
    ]
  },
  {
    "id": 3,
    "desc": "Retour zone de départ",
    "tasks": [
      { "type": "MOVEMENT", "subtype": "PATH_TO", "position_x": 300, "position_y": 300 },
      { "type": "ELEMENT", "subtype": "ADD_ZONE", "item_id": "zone_start" }
    ]
  }
]
```

---

## 9. Plan d'implémentation (phases)

### Phase 1 — Parser JSON + squelette runner (≈ 150 lignes)

- Créer `StrategyJsonRunner.hpp/.cpp`
- Dépendance : `nlohmann/json` (déjà dispo dans `common/utils/json.hpp`)
- Implémenter `loadFromFile()`, `run()`, `executeInstruction()`, `executeTask()`
- Subtypes initiaux (couverture des plus utilisés) : `LINE`, `GO_TO`, `PATH_TO`, `FACE_TO`, `ROTATE_DEG`
- Types hors MOVEMENT : `MANIPULATION`, `ELEMENT`, `SPEED`, `WAIT`
- CLI `/s <path>` pour passer le JSON en argument

### Phase 2 — ActionRegistry (≈ 50 lignes)

- `std::unordered_map<std::string, std::function<bool()>>`
- Méthodes : `register(id, fn)`, `call(id)`, `has(id)`
- Enregistrer les actions existantes (ouvrir_pinces, fermer_pinces, banderole, ...)

### Phase 3 — FlagManager + préconditions (≈ 30 lignes)

- `std::set<std::string>` + `set`, `clear`, `has`
- Respect de `needed_flag` sur instruction et task
- Levée automatique de `action_flag` après succès d'une instruction
- Clear de `clear_flags` après succès

### Phase 4 — Subtypes restants (≈ 100 lignes)

- Ajouter les 8 primitives restantes (`GO_BACK_TO`, `MOVE_*`, `ROTATE_ABS_DEG`, `FACE_BACK_TO`, `ORBITAL_TURN_DEG`, `MANUAL_PATH`, `PATH_BACK_TO`)
- Ajouter les 12 composites

### Phase 5 — StrategyJsonExporter

- Écriture inverse : sérialise une stratégie C++ en JSON au nouveau format
- Permet de démarrer en C++ puis d'exporter pour visualiser dans le simu

### Phase 6 — Migration stratégie hardcodée

- Réécrire `O_State_DecisionMakerIA` pour qu'il charge le JSON au lieu d'appeler directement `IAbyPath::ia_start()`
- Un seul point d'entrée : le runner JSON

### Phase 7 — Couche décision (optionnelle, si besoin match)

- `DecisionManager::selectBestObjective()` pour scoring dynamique (points / durée / priorité / préconditions)
- Tracking `completed[]`, `skipped[]`
- Voir §11 pour les détails

---

## 10. Avantages vs risques

### Avantages

- **Itération compétition** : modifier stratégie = éditer fichier, pas recompiler
- **Visualisation** : même JSON lu par le simu et le robot réel
- **Source unique** : plus de dérive entre « simulé » et « exécuté »
- **Collaboration** : un non-codeur peut écrire une stratégie JSON
- **Debug** : chaque task a une `desc` lisible
- **Miroir auto** : strat écrite en bleu uniquement, le C++ et le simu gèrent le jaune

### Risques à surveiller

- **Parser JSON robuste** : une virgule manquante casse le match. Tester avant chaque compet
- **Performance** : parser le JSON 1× au démarrage, stocker en mémoire (pas par task)
- **Actions complexes** : certaines actions C++ prennent des paramètres. Si besoin, étendre
  `MANIPULATION` avec un champ `action_args: {...}`
- **Safety** : un JSON mal formé NE doit PAS laisser le robot dans un état incohérent.
  Fallback `freeMotion` en cas d'erreur
- **Divergence subtypes simulateur / runner** : le simu implémente un **sous-ensemble** des
  31 subtypes (tracé visuel uniquement). Le runner C++ devra les implémenter tous.
  Maintenir la table §3.3 à jour pour savoir qui supporte quoi

---

## 11. Couche décision (évolution future)

Le runner JSON décrit ci-dessus est une **couche d'exécution linéaire** : il lit les
instructions dans l'ordre et les exécute. C'est suffisant pour démarrer, mais ce n'est
pas « intelligent » — on veut une couche au-dessus qui **choisit** dynamiquement
l'instruction à jouer selon l'état du match.

### Les deux couches empilées

```
┌──────────────────────────────────────────────┐
│ COUCHE DÉCISION (intelligente)               │
│   - Liste d'INSTRUCTIONS scorées             │
│   - Chaque instruction : points, priority,   │
│     estimatedDurationSec, needed_flag        │
│   - selectBestInstruction() choisit l'ordre │
└───────────────────┬──────────────────────────┘
                    │ « joue l'instruction X »
                    ▼
┌──────────────────────────────────────────────┐
│ COUCHE EXÉCUTION (linéaire)                 │
│   - Lit les tasks de l'instruction X         │
│   - Dispatch par (type, subtype)             │
│   - Retourne succès/échec à la couche        │
│     au-dessus                                │
└──────────────────────────────────────────────┘
```

### Algorithme de décision

À chaque appel `selectBestInstruction()` :

1. **Filtre** : pour chaque instruction non complétée :
   - Vérifier `needed_flag` actif (si défini)
   - Vérifier que l'instruction n'est pas dans `completed[]` / `skipped[]`
   - *(Extensions PMX optionnelles — voir §12 de STRATEGY_JSON_FORMAT.md)* :
     `min_time_remaining_sec`, `max_time_remaining_sec`, `forbidden_flag`, `near_position`
2. **Score** : calcul d'un score pour chaque instruction restante :
   - Option simple : `priority` (plus élevé d'abord)
   - Option évoluée : `(points / estimatedDurationSec) * priority / distanceFromRobot`
3. **Choix** : renvoie l'instruction au meilleur score. Si aucune : match fini.

### Ce qu'il faut ajouter côté C++ (en plus du runner de §6)

| Fichier | Rôle |
|---|---|
| `robot/src/common/ia/DecisionManager.hpp/.cpp` | `selectBestInstruction()` + tracking |

### Coût / bénéfice

**Coût estimé** : 150-200 lignes de code, 1 journée d'intégration propre.

**Bénéfice** :
- Adaptation en compétition sans recompilation
- Gestion fine du temps restant (retour zone fin forcé à 80 s)
- Flags conditionnels pour sauter des instructions impossibles
- Nouveau collaborateur peut écrire une stratégie sans C++

**Quand passer à la couche décision** :
- Dès que la liste d'instructions du match devient > 5 (séquentiel figé devient un risque)
- Dès qu'il faut gérer le temps restant (retour zone fin automatique)
- Dès qu'il faut des instructions optionnelles selon caméra / état robot

**Quand ça ne vaut PAS le coup** :
- Stratégie de 2-3 actions basiques (homologation)
- Préconditions simples gérables en hardcodé C++

---

## 12. Références

- [STRATEGY_JSON_FORMAT.md](STRATEGY_JSON_FORMAT.md) — spécification détaillée du format
- [STRATEGY_RESEARCH.md](STRATEGY_RESEARCH.md) — recherche initiale sur les approches (Esial, nesnes, DIT-ROBOTICS, ...)
- [ARCHITECTURE.md](ARCHITECTURE.md) — archi globale PMX
- [EsialRobotik/Ia-Python](https://github.com/EsialRobotik/Ia-Python) — référence Python du format 2026
- [/home/pmx/git/PMX-CORTEX/simulator/javascript/visualisator.js](../../simulator/javascript/visualisator.js) — parser JS (`playSimulatorInstruction`, `computeTaskTarget`, `mirrorStrategy`)
- [/home/pmx/git/PMX-CORTEX/simulator/resources/2026/strategyPMX0.json](../../simulator/resources/2026/strategyPMX0.json) — exemple test courant
- [/home/pmx/git/PMX-CORTEX/robot/src/common/navigator/Navigator.cpp](../src/common/navigator/Navigator.cpp) — API Navigator (13 primitives + 12 composites)
- [/home/pmx/git/PMX-CORTEX/robot/src/common/ia/ZoneJsonExporter.cpp](../src/common/ia/ZoneJsonExporter.cpp) — export zones déjà en place
- [/home/pmx/git/PMX-CORTEX/robot/src/bot-opos6ul/O_State_DecisionMakerIA.cpp](../src/bot-opos6ul/O_State_DecisionMakerIA.cpp) — stratégie C++ actuelle (hardcodée)
