# Recherche : Stratégie de positionnement et IA de décision

> Document de réflexion — Analyse comparative des approches de stratégie pour la Coupe de France de Robotique 2026.

---

## 1. Analyse de notre système actuel (PMX-CORTEX)

### Architecture existante

Notre IA de décision est dans `O_State_DecisionMakerIA.cpp`. Elle fonctionne en **séquence linéaire** :

```
Init hardware → Setup playground (A*) → Setup zones/actions →
Attente chrono → Actions initiales hardcodées (banderole, déplacement) →
Boucle séquentielle des actions zone par zone → Fin de match
```

### Définition des zones

Les zones sont des structs C++ définis dans `IACommon.hpp` :

```cpp
typedef struct {
    char name[400];
    float minX, minY;       // Coin bas-gauche
    float width, height;    // Dimensions
    float startX, startY;   // Position d'entrée/travail
    float startAngle;       // Orientation (degrés)
} ZONE;
```

Les actions sont des **pointeurs de fonction** (`bool (*)()`) enregistrés avec `ia_addAction("nom", &callback)`.

### Forces actuelles

- Séparation propre zones (géométrie) / actions (comportement)
- Logique de retry configurable (RetryPolicy : noRetry, standard, aggressive, patient)
- Gestion des threads, singleton robot, logging SVG
- Pathfinding A* fonctionnel via `Navigator`

### Limitations actuelles

| Problème | Impact |
|---|---|
| Ordre d'exécution figé (séquentiel) | Pas d'adaptation dynamique |
| Pas de scoring par action | Impossible d'optimiser points/temps |
| Pas de prise en compte du temps restant | Risque de rater la zone de fin |
| Pas de fallback si action échoue | Blocage potentiel |
| Tout est compilé en dur | Modifier la stratégie = recompiler |
| Pas de condition d'exécution | Une action se lance même si inutile |

---

## 2. Approche EsialRobotik (JSON + moteur Python)

**Repos** : [Ia-Python](https://github.com/EsialRobotik/Ia-Python) / [Simulateur](https://github.com/EsialRobotik/Simulateur)

### Philosophie

Tout le comportement match est défini en **JSON**. Le code Python est un pur moteur d'exécution générique. Aucune modification de code source pour changer la stratégie.

### Structure du fichier strategy.json

```json
{
    "color0": [ ...objectifs pour côté Y=0... ],
    "color3000": [ ...objectifs pour côté Y=3000 (miroir)... ]
}
```

Chaque **objectif** :

```json
{
    "desc": "Ramassage caisses 3",
    "id": 1,
    "points": 17,
    "priority": 1,
    "tasks": [ ...étapes séquentielles... ],
    "needed_flag": null,
    "action_flag": "caisses3_done",
    "clear_flags": null
}
```

Chaque **étape** (task) :

```json
{
    "desc": "Position caisse 3",
    "position_x": 820,
    "position_y": 1150,
    "type": "MOVEMENT",
    "subtype": "GOTO_ASTAR",
    "action_id": null,
    "timeout": -1,
    "needed_flag": null
}
```

### Types d'étapes

| Type | Sous-types | Description |
|---|---|---|
| `MOVEMENT` | `GOTO`, `GOTO_ASTAR`, `GOTO_BACK`, `GOTO_CHAIN`, `GO`, `FACE`, `SET_SPEED`, `SET_POSITION` | Déplacements robot |
| `MANIPULATION` | (réf. fichier action JSON) | Commandes actionneurs |
| `ELEMENT` | `DELETE_ZONE`, `ADD_ZONE`, `RESET_FLAG`, `WAIT`, `WAIT_CHRONO` | État table/jeu |

### Système de flags

Mécanisme conditionnel simple à base de chaînes de caractères :

- Une action (ex: détection caméra) peut **lever un flag** (`"rotateNut1"`)
- Les étapes suivantes avec `"needed_flag": "rotateNut1"` ne s'exécutent **que si le flag est actif**
- Les objectifs peuvent aussi être conditionnés par des flags
- Permet l'adaptation dynamique sans state machine complexe

### Actions composées (fichiers JSON séparés)

```json
{
    "type": "list",
    "description": "Ressere, ouvre, baisse, ferme, releve",
    "payload": {
        "list": ["coller", "ouvrir", "poser", "fermer", "lever", "separer"]
    }
}
```

Types d'actions : `AX12`, `list` (séquentiel), `list_join` (parallèle), `camera_detect_aruco`, `pwm_servo`, `wait`

### Prise de décision

**Actuellement très simple** (leur propre TODO : "Devenir intelligent avec gestion des priorités") :

```python
def get_next_objective(self):
    while self.current_index < len(self.objectives):
        next = self.objectives[self.current_index]
        self.current_index += 1
        if next.needed_flag and next.needed_flag not in self.action_flags:
            continue  # Skip si flag requis absent
        return next
    return None
```

Parcours séquentiel des objectifs, skip si flag manquant. Les champs `priority` et `points` **existent dans le JSON mais ne sont pas utilisés** dans la logique.

### Simulateur web (PrincessViewer)

- Visualiseur 2D Canvas/JS pour tester les stratégies
- Affiche la table SVG, les robots, les trajectoires, les zones interdites
- Lecture pas-à-pas ou automatique
- Support WebSocket pour monitoring live en match
- Ressources par année (`resources/2025/table.svg`, `strategy*.json`, etc.)

### Points forts / Points faibles EsialRobotik

| + | - |
|---|---|
| Stratégie modifiable sans recompiler | Décision séquentielle (pas intelligente) |
| Format JSON lisible et partageable | Duplication complète pour chaque couleur |
| Simulateur web intégré | Pas d'optimisation points/temps |
| Système de flags simple mais efficace | Pas de gestion d'échec / fallback |
| Actions composables (list, list_join) | |

---

## 3. Autres approches d'équipes (état de l'art)

### A. Table de priorité scorée (le plus courant)

Utilisé par **nesnes/Eurobot-AI**, **Robot ESEO**, et la majorité des équipes compétitives.

```
Pour chaque action disponible :
    - Filtrer par préconditions (temps restant, stockage, disponibilité)
    - Calculer score = points / durée estimée (ou poids manuel)
    - Trier par score décroissant
    - Exécuter la meilleure action
    - Sur succès : marquer complétée, recommencer
    - Sur échec : marquer skippée, passer à la suivante
```

**nesnes/Eurobot-AI** (Node.js) :

```javascript
{
    name: "Grab Plants",
    condition: () => robot.armFree && match.timeRemaining > 15,
    actions: [ moveToElement(...), grabPlant(...) ],
    onError: [ moveToSafeZone() ]
}
```

### B. Behavior Trees (équipes ROS 2)

Utilisé par **DIT-ROBOTICS**, **Memristor (mep3)**.

- Arbres définis en XML, éditables avec **Groot2** (éditeur visuel)
- Nodes enregistrés via factory pattern en C++
- Memristor supporte le **live reload** : modifier le fichier XML recharge la stratégie à chaud
- Plus modulaire que les state machines, mais setup plus lourd

### C. Stratégie 3 phases (pattern récurrent)

Utilisé par plusieurs équipes top :

| Phase | Timing | Approche |
|---|---|---|
| **Rush** | 0-5s | Séquence rapide pré-programmée pour points garantis |
| **Dynamique** | 5-80s | Sélection intelligente dans la table d'actions |
| **Retour zone** | 80-90s | Retour en zone de fin pour bonus |

### D. Optimisation Excel (PoBot)

Outil de pré-sélection des actions à implémenter mécaniquement :

- Pour chaque type d'action : nombre d'items, points/item, difficulté (1-5), temps estimé
- Excel Solver pour maximiser score tout en minimisant temps et complexité
- Utilisé en phase de conception, pas en runtime

### E. GOAP (Goal-Oriented Action Planning)

Approche issue du jeu vidéo. Chaque action définit préconditions, effets sur le monde, et coût. Un planificateur A* trouve la séquence optimale. Puissant mais lourd à implémenter — probablement overkill pour Eurobot.

---

## 4. Outils et simulateurs

| Outil | Équipe | Type | Usage |
|---|---|---|---|
| **PrincessViewer** | EsialRobotik | Web Canvas/JS | Visualisation trajectoires, replay match |
| **Toolkit** | Robot ESEO | Qt/C++ | Simulateur complet avec hardware-in-the-loop |
| **Groot2** | BehaviorTree.CPP | Desktop | Éditeur visuel d'arbres de comportement |
| **Webots** | VRAC-team | 3D | Simulation physique de la table |
| **MQTT Dashboard** | nesnes | Web | Monitoring temps réel des objectifs |
| **EurobotCalculator** | ret7020 | React | Calcul de points |

---

## 5. Analyse pour PMX-CORTEX : Quick wins vs Restructuration

### Ce qu'on a déjà et qui fonctionne

- `Navigator` avec pathfinding A*, retry policies → **garder tel quel**
- Structure `ZONE` avec position d'entrée + angle → **base solide**
- `IAbyPath` avec boucle d'exécution → **à enrichir, pas à remplacer**
- Logging SVG des trajectoires → **garder**

### Quick Win 1 : Table de décision scorée (sans toucher à l'archi)

**Effort : faible | Impact : fort**

Enrichir la struct `ACTIONS` avec des métadonnées de décision :

```cpp
struct ActionEntry {
    std::string name;
    ZONE zone;                          // Zone associée
    int expectedPoints;                 // Points estimés
    float estimatedDurationSec;         // Durée estimée
    float priority;                     // Poids calculé ou manuel
    std::function<bool()> precondition; // Condition d'exécution
    RobotAction action;                 // Callback existant
    bool completed = false;
    bool skipped = false;
};
```

Modifier `ia_start()` pour trier par priorité au lieu d'exécuter séquentiellement :

```cpp
void ia_start() {
    while (hasRemainingActions()) {
        auto* best = selectBestAction();  // Tri par priority, filtré par precondition
        if (!best) break;
        bool ok = best->action();
        best->completed = ok;
        best->skipped = !ok;
    }
}

ActionEntry* selectBestAction() {
    ActionEntry* best = nullptr;
    for (auto& a : actions_) {
        if (a.completed || a.skipped) continue;
        if (a.precondition && !a.precondition()) continue;
        if (!best || a.priority > best->priority) best = &a;
    }
    return best;
}
```

### Quick Win 2 : Fichier JSON de stratégie (sans restructuration majeure)

**Effort : moyen | Impact : très fort en compétition**

Charger les zones et l'ordre des actions depuis un fichier JSON sur la carte SD :

```json
{
    "name": "strategy_2026_v3",
    "actions": [
        {
            "name": "banderole",
            "zone": { "x": 1300, "y": 400, "angle": 90 },
            "points": 20,
            "duration": 5.0,
            "priority": 100,
            "min_time_remaining": 5,
            "callback": "banderole"
        },
        {
            "name": "push_prise_bas",
            "zone": { "x": 775, "y": 550, "angle": -90 },
            "points": 15,
            "duration": 10.0,
            "priority": 50,
            "min_time_remaining": 20,
            "callback": "push_prise_bas"
        },
        {
            "name": "end_of_match",
            "zone": { "x": 350, "y": 1100, "angle": 90 },
            "points": 20,
            "duration": 8.0,
            "priority": 200,
            "min_time_remaining": 0,
            "callback": "end_of_match_top"
        }
    ]
}
```

Le code C++ reste le moteur d'exécution (les callbacks sont enregistrés par nom), mais l'**ordre, les priorités, les positions** viennent du JSON. Modifier la stratégie = éditer un fichier texte, pas recompiler.

### Quick Win 3 : Préconditions basées sur le temps

**Effort : très faible | Impact : moyen**

Ajouter une vérification du temps restant avant chaque action :

```cpp
if (robot_.chrono().getElapsedTimeSec() + action.estimatedDurationSec > 89) {
    // Pas assez de temps, skip
    action.skipped = true;
    continue;
}
```

Et forcer le retour en zone de fin quand le temps restant < durée estimée du retour.

### Quick Win 4 : Système de flags simple (inspiré EsialRobotik)

**Effort : faible | Impact : moyen**

```cpp
std::set<std::string> flags_;

void setFlag(const std::string& f) { flags_.insert(f); }
void clearFlag(const std::string& f) { flags_.erase(f); }
bool hasFlag(const std::string& f) { return flags_.count(f) > 0; }
```

Permet de conditionner les actions sans state machine complexe :
- Détection caméra → flag `"plantes_detectees"`
- Action de ramassage → `precondition: hasFlag("plantes_detectees")`

---

## 6. Recommandation : plan d'implémentation progressif

### Phase 1 — Enrichir la struct d'action (Quick Win 1 + 3)

- Ajouter `expectedPoints`, `estimatedDurationSec`, `priority`, `precondition` à `ACTIONS`
- Modifier `ia_start()` pour sélection par priorité + vérification temps
- **Ne casse rien de l'existant**, les actions actuelles continuent de fonctionner
- Testable immédiatement

### Phase 2 — Fichier de configuration JSON (Quick Win 2)

- Parser JSON (nlohmann/json déjà disponible ?)
- Charger positions des zones + ordre + priorités depuis fichier
- Les callbacks restent en C++ (table de correspondance nom → fonction)
- **Gain énorme en compétition** : modifier stratégie = éditer fichier + redémarrer

### Phase 3 — Flags et conditions (Quick Win 4)

- Ajouter le système de flags
- Conditions sur les actions
- Permet stratégies adaptatives simples

### Phase 4 — Visualiseur (si temps disponible)

- Page web simple affichant la table + positions des zones + ordre prévu
- Peut utiliser SVG existant comme base
- Permet de valider visuellement une stratégie avant de l'envoyer au robot

---

## 7. Ressources externes

| Ressource | Lien |
|---|---|
| EsialRobotik IA Python | https://github.com/EsialRobotik/Ia-Python |
| EsialRobotik Simulateur | https://github.com/EsialRobotik/Simulateur |
| nesnes Eurobot-AI (Node.js) | https://github.com/nesnes/Eurobot-AI |
| DIT-ROBOTICS Eurobot 2025 | https://github.com/DIT-ROBOTICS/Eurobot-2025-Main |
| Memristor mep3 (ROS 2) | https://github.com/memristor/mep3 |
| VRAC liste ressources Eurobot | https://github.com/VRAC-team/la-maxi-liste-ressources-eurobot |
| PoBot scoring optimization | https://pobot.org/Optimization-of-Eurobot-scoring.html |
| Robot ESEO simulateur | https://robot-eseo.fr/strategie-du-robot-sur-simulateur/ |
