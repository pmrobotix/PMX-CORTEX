# Roadmap d'implémentation — Runner JSON + stratégie PMX

> Plan pratique pour implémenter le runner de stratégie JSON et tester
> progressivement (simulateur → hardware → match).
>
> Docs associées :
> - [STRATEGY_JSON_EXECUTION.md](STRATEGY_JSON_EXECUTION.md) — architecture
> - [STRATEGY_JSON_FORMAT.md](STRATEGY_JSON_FORMAT.md) — référence du format JSON
> - [STRATEGY_RESEARCH.md](STRATEGY_RESEARCH.md) — recherche initiale

---

## Phase 0 — Finitions (0.5 jour) — avant d'attaquer le runner

Objectif : avoir une base propre et commitée avant d'ajouter le runner.

1. **Test visuel simulateur** — ouvrir http://localhost:8080 :
   - Vérifier orientation robot (theta=0 pointe où il faut)
   - Vérifier alignement des zones sur `table.png`
   - Tester toggle zones + slider opacité
   - Ajuster `toCanvasRotationDeg()` si le robot pointe au mauvais endroit
2. **Fix playground dimensions** — vérifier si `SymmetricalPlayground(0, 0, 3400, 2500, ...)`
   dans `OPOS6UL_IAExtended::initPlayground()` est intentionnel ou à aligner
   sur 3000×2000 (la bordure haut déborde actuellement à y=3871)
3. **Commit de tout ce qui est fait** : simulateur, submodule, ZoneJsonExporter,
   docs MD. Pas de push.

---

## Phase 1 — Parser JSON minimal (1 jour) — testable en SIMU

Objectif : `StrategyJsonRunner` qui lit un JSON plat et navigate correctement.

### Code à écrire

| Fichier | Rôle |
|---|---|
| `robot/src/common/ia/StrategyJsonRunner.hpp/.cpp` | Parser + boucle + dispatch |

### Commandes à implémenter (minimum)

- Navigation : `start`, `goto#x;y`, `goto-astar#x;y`, `face#x;y`, `go#dist`
- Timing : `speed#N`, `wait#ms`
- Zones : `delete-zone#id`, `add-zone#id`
- Action : `action#name` (stub qui log pour l'instant, plein support en Phase 2)

### CLI

Ajouter `/j <path>` dans `Robot::configureDefaultConsoleArgs()`.
Attention : `/s` existe déjà pour autre chose.

### Intégration

Dans `O_State_DecisionMakerIA::execute()` : si `/j` passé, runner JSON ;
sinon fallback hardcodé (pour ne rien casser).

### Test SIMU

```bash
# Écrire un JSON minimal avec 3-4 étapes goto
# Build
cmake --build build-simu-debug --target bot-opos6ul
# Run
cd build-simu-debug/bin
echo "m" | ./bot-opos6ul /j test_strategy.json /k
# (BLEU par défaut ; /y pour JAUNE)
# Vérifier les logs + svgAPF.svg
```

### Checkpoint Phase 1

✅ Le robot (en SIMU) suit la trajectoire du JSON, A* actif sur `goto-astar`,
les `delete-zone#` masquent bien les zones dans le pathfinding.

---

## Phase 2 — ActionRegistry (0.5 jour) — testable en SIMU

Objectif : les `action#name` du JSON déclenchent les vraies actions C++.

### Code à écrire

| Fichier | Rôle |
|---|---|
| `robot/src/common/ia/ActionRegistry.hpp/.cpp` | Map `name → function<bool()>` |

### API minimale

```cpp
class ActionRegistry {
public:
    void registerAction(const std::string& name, std::function<bool()> fn);
    bool call(const std::string& name);
    bool has(const std::string& name) const;
};
```

### Enregistrement des actions existantes

Dans `OPOS6UL_RobotExtended` (ou équivalent), enregistrer :
- `"banderole"` → `actions().ax12_GO_banderole()`
- `"ouvrir"`, `"fermer"` → méthodes pinces
- `"lever"`, `"poser"` → méthodes ascenseur
- etc. (selon ce qui existe)

### Intégration

Le runner appelle `registry.call(name)` quand il voit `action#name`.

### Test SIMU

- JSON avec `action#banderole` → vérifier que `ax12_GO_banderole()` est appelé
- Séquence de plusieurs actions → vérifier ordre dans logs

### Checkpoint Phase 2

✅ Le JSON déclenche les vrais callbacks C++ via `ActionRegistry`.

---

## Phase 3 — Migration de la stratégie actuelle vers JSON (0.5 jour)

Objectif : rejouer la stratégie actuelle (`O_end_of_match_top` + `O_push_prise_bas`)
depuis un JSON, sans régression.

### Travail à faire

1. Écrire `simulator/resources/2026/strategyPMX0.json` qui reproduit le
   comportement actuel de `O_State_DecisionMakerIA::execute()` :
   - Position initiale
   - Banderole (`action#banderole`)
   - Avance `go#150`
   - `goto-astar` vers zone prise bas
   - Push (`go#150` agressif)
   - `goto-astar` vers zone fin top
   - `wait-chrono#96` + `go#451`
2. Laisser le fallback hardcodé actif si pas de `/j` (sécurité).

### Test SIMU

- Run avec `/j strategyPMX0.json` → comparer SVG avec l'ancienne stratégie
- Run sans `/j` → comportement identique à avant

### Test RÉEL (après SIMU OK)

1. Vrai robot en mode test (sans tirette, `/k`) → vérifier moteurs, actionneurs
2. Vrai robot en match complet → comparer points à l'ancien

### Checkpoint Phase 3

✅ Le robot réel fait la même chose avec le JSON qu'avec le hardcodé.

---

## Phase 4 — Validation compet + homologation (1 jour)

Objectif : utiliser le runner JSON en conditions de compet.

### Code à écrire : 0

On n'écrit plus de C++, on ajuste le JSON.

### Tests

1. Simulation de match complet au quartier (chrono 90s)
2. Tests en jaune (vérifier que l'Asserv miroir fonctionne bien via JSON)
3. Tests avec adversaire simulé (obstacle pathfinding)
4. Itération rapide en compet : modifier `strategyPMX0.json`, redémarrer
   `bot-opos6ul /j ...`, pas de recompilation

### Checkpoint Phase 4

✅ Le robot tourne en match avec JSON + on peut modifier la stratégie sans
recompiler.

---

## Phase 5 — Évolutions sur demande (selon besoin réel)

À faire **uniquement si un besoin concret émerge** en compet ou en test.

| Besoin | Quoi ajouter | Effort |
|---|---|---|
| Objectifs + priorités | Format §4 de [STRATEGY_JSON_FORMAT.md](STRATEGY_JSON_FORMAT.md) + `DecisionManager` C++ | 1 jour |
| Flag `needed_flag` | `FlagManager` simple (std::set<string>) | 0.5 jour |
| Gestion temps restant | extension PMX `min_time_remaining_sec` | 0.5 jour |
| Caméra + flags auto | action `camera_detect_aruco` façon Esial | 1 jour |
| `set-flag#` explicite | extension PMX | 0.25 jour |
| Simulateur affiche jaune | Option B (sélecteur JS) ou A (script mirror) | 0.5 jour |
| Export strat C++ → JSON | `StrategyJsonExporter` (miroir de `ZoneJsonExporter`) | 1 jour |

---

## Planning total

| Phase | Effort | Cumul | Résultat |
|---|---|---|---|
| 0. Finitions | 0.5 j | 0.5 j | Base propre, commité |
| 1. Parser JSON + runner basique | 1 j | 1.5 j | JSON → nav A* en SIMU |
| 2. ActionRegistry | 0.5 j | 2 j | JSON → actionneurs en SIMU |
| 3. Migration strat actuelle | 0.5 j | 2.5 j | Même comportement, via JSON |
| 4. Tests réels + homologation | 1 j | 3.5 j | Validé hardware |
| 5. Évolutions (optionnel) | N jours | — | Sur demande |

---

## Stratégie de test progressive

À chaque phase, valider **dans cet ordre** :

1. **SIMU** (`build-simu-debug/bin/bot-opos6ul`) : logs + `svgAPF.svg`
2. **Mode test hardware** (vrai OPOS6UL mais sans match) : moteurs + servos répondent
3. **Mode match complet** : chrono + tirette + match 90s
4. **Itération JSON** (dès Phase 3) : modifier JSON sans recompiler

### Bonnes pratiques

- **Ne jamais sauter une phase** de test. SIMU avant réel, toujours.
- **Commit à chaque phase** : rollback facile si ça casse
- **Logs détaillés** dans le runner : chaque étape avec son résultat
- **Fallback hardcodé** maintenu tant que Phase 4 pas validée

---

## État actuel (2026-04-19)

| Phase | Statut |
|---|---|
| Phase 0 — Finitions | ⬜ À faire (commit) |
| Phase 1 — Parser + runner | ⬜ À faire |
| Phase 2 — ActionRegistry | ⬜ À faire |
| Phase 3 — Migration strat | ⬜ À faire |
| Phase 4 — Tests réels | ⬜ À faire |
| Phase 5 — Évolutions | ⬜ À faire selon besoin |

### Ce qui est déjà fait (pré-Phase 0)

- ✅ Fork simulateur `pmrobotix/Simulateur` (submodule)
- ✅ Convention coords PMX adaptée dans `visualisator.js`
- ✅ Robot dessiné (cercle 28 cm, arrière segmenté)
- ✅ Toggle zones + slider opacité dans le simu
- ✅ `ZoneJsonExporter` C++ → `table.json`
- ✅ CLI export `/e /d` avec dry-run
- ✅ Docs : `STRATEGY_RESEARCH.md`, `STRATEGY_JSON_EXECUTION.md`,
  `STRATEGY_JSON_FORMAT.md`
