# Évolution du runner — décision « skip + continue »

> Document de design. Décrit l'évolution du `StrategyJsonRunner` pour passer
> d'un comportement linéaire « abort au premier échec » à un comportement
> « skip l'instruction et continue ». Pré-requis pour l'homologation 2026 :
> permettre au robot d'enchaîner d'autres instructions si une est bloquée
> par un adversaire ou par un obstacle.
>
> Voir aussi :
> - [STRATEGY_JSON_EXECUTION.md](STRATEGY_JSON_EXECUTION.md) — architecture du runner
> - [STRATEGY_JSON_FORMAT.md](STRATEGY_JSON_FORMAT.md) — format du JSON
> - [STRATEGY_JSON_ROADMAP.md](STRATEGY_JSON_ROADMAP.md) — feuille de route phases

---

## 1. Constat (Phase 1 actuelle)

Comportement aujourd'hui dans
[StrategyJsonRunner.cpp:41-52](../src/common/ia/StrategyJsonRunner.cpp#L41-L52) :

```cpp
for (const auto& instr : instructions_) {
    if (!executeInstruction(instr)) {
        logger().error() << "run: instruction id=" << instr.id << " FAILED, abort";
        return false;
    }
}
```

Et dans
[StrategyJsonRunner.cpp:106-112](../src/common/ia/StrategyJsonRunner.cpp#L106-L112) :

```cpp
TRAJ_STATE ts = executeTask(task);
if (ts != TRAJ_FINISHED) {
    logger().error() << "[instr " << instr.id << "] task ... -> ts=" << ts;
    return false;
}
```

Conséquence : si un adversaire bloque une instruction (Navigator a déjà
épuisé ses `maxObstacleRetries`), tout le run est arrêté. Démontré par le
scénario **SR06** dans
[O_StrategyJsonRunnerTest.cpp:101-109](../src/bot-test-opos6ul/O_StrategyJsonRunnerTest.cpp#L101-L109)
(`expectedRunCompleted = false`).

C'est inacceptable en match : si une instruction n'est pas jouable, le
robot doit passer à la suivante, pas s'arrêter.

---

## 2. Objectif

Faire évoluer `run()` pour :

- continuer la boucle même si une instruction renvoie `TRAJ != TRAJ_FINISHED`,
- garder une trace des instructions skippées,
- ne pas dérouler les `action_flag` / `clear_flags` d'une instruction
  qui n'a pas réussi.

**Hors scope (v2)** : reprise différée des instructions skippées (boucle
multi-passes), gestion du temps restant, scoring `points / estimatedDurationSec`.
Ces éléments sont décrits §8 (« v3 future »).

---

## 3. Sémantique des `TRAJ_STATE` retournés à `executeInstruction`

Rappel : Navigator a **déjà** consommé ses retries sur la trajectoire
elle-même (`policy.maxObstacleRetries`, recul, reset). Quand on remonte un
`TRAJ != FINISHED` au runner, c'est que ces retries n'ont rien donné : la
trajectoire est définitivement bloquée à cet instant.

| Code retour | Cause typique | Décision runner v2 |
|---|---|---|
| `TRAJ_FINISHED` | Trajectoire OK | Lève `action_flag`, clear `clear_flags`, instruction suivante |
| `TRAJ_NEAR_OBSTACLE` | Adversaire bloque (capteurs ToF/balise), retries épuisés | **Skip instruction** → `SKIPPED_OBSTACLE`, instruction suivante |
| `TRAJ_COLLISION` | Collision physique non ignorée | **Skip instruction** → `SKIPPED_COLLISION`, instruction suivante |
| `TRAJ_IMPOSSIBLE` | Pathfinding A\* introuvable, ou cmd refusée | **Skip instruction** → `SKIPPED_IMPOSSIBLE`, instruction suivante |
| `TRAJ_ERROR` | Timeout cmd Nucleo, ack manquant | **Skip instruction** → `SKIPPED_ERROR`, instruction suivante |

> Décision : **aucun de ces codes ne provoque l'abort du run en v2**. Seules
> exceptions possibles d'un crash global : exception C++ non rattrapée. Le
> runner n'arrête le run de lui-même que quand toutes les instructions
> ont été tentées (succès, skip, ou skip pour `needed_flag` absent /
> `priority<0` / `max_match_sec` dépassé).

> **Important** : après un skip sur erreur trajectoire, le runner appelle
> `robot_->asserv().resetEmergencyOnTraj("StrategyJsonRunner skip
> instruction")` avant de passer à l'instruction suivante. Sans ce reset,
> la Nucleo resterait en `emergencyStop_` (Navigator ne reset pas sur le
> dernier essai, volontairement, pour laisser la stratégie décider) et la
> commande suivante échouerait immédiatement. Cf
> [Navigator.cpp:96-98](../src/common/navigator/Navigator.cpp#L96-L98).

### 3.1 Effets de bord côté flags

Une instruction skippée :
- ne lève **pas** son `action_flag`,
- ne fait **pas** ses `clear_flags`.

Cela permet à une instruction suivante avec `needed_flag: "i2_done"` d'être
elle-même skippée logiquement (et marquée `skipped_needed_flag_<id>`).

---

## 4. Variables disponibles côté JSON

Aucune nouvelle clé JSON n'est ajoutée en v2. On réutilise ce qui existe
déjà ([StrategyJsonParser.hpp:48-62](../src/common/ia/StrategyJsonParser.hpp#L48-L62)).

### 4.1 Sur une **instruction**

| Variable | Type | Effet runtime v2 | Origine |
|---|---|---|---|
| `id` | int | Logs, traçage des skipped | obligatoire de fait |
| `desc` | string | Logs, debug simu | recommandé |
| `tasks` | array | Liste ordonnée des tasks | **obligatoire** |
| `priority` | float | Tri desc au parsing (stable). `<0` = désactivée (skip silencieux). | défaut `0` |
| `points` | int? | **Non utilisé runtime v2** (réservé pour scoring v3) | optionnel |
| `estimatedDurationSec` | float? | **Non utilisé runtime v2** (réservé pour gestion temps v3) | optionnel |
| `needed_flag` | string? | Skip si flag non levé. Marqué `skipped_needed_flag_<id>`. | optionnel |
| `action_flag` | string? | Levé **uniquement si toutes les tasks réussissent** | optionnel |
| `clear_flags` | array<string\> | Effacés **uniquement si toutes les tasks réussissent** | optionnel |

### 4.2 Sur une **task**

Identique à [STRATEGY_JSON_FORMAT.md §2.3](STRATEGY_JSON_FORMAT.md). Pas
de changement. Une task avec `needed_flag` non levé est skippée (la suite
de l'instruction continue), comportement déjà en place dans
[StrategyJsonRunner.cpp:71-76](../src/common/ia/StrategyJsonRunner.cpp#L71-L76).

### 4.3 Conventions d'usage pour rédaction d'une stratégie

| Cas d'usage | Convention |
|---|---|
| Action critique à jouer en premier | `priority` élevée (ex: `100`) |
| Action de points secondaire | `priority` moyenne (ex: `50`) |
| Action de repli alternative à une autre | `priority` un peu plus basse (ex: `40`) + `needed_flag` antagoniste si exclusivité voulue |
| Retour zone fin | `priority` très basse (ex: `1`) — joué en dernier après toutes les actions de points |
| Désactiver temporairement une instruction | `priority: -1` |
| Instruction conditionnelle à une réussite précédente | `needed_flag: "X_done"` + l'instruction qui réussit a `action_flag: "X_done"` |
| Empêcher 2 actions exclusives (i2 OU i3) | i2 met `action_flag: "i2_done"`, i3 a `needed_flag: "NEVER"` ou utilise un mécanisme de mutex (TODO si besoin) |

> **Note importante** : `points`, `estimatedDurationSec`, `priority` sont
> tous trois lus par le parser dès aujourd'hui. Seul `priority` a un effet
> runtime en v2 (tri + désactivation). Les deux autres sont là pour la v3
> future, mais on peut **déjà les renseigner** dans le JSON pour préparer
> le terrain — ils seront ignorés sans erreur.

---

## 5. Implémentation côté runner

### 5.1 Nouveaux champs dans `StrategyJsonRunner`

Dans [StrategyJsonRunner.hpp](../src/common/ia/StrategyJsonRunner.hpp),
section `private` :

```cpp
struct InstructionOutcome {
    int id;
    enum class Status {
        FINISHED,           // OK, flags levés
        SKIPPED_PRIORITY,   // priority < 0
        SKIPPED_NEEDED_FLAG,
        SKIPPED_OBSTACLE,   // TRAJ_NEAR_OBSTACLE
        SKIPPED_COLLISION,  // TRAJ_COLLISION
        SKIPPED_IMPOSSIBLE, // TRAJ_IMPOSSIBLE
        SKIPPED_ERROR       // TRAJ_ERROR
    };
    Status status;
    TRAJ_STATE lastTs = TRAJ_IDLE;  // si SKIPPED_* trajectoire
};

std::vector<InstructionOutcome> outcomes_;  // chronologique
```

Méthodes utiles à exposer :

```cpp
const std::vector<InstructionOutcome>& outcomes() const { return outcomes_; }
size_t countByStatus(InstructionOutcome::Status s) const;
```

### 5.2 Boucle `run()` v2

```cpp
bool StrategyJsonRunner::run()
{
    outcomes_.clear();
    logger().info() << "run: start (" << instructions_.size() << " instructions)";

    for (const auto& instr : instructions_) {
        InstructionOutcome o;
        o.id = instr.id;

        // priority < 0 : désactivée
        if (instr.priority < 0.0f) {
            o.status = InstructionOutcome::Status::SKIPPED_PRIORITY;
            outcomes_.push_back(o);
            logger().info() << "[instr " << instr.id << "] SKIP priority<0";
            continue;
        }

        // needed_flag absent : skip silencieux
        if (flags_ && instr.needed_flag && !flags_->has(*instr.needed_flag)) {
            o.status = InstructionOutcome::Status::SKIPPED_NEEDED_FLAG;
            outcomes_.push_back(o);
            logger().info() << "[instr " << instr.id << "] SKIP needed_flag '"
                            << *instr.needed_flag << "' absent";
            continue;
        }

        TRAJ_STATE ts = executeInstructionV2(instr);  // ne touche pas aux flags
        if (ts == TRAJ_FINISHED) {
            // Lever flags maintenant que tout l'enchaînement est OK
            if (flags_) {
                if (instr.action_flag) flags_->set(*instr.action_flag);
                for (const auto& f : instr.clear_flags) flags_->clear(f);
            }
            o.status = InstructionOutcome::Status::FINISHED;
            outcomes_.push_back(o);
            continue;
        }

        // Skip selon le code de retour
        o.lastTs = ts;
        switch (ts) {
            case TRAJ_NEAR_OBSTACLE:
                o.status = InstructionOutcome::Status::SKIPPED_OBSTACLE; break;
            case TRAJ_COLLISION:
                o.status = InstructionOutcome::Status::SKIPPED_COLLISION; break;
            case TRAJ_IMPOSSIBLE:
                o.status = InstructionOutcome::Status::SKIPPED_IMPOSSIBLE; break;
            default: // TRAJ_ERROR ou autre
                o.status = InstructionOutcome::Status::SKIPPED_ERROR; break;
        }
        outcomes_.push_back(o);
        logger().warn() << "[instr " << instr.id << "] SKIP ts=" << ts
                        << " -> instruction suivante";
    }

    size_t finished = countByStatus(InstructionOutcome::Status::FINISHED);
    logger().info() << "run: done. finished=" << finished
                    << "/" << instructions_.size();
    return true;  // run() v2 ne retourne false que sur erreur infra (load)
}
```

> **Note** : `run()` v2 retourne **toujours `true`** une fois la boucle
> consommée. La distinction « toutes OK » vs « certaines skippées » se
> lit via `outcomes()` ou `countByStatus()`. C'est la couche appelante
> (`O_State_DecisionMakerIA`) qui décide quoi faire (retour zone fin,
> log compétition, etc.).

### 5.3 `executeInstructionV2` : retourne `TRAJ_STATE` au lieu de `bool`

Refactor de l'actuel `executeInstruction` :

```cpp
TRAJ_STATE StrategyJsonRunner::executeInstructionV2(const StrategyInstruction& instr)
{
    logger().info() << "[instr " << instr.id << "] " << instr.desc;
    bool firstTask = true;
    bool prevChain = false;

    for (const auto& task : instr.tasks) {
        // skip task selon needed_flag (inchangé)
        if (flags_ && task.needed_flag && !flags_->has(*task.needed_flag)) {
            continue;
        }

        if (!firstTask) {
            const int sleep_us = (prevChain && task.chain) ? 20000 : 50000;
            utils::sleep_for_micros(sleep_us);
        }
        firstTask = false;

        if (task.chain) {
            if (!executeTaskSendOnly(task)) {
                return TRAJ_ERROR;  // erreur send chain → remonte ts
            }
            prevChain = true;
            continue;
        }

        TRAJ_STATE ts = executeTask(task);
        if (ts != TRAJ_FINISHED) {
            return ts;  // remonte le ts brut au caller (run)
        }
        prevChain = false;
    }

    // wait final chain (inchangé, mais remonte ts au lieu de bool)
    if (prevChain) {
        const StrategyTask& last = instr.tasks.back();
        Asserv::MovementType type = ...;  // idem code actuel
        TRAJ_STATE ts = robot_->asserv().waitEndOfTrajWithDetection(type);
        if (ts != TRAJ_FINISHED) return ts;
    }

    return TRAJ_FINISHED;
}
```

L'ancien `executeInstruction(bool)` peut être supprimé OU conservé en
wrapper :
```cpp
bool executeInstruction(const StrategyInstruction& i) {
    return executeInstructionV2(i) == TRAJ_FINISHED;
}
```
(uniquement si `runTasks` au-dessus en a besoin — voir 5.4).

### 5.4 `runTasks()` (utilisé pour `setpos_tasks`)

`runTasks` enveloppe une liste de tasks dans une pseudo-instruction et
appelle `executeInstruction`. Pour `setpos_tasks` (séquence avant tirette)
on **veut** l'ancien comportement « abort si échec » : un échec à
l'init avant tirette doit remonter pour que `O_State_NewInit` puisse
réagir. → garder `executeInstruction` retournant `bool` comme façade
appelant `executeInstructionV2`, et `runTasks` continue à l'utiliser tel
quel.

---

## 6. Impact sur le test SR06

[O_StrategyJsonRunnerTest.cpp:101-109](../src/bot-test-opos6ul/O_StrategyJsonRunnerTest.cpp#L101-L109)
définit aujourd'hui :

```cpp
{
    "SR06", "AdvBlocksInstruction : adv bloque i2.GO_TO, run() abort, i3 non atteinte",
    "strategySR06.json",
    300, 300, 0,
    true, 2000, 900,
    {"trace_i1"},     // expectedTrace
    {"a_done"},       // expectedFlags
    false             // expectedRunCompleted
}
```

Avec la v2, le comportement attendu devient :
- i1 OK → `trace_i1`, `a_done` levé
- i2 GO_TO bloquée par adv → SKIP
- i3 GO_TO vers (300, 1800) → libre, OK → `trace_i3`

Donc `expectedTrace = {"trace_i1", "trace_i3"}`, `expectedFlags = {"a_done"}`,
`expectedRunCompleted = true`.

### 6.1 Décision retenue — option A (SR06 adapté)

SR06 a été adapté en place. Pas de SR07 ajouté. Le scénario démontre
maintenant que **i2 est skippée et le run continue**, et la trace
finale est `{trace_i1}` (i3 ne réussit pas dans ce setup figé : l'adv
reste injecté en (2300, 1100) tout le scénario, donc i3 est aussi
bloquée par détection cône front). Ce qui distingue v2 de v1, ce n'est
plus la trace mais **la séquence d'outcomes** :
- v1 : `[FINISHED]` (abort après i2)
- v2 : `[FINISHED, SKIPPED_OBSTACLE, SKIPPED_OBSTACLE]`

### 6.2 Extension du test — vérification d'`outcomes()`

Pour exposer cette différence proprement, le test compare désormais
trois choses au lieu de deux :

| Comparaison | Avant v2 | Après v2 |
|---|---|---|
| `trace` (MANIPULATION appelées) | ✅ | ✅ |
| `finalFlags` (Set d'actifs) | ✅ | ✅ |
| `outcomeStatuses` (vector status par instruction) | — | ✅ **nouveau** |

Chaque scénario `SR01..SR06` fournit son `expectedOutcomeStatuses`. Le
test échoue si l'un des trois ne matche pas. Bilan actuel : **6/6 PASS**.

> Note : la position adversaire de SR06 a aussi été déplacée de
> (2000, 900) vers (2300, 1100) pour bloquer i2 sans avoir à se
> contorsionner — le comportement testé reste identique.

### 6.3 Tableau récap des 6 scénarios SR

CLI : `./bot-opos6ul jrun`. Les fichiers JSON sont dans
[robot/src/bot-test-opos6ul/scenarios/](../src/bot-test-opos6ul/scenarios/).
Setup commun : start `(300, 300, 0°)` sauf SR07 `(1500, 1500, 0°)`. Sensors
front **et** back centre actifs (front/back G+D ignorés, capteurs ToF latéraux
pas qualifiés). Aligné sur le setup match
([O_State_DecisionMakerIA.cpp](../src/bot-opos6ul/O_State_DecisionMakerIA.cpp)).

| Scénario | Ce qu'il vérifie | JSON instructions | Adv | Trace attendue | Outcomes attendus |
|---|---|---|---|---|---|
| **SR01** | Tri par `priority` desc (parser + run order). Pas de flags. | i1 p=100, i2 p=10, i3 p=50 → joué dans l'ordre i1, i3, i2 | — | `[trace_i1, trace_i3, trace_i2]` | `[OK, OK, OK]` |
| **SR02** | `action_flag` (i1 lève A) + `needed_flag` au niveau instruction (i2 needs A → OK ; i3 needs NEVER → skip). | i1 p=100 action_flag=A, i2 p=50 needed_flag=A, i3 p=10 needed_flag=NEVER | — | `[trace_i1, trace_i2]` | `[OK, OK, SK_FLAG]` |
| **SR03** | `priority<0` désactive l'instruction. Tri stable_sort + skip silencieux. | i1 p=-1, i2 p=10 → joué i2 puis i1 marqué SK_PRIO | — | `[trace_i2]` | `[OK, SK_PRIO]` |
| **SR04** | `needed_flag` au niveau **task** (pas instruction) : task interne skippée, instruction reste OK. | 1 instruction avec 4 tasks dont la 2e a needed_flag=NEVER | — | `[trace_t1, trace_t3]` | `[OK]` |
| **SR05** | `clear_flags` : i1 lève A, i2 clear A, donc i3 needed=A skippée. Démontre que les flags d'instruction sont levés/effacés **après** succès complet. | i1 action_flag=A, i2 clear_flags=[A], i3 needed_flag=A | — | `[trace_i1, trace_i2]` | `[OK, OK, SK_FLAG]` |
| **SR06** | **Détection FRONT** : adv bloque i2 ET i3 (cône front). 3 outcomes complets, pas d'abort. | i1 (1500,400), i2 (2500,1400), i3 (300,300) | ✓ (2300, 1100) | `[trace_i1]` | `[OK, SK_OBST, SK_OBST]` |
| **SR07** | **Détection BACK** : adv pile derrière bloque `LINE -300`. Valide que `filtre_levelInBack` retourne -4 et déclenche `setEmergencyStop()` en `BACKWARD`. i2 GO_TO ailleurs, libre. | i1 LINE -300, i2 GO_TO (1500,800) | ✓ (1300,1500) (200mm derrière) | `[trace_i2]` | `[SK_OBST, OK]` |
| **SR08** | `max_match_sec` : i2 a `max_match_sec=1`, joué après i1 (~2s) → skip. i3 a `max_match_sec=999` → OK. Couvre `SK_TIME`. | i1 OK, i2 max=1, i3 max=999 | — | `[trace_i1, trace_i3]` | `[OK, SK_TIME, OK]` |
| **SR09** | Pathfinding A\* impossible : i2 `PATH_TO (50,50)` dans bordure → A\* renvoie chemin vide → `TRAJ_IMPOSSIBLE` → `SK_IMPS`. Couvre `SK_IMPS`. | i1 OK, i2 PATH_TO (50,50), i3 OK | — | `[trace_i1, trace_i3]` | `[OK, SK_IMPS, OK]` |

Légende des statuts (cf [O_StrategyJsonRunnerTest.cpp `statusName()`](../src/bot-test-opos6ul/O_StrategyJsonRunnerTest.cpp)) :

| Code | Status enum | Cause |
|---|---|---|
| `OK` | `FINISHED` | toutes les tasks OK, flags levés/effacés |
| `SK_PRIO` | `SKIPPED_PRIORITY` | `priority < 0` |
| `SK_FLAG` | `SKIPPED_NEEDED_FLAG` | `needed_flag` instruction non levé |
| `SK_TIME` | `SKIPPED_MAX_TIME` | `max_match_sec` dépassé |
| `SK_OBST` | `SKIPPED_OBSTACLE` | `TRAJ_NEAR_OBSTACLE` retries épuisés |
| `SK_COLL` | `SKIPPED_COLLISION` | `TRAJ_COLLISION` |
| `SK_IMPS` | `SKIPPED_IMPOSSIBLE` | `TRAJ_IMPOSSIBLE` (A\* introuvable) |
| `SK_ERR` | `SKIPPED_ERROR` | `TRAJ_ERROR` (timeout cmd, ack manquant) |

Chaque exécution écrit un SVG de synthèse `test_strategy_runner.svg` à
côté du binaire (grille 2×3, une cellule par scénario, attendus vs réels
en vert/rouge).

> **Note couverture** : `SK_TIME` couvert par SR08, `SK_IMPS` par SR09,
> `SK_OBST` par SR06 (front) **et SR07 (back)**.
> Restent non couverts : `SK_COLL` (collision détectée par capteurs ToF
> latéraux, désactivés en match) et `SK_ERR` (timeout cmd asserv, difficile
> à reproduire en simu).

---

## 7. Stratégie d'homologation 2026

L'objectif d'homologation est qu'un robot fasse la démonstration de
règles minimales. Doit pouvoir tourner même si un adversaire est posé
sur la table. Donc :

- au moins une **action de points** alternative (en cas de blocage),
- un **retour zone de fin** garanti,
- des **flags d'exclusivité** entre les alternatives pour ne pas tenter
  les deux si la première a réussi.

### 7.1 Variables JSON à utiliser

Pour rédiger ce JSON, voir §4.1 et §4.2. En particulier :

- `priority` : ordonne les tentatives. La plus haute = essayée d'abord.
- `needed_flag` / `action_flag` / `clear_flags` : exclusivité,
  conditionnement.
- `points`, `estimatedDurationSec` : à renseigner dès aujourd'hui (utilisés
  par la couche décision v3, ignorés en v2).

### 7.2 Squelette à compléter

Fichier cible : `simulator/resources/2026/strategyHOMOLOG.json`.
Le JSON exact sera fourni par l'utilisateur ; ce squelette n'est qu'un
**modèle pédagogique** :

```jsonc
[
  {
    "id": 1, "desc": "Sortie zone départ",
    "priority": 200,
    "tasks": [
      { "type": "ELEMENT", "subtype": "DELETE_ZONE", "item_id": "depart_bleu" },
      { "type": "MOVEMENT", "subtype": "GO_TO", "position_x": 500, "position_y": 500 }
    ]
  },
  {
    "id": 2, "desc": "Action de points principale",
    "priority": 100,
    "points": 10,
    "estimatedDurationSec": 12,
    "action_flag": "points_done",
    "tasks": [
      { "type": "MOVEMENT", "subtype": "GO_TO", "position_x": 1500, "position_y": 1000 },
      { "type": "MANIPULATION", "action_id": "ouvrir_pinces" }
    ]
  },
  {
    "id": 3, "desc": "Action de repli (jouée si points_done absent)",
    "priority": 90,
    "points": 5,
    "estimatedDurationSec": 8,
    "action_flag": "points_done",
    "needed_flag_NOT": "points_done",  // ← n'existe pas encore — voir §7.3
    "tasks": [
      { "type": "MOVEMENT", "subtype": "GO_TO", "position_x": 800, "position_y": 1500 }
    ]
  },
  {
    "id": 99, "desc": "Retour zone fin",
    "priority": 1,
    "points": 0,
    "estimatedDurationSec": 6,
    "tasks": [
      { "type": "MOVEMENT", "subtype": "PATH_TO", "position_x": 300, "position_y": 1800 }
    ]
  }
]
```

### 7.3 Exclusivité d'alternatives — pattern flags (calé sur EsialRobotik)

Pas besoin d'extension `forbidden_flag`. EsialRobotik fait toute leur
décision dynamique avec uniquement `needed_flag` + `action_flag` +
`clear_flags`. On reprend le même pattern.

**Cas type** : i2 et i3 sont deux façons d'obtenir les mêmes points. On
veut jouer i3 **seulement si i2 a échoué** (typiquement bloquée par
adv).

```jsonc
[
  {
    "id": 0, "desc": "Init flags",
    "priority": 999,
    "action_flag": "zone_points_libre",
    "tasks": [ /* ou aucune task = juste lever le flag au démarrage */ ]
  },
  {
    "id": 2, "desc": "Action principale",
    "priority": 100,
    "clear_flags": ["zone_points_libre"],
    "tasks": [
      { "type": "MOVEMENT", "subtype": "GO_TO", "position_x": 1500, "position_y": 1000 },
      { "type": "MANIPULATION", "action_id": "ouvrir_pinces" }
    ]
  },
  {
    "id": 3, "desc": "Action de repli (si i2 a échoué)",
    "priority": 90,
    "needed_flag": "zone_points_libre",
    "tasks": [
      { "type": "MOVEMENT", "subtype": "GO_TO", "position_x": 800, "position_y": 1500 }
    ]
  }
]
```

**Mécanique** :
- i0 lève `zone_points_libre` au démarrage.
- i2 efface le flag **uniquement si elle réussit** (rappel §3.1 : flags
  d'instruction levés/effacés seulement si l'instruction complète OK).
- Si i2 réussit → i3 voit `needed_flag` absent → skippée.
- Si i2 échoue (skip obstacle) → flags inchangés → i3 voit le flag
  toujours levé → joue son alternative.

**Variante sans i0 d'init** : si l'absence de flag par défaut suffit (ex:
i3 doit jouer **par défaut** sauf si i2 réussit), on inverse :

```jsonc
{
  "id": 2, "desc": "Action principale",
  "priority": 100,
  "action_flag": "fait",
  "tasks": [...]
},
{
  "id": 3, "desc": "Repli si i2 KO",
  "priority": 90,
  "needed_flag": "PAS_fait",   // jamais levé → toujours candidat
  "tasks": [...]
}
```

Mais c'est sale (sémantique « PAS_fait » non lisible). Préférer le
pattern à 3 instructions ci-dessus.

**Décision pour l'homologation** : utiliser ce pattern dès le début. Pas
d'extension de format à attendre.

---

## 8. v3 future — couche décision

Hors scope de cette évolution mais documenté ici pour cadrer.

### 8.1 Reprise différée des skippées

Aujourd'hui v2 : passe linéaire unique. Une instruction skippée pour
obstacle est définitivement perdue.

v3 : après la passe principale, si `outcomes()` contient des
`SKIPPED_OBSTACLE` et qu'il reste du temps, faire une 2e passe sur ces
seules instructions. L'adversaire a peut-être bougé.

Implémentation pressentie : `int maxRetryPasses_ = 2` (configurable),
boucle externe sur les passes, condition d'arrêt = aucun nouveau
`FINISHED` dans la dernière passe OU `maxRetryPasses_` atteint.

### 8.2 Gestion du temps restant

Utilisation de `estimatedDurationSec` :
- avant de lancer une instruction, vérifier qu'il reste assez de temps,
- forcer le retour zone fin à `T - margin` secondes restantes,
  indépendamment de la `priority`.

### 8.3 Scoring dynamique

`selectBestInstruction()` qui choisit non plus par `priority` figé mais
par `(points / estimatedDurationSec) * priority / distanceFromRobot`,
voir §11 de [STRATEGY_JSON_EXECUTION.md](STRATEGY_JSON_EXECUTION.md#11-couche-décision-évolution-future).

### 8.4 Préconditions étendues

Voir §12 de [STRATEGY_JSON_FORMAT.md](STRATEGY_JSON_FORMAT.md#12-annexe--extensions-pmx-non-prioritaires) :
`min_time_remaining_sec`, `max_time_remaining_sec`, `forbidden_flag`,
`near_position`.

---

## 9. Checklist d'implémentation v2

- [x] [StrategyJsonRunner.hpp](../src/common/ia/StrategyJsonRunner.hpp) : ajouté `struct InstructionOutcome` (+ enum `Status` à 8 valeurs incluant `SKIPPED_MAX_TIME`) + `std::vector<InstructionOutcome> outcomes_` + accesseurs `outcomes()` / `countByStatus()`.
- [x] [StrategyJsonRunner.cpp](../src/common/ia/StrategyJsonRunner.cpp) :
  - `executeInstruction` retourne maintenant `TRAJ_STATE` (au lieu de `bool`).
  - `runTasks` adapté : compare `== TRAJ_FINISHED` pour préserver son retour `bool` (utilisé par `O_State_NewInit::setPos()`).
  - `run()` réécrit selon §5.2 (skip+continue, populate outcomes, retourne toujours `true` après boucle).
  - `resetEmergencyOnTraj` appelé après skip sur erreur trajectoire (cf §3, encadré « Important »).
  - `priority<0` / `needed_flag` / `max_match_sec` déplacés de `executeInstruction` vers `run()` pour distinguer les statuts. `min_match_sec` (attente) reste dans `executeInstruction`.
- [x] Tests : SR06 adapté (option A §6.1). Ajout d'`expectedOutcomeStatuses` aux 6 scénarios. Statut : **6/6 PASS** sur build-simu-debug.
- [ ] [STRATEGY_JSON_EXECUTION.md](STRATEGY_JSON_EXECUTION.md) : compléter le §6.4 d'intégration `O_State_DecisionMakerIA` pour qu'il consomme `outcomes()` (log de fin de match).
- [ ] Stratégie d'homologation : `simulator/resources/2026/strategyHOMOLOG.json` à rédiger avec l'utilisateur.
- [ ] CLI `/s strategyHOMOLOG.json` doit fonctionner sans recompilation.
- [x] **Simulateur** : aucune modification en v2 (option SIM-C, voir §10). Documenté ci-dessous comme dette.

---

## 10. Symétrie simulateur ↔ runner — décision SIM-C

Le simulateur JS [visualisator.js](../../simulator/javascript/visualisator.js)
exécute aujourd'hui une lecture linéaire de la stratégie en supposant que
**toutes** les instructions réussissent (tracé continu, pas de notion de
blocage). Le runner C++ v2 va diverger en sautant des instructions sur
`TRAJ != FINISHED`. On a trois façons de gérer la symétrie :

| Option | Comportement simu | Coût |
|---|---|---|
| **SIM-A** | Champ JSON `simu_skip: true` ou `simu_skip_on_color: "bleu"` posé sur l'instruction → le simu force le skip | ~30 lignes JS, ignoré par le runner C++ |
| **SIM-B** | Adversaire dessiné dans le simu (cercle déplaçable), instruction skippée si trajectoire passe à <N mm | ~150 lignes JS, plus réaliste |
| **SIM-C** | Aucune simulation de blocage en v2. Le simu joue toujours « tout réussit ». Symétrie skip ajoutée plus tard. | 0 ligne |

**Décision retenue : SIM-C**.

Justification :
- v2 doit livrer la valeur métier (skip+continue côté robot) sans s'éparpiller.
- Le simulateur reste un outil de visualisation des trajectoires, pas un
  validateur de logique de décision.
- Quand on aura besoin de tester des scénarios de blocage côté simu (ex:
  homologation passée, on veut roder la stratégie de match contre adv),
  on choisira entre SIM-A et SIM-B en connaissance de cause.

**Note importante** : tant que le simu ne simule pas le skip, un fichier
JSON vu OK dans le simu peut **comporter une instruction qui ne sera
jamais jouée** côté robot (à cause d'un `needed_flag` mal calé ou d'une
trajectoire qui ne passe pas). Le `outcomes()` du runner reste l'unique
source de vérité runtime — à logger en fin de match.

---

## 11. Comparaison EsialRobotik / décisions retenues

Notre format JSON est calqué sur EsialRobotik
[Ia-Python](https://github.com/EsialRobotik/Ia-Python). Leur runtime
diverge sur plusieurs points qu'il faut connaître pour comprendre les
choix retenus ici.

### 11.1 Ce qu'EsialRobotik fait

**`StrategyManager.get_next_objective()`** (`ia/manager/strategy_manager.py`) :

```python
while self.current_index < len(self.objectives):
    next_objective = self.objectives[self.current_index]
    self.current_index += 1
    if next_objective.needed_flag is not None and next_objective.needed_flag not in self.action_flags:
        continue
    return next_objective
```

- Index linéaire qui n'avance jamais en arrière → un objectif passé est
  définitivement perdu.
- **`priority` n'est pas utilisé au runtime** (commentaire `# TODO:
  Devenir intelligent avec gestion des priorités` dans le code).
- Skip uniquement sur `needed_flag` absent. Pas de skip sur échec.

**`master_loop.main_loop()`** : boucle de contrôle continue qui ne
**skippe jamais** un objectif sur échec. Si une trajectoire A* est
bloquée par un adversaire détecté au lidar, ils **recalculent un A* à
chaud** en excluant la zone adversaire. Si l'asserv est bloquée
physiquement, ils loggent et attendent.

**Décision dynamique = 100% via flags** : pas de `forbidden_flag`, pas
de scoring, pas de `points/durée`. Ils utilisent uniquement
`needed_flag` + `action_flag` + `clear_flags`, ce qui est suffisant pour
exprimer toutes les exclusivités (cf §7.3).

### 11.2 Tableau de comparaison

| Sujet | EsialRobotik | PMX v2 (ce doc) |
|---|---|---|
| Trajectoire bloquée par adv | Recalcul A* dynamique (lidar) | Retry Navigator épuisé → skip instruction |
| Asserv bloquée physiquement | Log + attendre indéfiniment | Skip instruction |
| `priority` runtime | Non utilisé (TODO dans le code) | Tri stable_sort desc + `<0` désactive |
| Skip d'objectif | Uniquement `needed_flag` absent | Aussi sur `TRAJ != FINISHED` |
| Reprise différée | Impossible (index linéaire) | Pas en v2, prévu v3 (§8.1) |
| Décision dynamique | Flags uniquement | Flags + priority au tri |
| Format JSON | Identique en termes de champs | Identique (1:1) |

### 11.3 Ce qu'on garde d'EsialRobotik

- **Pattern d'exclusivité par flags** (§7.3) : `clear_flags` +
  `needed_flag` suffisent, pas besoin d'inventer `forbidden_flag`.
- **Sémantique des flags d'instruction** : levés/effacés uniquement si
  l'instruction complète a réussi.
- **Format JSON 1:1** : un fichier de stratégie écrit pour PMX peut être
  ouvert tel quel dans le simulateur et inversement.

### 11.4 Ce qu'on diverge volontairement

- **`priority` au runtime** : on l'utilise (tri + désactivation), eux non.
  Plus pratique que l'index linéaire fixe pour la lisibilité de la strat.
- **Skip sur échec** : on n'a pas leur lidar ni leur recalcul A* à chaud.
  Sauter une instruction bloquée et passer à la suivante est notre seule
  option fiable. Ils peuvent se permettre de rester sur un objectif parce
  qu'ils savent contourner.

### 11.5 Dette technique vs EsialRobotik

À traiter si/quand le besoin émergera :

| Item | Bénéfice | Effort estimé |
|---|---|---|
| Recalcul A* dynamique sur trajectoire bloquée | S'aligne sur Esial. Évite le skip d'objectif quand un contournement est possible. | Moyen (intégration capteurs adv ↔ playground A* à chaud) |
| Couche décision avec scoring `points/durée/distance` | Choix optimal d'instruction selon contexte | Moyen (§8.3) |
| Reprise différée des skipped_obstacle | Récupère une 2e chance si l'adv bouge | Faible (§8.1) |
| Préconditions étendues (`forbidden_flag`, `min_time_remaining_sec`) | Confort d'écriture, plus expressif | Faible-moyen |

Aucune n'est bloquante pour l'homologation 2026. À ré-évaluer après les
premiers matchs réels.

---

## 12. Références

- [STRATEGY_JSON_EXECUTION.md](STRATEGY_JSON_EXECUTION.md) — architecture du runner et phases
- [STRATEGY_JSON_FORMAT.md](STRATEGY_JSON_FORMAT.md) — format JSON détaillé
- [STRATEGY_JSON_ROADMAP.md](STRATEGY_JSON_ROADMAP.md) — feuille de route
- [/home/pmx/git/PMX-CORTEX/robot/src/common/ia/StrategyJsonRunner.cpp](../src/common/ia/StrategyJsonRunner.cpp)
- [/home/pmx/git/PMX-CORTEX/robot/src/common/ia/StrategyJsonRunner.hpp](../src/common/ia/StrategyJsonRunner.hpp)
- [/home/pmx/git/PMX-CORTEX/robot/src/common/ia/StrategyJsonParser.hpp](../src/common/ia/StrategyJsonParser.hpp)
- [/home/pmx/git/PMX-CORTEX/robot/src/common/navigator/Navigator.cpp](../src/common/navigator/Navigator.cpp)
- [/home/pmx/git/PMX-CORTEX/robot/src/bot-test-opos6ul/O_StrategyJsonRunnerTest.cpp](../src/bot-test-opos6ul/O_StrategyJsonRunnerTest.cpp)
- [/home/pmx/git/PMX-CORTEX/robot/src/bot-test-opos6ul/scenarios/strategySR06.json](../src/bot-test-opos6ul/scenarios/strategySR06.json)
