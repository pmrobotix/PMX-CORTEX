# Architecture des gestes ToF — Teensy beacon

Document d'architecture pour la détection et l'action des gestes (UA bilatéral,
long hold, convergent, swipe CW/CCW, Hey, …) sur la balise IO_t41_ToF_DetectionBeacon.

> **Statut** : infrastructure créée, **non encore intégrée**. Cohabite avec
> l'ancien code `thread_display()` / fin de `tof_loop()` jusqu'à migration.

---

## 1. Contexte et problème actuel

Aujourd'hui la détection des gestes est répartie en deux endroits :

| Endroit | Rôle |
|---|---|
| `TofSensors.cpp` fin de `tof_loop()` | Calcule `proximity_level` (0/1/2), classifie le geste UA à la transition de sortie (`last_ua_gesture`), accumule le swipe (`last_swipe_gesture`) |
| `LedPanels.cpp` `thread_display()` | Lit ces variables, déclenche actions (toggle match, flash couleur, scroll Hey, …) |

Symptômes :
- **Variables globales dispersées** : `proximity_level`, `last_ua_gesture`,
  `last_swipe_gesture`, `match_mode_actif`, `video_2`. Ajouter un geste = ajouter
  une variable et toucher 2 fichiers.
- **Conditions et actions mélangées** : la décision "BILATERAL → toggle match"
  est codée en dur dans `thread_display()` au milieu d'autres branches.
- **Désactivation = commenter** : pour la compétition on a commenté Hey
  (`proximity_level == 2`) et les flashs swipe → restes morts (`video_2`,
  `last_swipe_gesture` non consommé).
- **Blocages** : `while (proximity_level >= 1) { threads.delay(500); }`
  dans `thread_display()` peut faire grimper la latence (`latency_thread_error`
  à 90 ms dans `tof_loop`).
- **Effets de bord cachés** : `match_mode_actif = 1` forcé en cas de latence
  élevée à `TofSensors.cpp:1543` perturbe l'affichage LED sans être documenté.

---

## 2. Architecture cible

**Un seul registry `Gesture[]`**, évalué chaque cycle par une fonction
`gestures_evaluate()` non-bloquante. Chaque entrée du registry décrit
**complètement** un geste : nom, enabled, cooldown, condition, action.

```
+------------------+        +-------------------+        +-------------------+
| TofSensors       |        | Gestures          |        | LedPanels / Match |
| (calcule proxi-  | -----> | (registry+eval()) | -----> | (effets visuels,  |
|  mity_level,     |        |                   |        |  toggle, etc.)    |
|  last_ua_gesture,|        |                   |        |                   |
|  last_swipe...)  |        |                   |        |                   |
+------------------+        +-------------------+        +-------------------+
        producteur                arbitre                    consommateurs
```

`Gestures` est l'**arbitre unique** :
- Lit l'état partagé (variables globales existantes côté TofSensors).
- Évalue les conditions de chaque geste activé.
- Déclenche les actions (toggle, flash, etc.) — qui sont **non-bloquantes**.
- Consomme l'état (reset `last_ua_gesture`, `last_swipe_gesture`) pour éviter
  les re-triggers et les accumulations.

---

## 3. Spec de la struct `Gesture`

```cpp
struct Gesture {
    const char* name;          // log/debug
    bool        enabled;       // ON/OFF, à flipper pour activer/désactiver
    uint32_t    cooldown_ms;   // anti-rebond après trigger
    bool      (*condition)();  // évaluation rapide, lit l'état partagé
    void      (*action)();     // effet (idéalement non-bloquant)
    uint32_t    last_fire_ms;  // timestamp du dernier trigger
};
```

| Champ | Sémantique |
|---|---|
| `name` | Affiché dans les logs (`Serial.print` debug). |
| `enabled` | Si `false`, la condition n'est même pas évaluée. |
| `cooldown_ms` | Délai minimum entre deux triggers consécutifs **du même geste**. Tant que `now - last_fire_ms < cooldown_ms`, on skippe. |
| `condition()` | Retourne `true` si le geste doit firer **maintenant**. Doit être courte et sans `delay`. |
| `action()` | Effet quand condition vraie. Doit être **courte** : marquer un état (FSM flash) ou toggler une variable, **jamais** bloquer. |
| `last_fire_ms` | Mis à `millis()` après chaque trigger. Initialisé à 0. |

---

## 4. Règles non-bloquantes — **strictes**

Le thread d'affichage et `tof_loop` partagent la cadence. Tout blocage dans
les gestes risque d'augmenter la latence et de casser l'acquisition VL53L1X
(18 capteurs en couronne sur la balise).

| ✅ Autorisé | ❌ Interdit |
|---|---|
| `if (now - last < cooldown) continue;` | `threads.delay(cooldown);` |
| Marquer un flash via `FlashState` (timestamp + durée), rendu au cycle suivant | `flash_ua_gesture(color, 1000)` qui bloque |
| `return` rapide depuis `condition()` | `while (cond) { yield/delay }` dans condition |
| FSM (état persistent + check timestamp à chaque cycle) | Toute attente synchrone d'un événement |

Anti-pattern à éliminer pendant la migration :
[`LedPanels.cpp:264-267`](src/LedPanels.cpp#L264-L267) `while (proximity_level >= 1) { threads.delay(500); }`.

---

## 5. Inventaire des gestes

| Nom | Source événement | Condition | Action | Cooldown | Activé init |
|---|---|---|---|---|---|
| `BILATERAL`  | `last_ua_gesture == UA_GESTURE_BILATERAL`  | sortie UA, mains des 2 côtés, < 2 s | toggle `match_mode_actif` | 500 ms | **oui** |
| `LONG_HOLD`  | `last_ua_gesture == UA_GESTURE_LONG_HOLD`  | UA tenu ≥ 2 s | flash rouge 1.5 s | 1500 ms | non |
| `CONVERGENT` | `last_ua_gesture == UA_GESTURE_CONVERGENT` | sortie UA, mains concentrées d'un côté | flash vert 1 s | 1000 ms | non |
| `SWIPE_CW`   | `last_swipe_gesture == SWIPE_CW`           | swipe horaire détecté par TofSensors | flash cyan 0.8 s | 800 ms | non |
| `SWIPE_CCW`  | `last_swipe_gesture == SWIPE_CCW`          | swipe anti-horaire | flash magenta 0.8 s | 800 ms | non |
| `HEY`        | `proximity_level == 2`                     | objet très proche (< 60 mm) sur ≥ 10 zones | scroll text "Hey!!" | 2000 ms | non |

**Exclusivité UA** : la priorité (`LONG_HOLD > CONVERGENT > BILATERAL`) est déjà
gérée par `TofSensors::tof_loop()` qui n'écrit qu'**une seule** valeur dans
`last_ua_gesture` à la sortie. Le registry n'a donc pas à arbitrer : la première
condition vraie consomme la variable, les suivantes voient `UA_GESTURE_NONE`.

---

## 6. FSM flash non-bloquante

Pour les actions visuelles longues (flash 1 s, scroll text), l'action ne fait
que **marquer un état** consulté par le rendu LED.

```cpp
struct FlashState {
    bool     active;
    uint32_t color;       // RRGGBB packé
    uint32_t start_ms;
    uint32_t duration_ms;
};
extern volatile FlashState g_flash;
```

L'action :
```cpp
g_flash.color       = 0xFF0000;
g_flash.start_ms    = millis();
g_flash.duration_ms = 1500;
g_flash.active      = true;
```

Le rendu LED (à intégrer dans `thread_display()` plus tard) :
```cpp
if (g_flash.active) {
    if (millis() - g_flash.start_ms >= g_flash.duration_ms) {
        g_flash.active = false;
    } else {
        // dessiner le flash
    }
}
```

Aucun `delay()`, le rendu repasse simplement chaque cycle.

---

## 7. Plan de migration (progressif, testé entre chaque étape)

| Étape | Action | État |
|---|---|---|
| 1 | Créer `Gestures.hpp` / `Gestures.cpp` avec registry et BILATERAL seulement (`enabled=true`) | **fait** |
| 2 | Créer ce MD | **fait** |
| 3 | Appeler `gestures_evaluate()` dans `thread_display()` après le bloc `proximity_level == 1`. Garder l'ancien code en parallèle. Vérifier : BILATERAL toggle match comme avant ? | à faire |
| 4 | Retirer le bloc UA inline dans `thread_display()` (lignes 273-289). BILATERAL passe par le registry. Tester en match. | à faire |
| 5 | Activer LONG_HOLD (`enabled=true`), porter `flash_ua_gesture` en mode FSM, vérifier non-bloquant. | à faire |
| 6 | Activer CONVERGENT idem. Vérifier robustesse de la classification (cf. note §8). | à faire |
| 7 | Migrer le rendu Hey en FSM, activer HEY. Supprimer `video_2`. | à faire |
| 8 | Activer SWIPE_CW / SWIPE_CCW si la détection est jugée fiable, sinon laisser `enabled=false`. | à faire |
| 9 | Si tous les gestes UA passent par le registry et que rien ne lit `last_ua_gesture` ailleurs, simplifier `tof_loop()` (factoriser). Idem swipe si `enabled=false` partout : retirer 190 lignes de détection. | optionnel |
| 10 | Retirer `match_mode_actif = 1` à `TofSensors.cpp:1543` (effet de bord caché, non lié aux gestes). | optionnel |

---

## 8. Notes de robustesse (à traiter pendant la migration)

- **CONVERGENT instable** : `ua_last_concentrated` n'est pas latché — c'est
  l'état du **dernier cycle** avant sortie de proximity==1, alors que
  l'utilisateur est en train de retirer ses mains. À latcher au mieux
  observé pendant la fenêtre UA (ou exiger N cycles consécutifs concentrés).
- **Seuils UA en dur** : `>5` et `<=2` zones front/back, sans hystérésis.
  À paramétrer si on garde le geste.
- **`match_mode_actif` à 1543** : effet de bord caché, à retirer (le
  `latency_thread_error = 1` suffit). Hors scope gestes.
- **Init "tout seul"** : symptôme rapporté par l'utilisateur, **pas causé
  par les gestes** (`match_mode_actif` n'est pas exposé I2C). À analyser
  côté OPOS6UL `O_State_NewInit` séparément.

---

## 9. Recette : ajouter un nouveau geste

1. **Écrire la condition** dans `Gestures.cpp` :
   ```cpp
   static bool cond_mongeste() {
       // lecture de l'état partagé, retour bool
   }
   ```
2. **Écrire l'action** non-bloquante :
   ```cpp
   static void act_mongeste() {
       // marquer un état, toggler une variable, etc.
   }
   ```
3. **Ajouter une ligne au registry** :
   ```cpp
   {"MONGESTE", true, 500, cond_mongeste, act_mongeste, 0},
   ```

Pas besoin de toucher `TofSensors.cpp`, `LedPanels.cpp`, ni les autres
fichiers tant que la condition s'exprime en fonction de l'état déjà exposé.
