# Configuration match : saisie rapide de la disposition des zones de prise

## Objectif

Pendant les **3 minutes de préparation** avant le match, l'arbitre annonce la disposition des **8 zones de prise** (P1–P4 et P11–P14) sur la table. Il faut saisir cette disposition dans le robot **rapidement, sans erreur, en minimum de clics** via l'écran LCD tactile de la balise.

Note : les cases **GM1–GM14** visibles sur le plan de table sont d'autres éléments (zones de stockage / score) et ne sont **pas** couvertes par cet écran.

## Contexte & contraintes

- Temps de préparation très court (3 min pour tout : config, calibration, placement).
- Saisie sous stress, par un humain non spécialiste de l'UI.
- **Pas de symétrie** imposée par le règlement → chaque zone est indépendante.
- Pas de preset "match précédent" ni de scénario type → **pas d'accélérateur** dans la v1.
- L'écran est toujours dessiné en vue opérateur côté **bleu bas-gauche** (rotation 180°), conformément au plan de table de référence.

## Zones de prise

8 emplacements **P1–P4** (côté bleu en orientation rotation 180°) et **P11–P14** (côté jaune), chacun composé de **4 rectangles** côte à côte sur la longueur, colorés **2 jaunes + 2 bleus** dans un ordre variable.

### Orientation par zone

| Zone | Orientation | Côté |
|---|---|---|
| P1 | Verticale | bleu |
| P2 | Verticale | bleu |
| P3 | Horizontale | bleu |
| P4 | Horizontale | bleu |
| P11 | Verticale | jaune |
| P12 | Verticale | jaune |
| P13 | Horizontale | jaune |
| P14 | Horizontale | jaune |

### Disposition sur la table (vue opérateur, rotation 180° — bleu à gauche)

```
                ARRIÈRE (fond de table)
  ┌────────────────────────────────────────────┐
  │                                            │
  │              [P3-H]         [P13-H]        │
  │   [P2-V]                                   │
  │                                    [P12-V] │
  │              [P4-H]         [P14-H]        │
  │                                            │
  │   [P1-V]                                            │
  │                                    [P11-V] │
  │                                            │
  │   NID BLEU                        NID JAUNE│
  └────────────────────────────────────────────┘
                AVANT (public)
```

(Positions précises à caler sur le plan officiel — voir `robot/svg*.svg` et TODO en bas.)

## Combinaisons par zone

Avec 4 positions et la contrainte **2 jaunes + 2 bleues**, il y a exactement $\binom{4}{2} = 6$ arrangements :

```
  1: YYBB     2: YBYB     3: YBBY
  4: BYYB     5: BYBY     6: BBYY
```

Chaque zone choisit **1 combinaison parmi 6**, indépendamment des autres.

### Rendu horizontal / vertical

```
Horizontal (P3, P4, P13, P14) :
  YYBB →  [ Y | Y | B | B ]

Vertical (P1, P2, P11, P12) :
  YYBB →  [ Y ]
          [ Y ]
          [ B ]
          [ B ]
```

## Nombre total de dispositions

$6^8 = 1\,679\,616$ combinaisons possibles sur la table.

## Interface LCD tactile — cycle par flèches

### Règles ergonomiques figées

1. **Accès direct** : depuis l'écran en cours (menu balise principal), **1 clic** ouvre l'écran de config match. Pas de sous-menu intermédiaire.
2. **Pas de popup, pas de header, pas de footer** : tout l'écran est occupé par les 8 zones pour un affichage aussi grand que possible.
3. **Une zone = 3 éléments tactiles** : flèche `◀` (combinaison précédente) / combi courante affichée en couleurs réelles / flèche `▶` (combinaison suivante). Cycle bidirectionnel sur les 6 combinaisons.
4. **Validation automatique via registry** : pas de bouton VALIDER. Chaque tap sur une flèche met à jour immédiatement le registre I2C `Settings` (bloc 2, Teensy → OPOS6UL) et incrémente `seq_touch`, comme les autres paramètres pré-match (`matchColor`, `strategy`, `testMode`, `advDiameter`). L'OPOS6UL relit à son rythme et applique la nouvelle config. Par défaut, toutes les zones valent **0 = `BBYY`**.
5. **Libellé Pn** au-dessus de chaque zone, pour éviter toute ambiguïté.

### Ordre canonique des 6 combinaisons (cycle `▶`)

```
  1 → 2 → 3 → 4 → 5 → 6 → 1 ...
┌──────┬──────┬──────┬──────┬──────┬──────┐
│ BBYY │ YYBB │ BYYB │ YBBY │ BYBY │ YBYB │
└──────┴──────┴──────┴──────┴──────┴──────┘
   (défaut)
```

- `▶` avance d'un cran (`BBYY → YYBB → BYYB → …`).
- `◀` recule d'un cran (`BBYY → YBYB → BYBY → …`).
- Distance max entre deux combinaisons = 3 clics (cycle de 6 bidirectionnel).

Rendu des miniatures selon l'orientation de la zone :

```
Horizontale (P3, P4, P13, P14)
  BBYY :  [ B | B | Y | Y ]

Verticale (P1, P2, P11, P12)
  BBYY :  [ B ]
          [ B ]
          [ Y ]
          [ Y ]
```

### Budget clics

- **1** clic pour entrer sur l'écran.
- **0 à 3** clics par zone selon la distance entre défaut et cible.
- **Moyenne estimée** : ~1,5 clic × 8 zones = **12 clics + 1 entrée ≈ 13 taps**.
- **Pire cas** : 1 + 8 × 3 = **25 taps**. Largement dans le budget 3 min.

### Mockup écran (~320×240, vue opérateur, rotation 180°)

```
┌────────────────────────────────────────────────────────────┐
│                                                            │
│                   P3                    P13                │
│            ◀ [B|B|Y|Y] ▶         ◀ [B|B|Y|Y] ▶             │
│                                                            │
│                                                            │
│     P2                                              P12    │
│    ┌─┐                                              ┌─┐    │
│    │B│                                              │B│    │
│  ◀ │B│ ▶                                          ◀ │B│ ▶  │
│    │Y│                                              │Y│    │
│    │Y│                                              │Y│    │
│    └─┘                                              └─┘    │
│                                                            │
│                   P4                    P14                │
│            ◀ [B|B|Y|Y] ▶         ◀ [B|B|Y|Y] ▶             │
│                                                            │
│                                                            │
│     P1                                              P11    │
│    ┌─┐                                              ┌─┐    │
│    │B│                                              │B│    │
│  ◀ │B│ ▶                                          ◀ │B│ ▶  │
│    │Y│                                              │Y│    │
│    │Y│                                              │Y│    │
│    └─┘                                              └─┘    │
│                                                            │
└────────────────────────────────────────────────────────────┘
```

Toutes les zones sont initialisées à `BBYY` (défaut).

### Affordances proposées

- **Couleur réelle** dans les cases : `Y` = fond jaune, `B` = fond bleu, lettre blanche par-dessus pour lisibilité à distance.
- **Triangles** : cibles tactiles de ~40×40 px mini, bien espacées des cases couleur pour éviter les taps accidentels.
- **Feedback visuel** sur un tap : brève animation de la combi affichée (flash ou transition) pour confirmer que le cycle a avancé.
- Sortie de l'écran → pas d'écriture spécifique, l'état est déjà dans le registre I2C (mis à jour à chaque tap).

## Persistance : registry I2C `Settings`

On réutilise le même mécanisme que les autres paramètres pré-match (`matchColor`, `strategy`, `testMode`, `advDiameter`) : les 8 zones sont ajoutées à la struct `Settings` dans [src/TofSensors.h](/home/pmx/git/PMX-CORTEX/teensy/IO_t41_ToF_DetectionBeacon/src/TofSensors.h), bloc 2 (Teensy LCD → OPOS6UL). Pas de fichier JSON, pas de flux séparé.

### Encodage

Chaque zone = **1 octet** contenant l'**index 0..5** dans le cycle canonique :

| index | combinaison |
|---|---|
| 0 | BBYY (défaut) |
| 1 | YYBB |
| 2 | BYYB |
| 3 | YBBY |
| 4 | BYBY |
| 5 | YBYB |

### Extension de la struct `Settings`

```cpp
struct Settings {
    // === Bloc 1 : OPOS6UL -> Teensy (5 bytes) ===
    int8_t  numOfBots     = 3;
    int8_t  ledLuminosity = 10;
    uint8_t matchPoints   = 0;
    uint8_t matchState    = 0;
    uint8_t lcdBacklight  = 1;

    // === Bloc 2 : Teensy (LCD) -> OPOS6UL (5 + 8 = 13 bytes) ===
    uint8_t matchColor    = 0;
    uint8_t strategy      = 0;
    uint8_t testMode      = 0;
    uint8_t advDiameter   = 40;
    uint8_t actionReq     = 0;

    // Zones de prise (config pré-match, index 0..5 dans le cycle canonique)
    uint8_t pickup_P1  = 0;  ///< Reg 10. 0=BBYY ... 5=YBYB (W: LCD).
    uint8_t pickup_P2  = 0;  ///< Reg 11.
    uint8_t pickup_P3  = 0;  ///< Reg 12.
    uint8_t pickup_P4  = 0;  ///< Reg 13.
    uint8_t pickup_P11 = 0;  ///< Reg 14.
    uint8_t pickup_P12 = 0;  ///< Reg 15.
    uint8_t pickup_P13 = 0;  ///< Reg 16.
    uint8_t pickup_P14 = 0;  ///< Reg 17.

    // === Bloc 3 : compteur de clics touch (1 byte) ===
    uint8_t seq_touch = 0;   ///< Reg 18. Incrémenté à chaque modif Bloc 2.
};
static_assert(sizeof(Settings) == 19, "Settings must be exactly 19 bytes for I2C layout");
```

### Flux

1. **Boot Teensy** : tous les `pickup_Pn` = 0 (`BBYY`). `seq_touch` = 0.
2. **Tap sur la combi** (callback LVGL) : met à jour `pickup_Pn`, cycle `(val + 1) % 6` (tap court) ou `(val + 5) % 6` (appui long), puis `seq_touch++`.
3. **OPOS6UL** (`SensorsDriver::syncFull`) lit les 19 bytes de `Settings` à chaque tick. `MenuBeaconLCDTouch::pollInputs()` détecte `seq_touch` incrémenté et propage les 8 `pickup_Pn` dans `Robot` via les setters dédiés.
4. **IA** consulte `Robot::pickupPn()` au démarrage du match pour adapter trajectoires/ordres de prise (Phase 3, à venir).

## Intégration code (réalisée)

### Côté balise (Teensy) — écran tactile LVGL

- Écran LVGL créé via `create_pickup_config()` dans [src/LCDScreen.cpp](/home/pmx/git/PMX-CORTEX/teensy/IO_t41_ToF_DetectionBeacon/src/LCDScreen.cpp).
- 8 widgets uniformes 72×74 (grille 4×2 vue opérateur rotation 180°).
- Tap court = cycle +1, appui long (~500 ms) = cycle -1. Pas de flèches → écran moins chargé.
- Bouton **ZONES** noir dans le menu pré-match (à côté de COULEUR et SETPOS) — accès en 1 clic.
- Bouton **MENU** noir centré en bas pour revenir.
- Carrés pointillés bleu (bas-gauche) / jaune (bas-droite) pour rappeler le sens de la table.

### Côté OPOS6UL (brain)

Pas de classe `MatchConfig` séparée (over-engineering écarté). Pattern aligné sur les autres champs LCD (`matchColor`, `strategy`, `advDiameter`) :

- **`Robot`** : 8 membres `pickup_Pn_` privés + getters `pickupPn()` + setters `setPickupPn(idx)` bornés à 0..5 et bloqués en `PHASE_MATCH`.
- **`MatchSettingsData`** ([common/interface/ASensorsDriver.hpp](/home/pmx/git/PMX-CORTEX/robot/src/common/interface/ASensorsDriver.hpp)) : 8 nouveaux champs `pickup_Pn` (contrat commun driver/menu).
- **`SensorsDriver::syncFull`** ([driver-arm/SensorsDriver.cpp](/home/pmx/git/PMX-CORTEX/robot/src/driver-arm/SensorsDriver.cpp)) : recopie les 8 nouveaux champs dans `cached_settings_` après chaque lecture I2C.
- **`MenuBeaconLCDTouch::pollInputs`** ([common/menu/MenuBeaconLCDTouch.cpp](/home/pmx/git/PMX-CORTEX/robot/src/common/menu/MenuBeaconLCDTouch.cpp)) :
  - Adoption au boot (`shadowInit`) → 8 appels `robot.setPickupPn(current.pickup_Pn)`.
  - Delta runtime déclenché par `seq_touch` incrémenté → macro locale `POLL_DELTA_PICKUP` × 8.

### Fichiers concernés

- [teensy/IO_t41_ToF_DetectionBeacon/src/TofSensors.h](/home/pmx/git/PMX-CORTEX/teensy/IO_t41_ToF_DetectionBeacon/src/TofSensors.h) — struct `Settings` 19 bytes.
- [teensy/IO_t41_ToF_DetectionBeacon/src/LCDScreen.cpp](/home/pmx/git/PMX-CORTEX/teensy/IO_t41_ToF_DetectionBeacon/src/LCDScreen.cpp) — écran LVGL + callbacks tap/long-press.
- [robot/src/driver-arm/BeaconSensors.hpp](/home/pmx/git/PMX-CORTEX/robot/src/driver-arm/BeaconSensors.hpp) / [.cpp](/home/pmx/git/PMX-CORTEX/robot/src/driver-arm/BeaconSensors.cpp) — `SETTINGS_SIZE_BeaconSensors=19`, struct miroir, `readSettings` parse 19 bytes.
- [robot/src/common/interface/ASensorsDriver.hpp](/home/pmx/git/PMX-CORTEX/robot/src/common/interface/ASensorsDriver.hpp) — `MatchSettingsData` + 8 `pickup_Pn`.
- [robot/src/driver-arm/SensorsDriver.cpp](/home/pmx/git/PMX-CORTEX/robot/src/driver-arm/SensorsDriver.cpp) — `syncFull` recopie les 8 champs.
- [robot/src/common/Robot.hpp](/home/pmx/git/PMX-CORTEX/robot/src/common/Robot.hpp) — 8 membres + getters/setters.
- [robot/src/common/menu/MenuBeaconLCDTouch.cpp](/home/pmx/git/PMX-CORTEX/robot/src/common/menu/MenuBeaconLCDTouch.cpp) — adoption + delta.

### Phase 3 (à venir) — consommation côté IA

- Brancher `Robot::pickupPn()` dans `StrategyJsonRunner` ou code dédié 2026.
- Pour chaque zone, dispatch sur l'index 0..5 → ordre de prise spécifique (trajectoire, séquence d'actuateurs, etc.).

## Questions ouvertes / TODO

- [x] Côté OPOS6UL : identifier le reader I2C beacon — `BeaconSensors`/`SensorsDriver`, sync via `MenuBeaconLCDTouch`.
- [x] Définir l'interaction de sortie de l'écran — bouton MENU noir centré en bas.
- [x] Spécifier le protocole balise ↔ OPOS6UL pour la config — réutilise le pattern `seq_touch` existant.
- [ ] Confirmer coordonnées (x,y) mm des 8 zones sur la table officielle 2026 (impact stratégie).
- [ ] Vérifier que la numérotation **P1–P4 / P11–P14** (et non P1–P8 séquentiel) correspond bien au règlement 2026 officiel.
- [ ] Préciser correspondance P ↔ GM (ex : P3 = zone de prise associée à GM5/GM6 ?).
- [ ] **Phase 3** : brancher `Robot::pickupPn()` côté IA — dispatch sur index 0..5 → ordre de prise spécifique par zone.

## Références

- [ARCHITECTURE_BEACON.md](/home/pmx/git/PMX-CORTEX/teensy/IO_t41_ToF_DetectionBeacon/ARCHITECTURE_BEACON.md) — architecture du projet balise (ce même dossier)
- [robot/md/ARCHITECTURE.md](/home/pmx/git/PMX-CORTEX/robot/md/ARCHITECTURE.md) — architecture globale OPOS6UL
- [robot/md/HARDWARE_CONFIG.md](/home/pmx/git/PMX-CORTEX/robot/md/HARDWARE_CONFIG.md) — pattern de config fichier à côté de l'exécutable
- [robot/md/STRATEGY_JSON_EXECUTION.md](/home/pmx/git/PMX-CORTEX/robot/md/STRATEGY_JSON_EXECUTION.md) — runner stratégie JSON
- [robot/md/O_STATE_NEW_INIT.md](/home/pmx/git/PMX-CORTEX/robot/md/O_STATE_NEW_INIT.md) — menu balise tactile multi-sources
