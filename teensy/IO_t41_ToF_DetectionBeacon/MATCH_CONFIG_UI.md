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
2. **Tap sur flèche** (callback LVGL) : met à jour `pickup_Pn`, cycle `(val + 1) % 6` (▶) ou `(val + 5) % 6` (◀), puis `seq_touch++`.
3. **OPOS6UL** polle le registre I2C, détecte `seq_touch` incrémenté, relit les 8 `pickup_Pn` et met à jour sa `MatchConfig` en mémoire.
4. **StrategyJsonRunner** consulte `MatchConfig` au démarrage du match (`matchState` == EN_COURS).

## Intégration code (esquisse)

### Côté balise (Teensy) — écran tactile LVGL

- Nouvel écran LVGL `setup_screen_pickup_config()` créé au boot.
- 8 widgets composites (label + ◀ + carré couleur + ▶), placés selon le mockup.
- Callback flèche : cycle sur l'index + `settings.pickup_Pn = new_val` + `settings.seq_touch++`.
- Accès depuis le menu principal : 1 bouton tactile dédié.

### Côté OPOS6UL (brain)

- Classe `MatchConfig` (nouveau, `robot/src/common/MatchConfig.{hpp,cpp}`), qui expose :
  - `setZoneIndex(ZoneId, uint8_t idx)` appelée par le reader I2C beacon.
  - `getZone(ZoneId) -> ZoneLayout { combination: enum, orientation: H|V }`.
- Reader I2C beacon existant (polling `seq_touch`) lit les 8 nouveaux octets et appelle `MatchConfig::setZoneIndex`.
- `StrategyJsonRunner` consulte `MatchConfig::getZone` pour adapter trajectoires/ordres de prise.

### Fichiers concernés (prévisionnel)

- [teensy/IO_t41_ToF_DetectionBeacon/src/TofSensors.h](/home/pmx/git/PMX-CORTEX/teensy/IO_t41_ToF_DetectionBeacon/src/TofSensors.h) : extension struct `Settings`.
- [teensy/IO_t41_ToF_DetectionBeacon/src/LCDScreen.cpp](/home/pmx/git/PMX-CORTEX/teensy/IO_t41_ToF_DetectionBeacon/src/LCDScreen.cpp) : nouvel écran LVGL + callbacks.
- `robot/src/common/MatchConfig.{hpp,cpp}` (nouveau).
- Reader I2C beacon côté OPOS6UL (à localiser — probablement dans driver balise existant).
- `robot/src/ia/StrategyJsonRunner.{hpp,cpp}` (hook lecture config match).

## Questions ouvertes / TODO

- [ ] Confirmer la résolution exacte du LCD tactile balise (impacte dimensions miniatures).
- [ ] Confirmer coordonnées (x,y) mm des 8 zones sur la table officielle 2026.
- [ ] Spécifier le protocole série balise ↔ OPOS6UL pour la config match.
- [ ] Vérifier que la numérotation **P1–P4 / P11–P14** (et non P1–P8 séquentiel) correspond bien au règlement 2026 officiel.
- [ ] Préciser correspondance P ↔ GM (ex : P3 = zone de prise associée à GM5/GM6 ?).
- [ ] Définir l'interaction de sortie de l'écran (bouton "retour menu" ? tap sur zone vide ?).
- [ ] Côté OPOS6UL : identifier le reader I2C beacon existant pour y brancher la lecture des 8 nouveaux octets.

## Références

- [ARCHITECTURE_BEACON.md](/home/pmx/git/PMX-CORTEX/teensy/IO_t41_ToF_DetectionBeacon/ARCHITECTURE_BEACON.md) — architecture du projet balise (ce même dossier)
- [robot/md/ARCHITECTURE.md](/home/pmx/git/PMX-CORTEX/robot/md/ARCHITECTURE.md) — architecture globale OPOS6UL
- [robot/md/HARDWARE_CONFIG.md](/home/pmx/git/PMX-CORTEX/robot/md/HARDWARE_CONFIG.md) — pattern de config fichier à côté de l'exécutable
- [robot/md/STRATEGY_JSON_EXECUTION.md](/home/pmx/git/PMX-CORTEX/robot/md/STRATEGY_JSON_EXECUTION.md) — runner stratégie JSON
- [robot/md/O_STATE_NEW_INIT.md](/home/pmx/git/PMX-CORTEX/robot/md/O_STATE_NEW_INIT.md) — menu balise tactile multi-sources
