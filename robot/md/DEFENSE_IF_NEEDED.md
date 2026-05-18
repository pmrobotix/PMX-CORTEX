# Défense réactive — `defense_if_needed`

Action de stratégie qui, **à la fin d'une instruction push**, va placer le robot
sur un point de défense si l'adversaire occupe une zone d'attaque, puis lui rend
la main après une temporisation courte.

- **Code** : `robot/src/bot-opos6ul/StrategyActions2026.cpp`, fonction `defense_if_needed()`.
- **Enregistrement** : `registry.registerAction("defense_if_needed", ...)` dans le même fichier.
- **Appel** : depuis le JSON de stratégie, comme `action_id` (voir [STRATEGY_JSON_FORMAT.md](STRATEGY_JSON_FORMAT.md)).

---

## ⚠️ Statut actuel : STUB

L'action est **enregistrée et appelable** depuis le JSON, mais le corps **ne bouge
pas le robot**. Il se contente de :

- lire la position adverse (`robot.asserv().pos_getAdvPosition()`),
- logger une ligne `defense_if_needed STUB yellow=… adv=(x,y)`,
- retourner `true`.

Toutes les constantes de zones/points sont consommées par des `(void)` pour
éviter les warnings tant que le corps n'est pas finalisé.

➡️ **Conséquence** : modifier les coordonnées ci-dessous n'a aucun effet visible
tant que le corps reste un stub. Voir [Plan d'activation](#plan-dactivation).

---

## Où changer les positions

Les positions sont des `constexpr float` **en début de fonction `defense_if_needed()`**.
Elles s'écrivent en **coordonnées BLEU** (convention single-color) ; le côté jaune
est obtenu par **miroir X automatique** via `robot.changeMatchX()`.

| Constante | Rôle | Valeur BLEU actuelle |
|---|---|---|
| `ZONE1_XMIN_B` / `ZONE1_XMAX_B` | Zone d'attaque 1 — bornes X de l'adversaire | 1700 / 2000 |
| `ZONE1_YMIN_B` / `ZONE1_YMAX_B` | Zone d'attaque 1 — bornes Y de l'adversaire | 1300 / 1800 |
| `DEF1_X_B` / `DEF1_Y_B` | Point de défense associé à la zone 1 | 1400 / 1500 |
| `ZONE2_XMIN_B` / `ZONE2_XMAX_B` | Zone d'attaque 2 — bornes X de l'adversaire | 1700 / 2000 |
| `ZONE2_YMIN_B` / `ZONE2_YMAX_B` | Zone d'attaque 2 — bornes Y de l'adversaire | 700 / 1100 |
| `DEF2_X_B` / `DEF2_Y_B` | Point de défense associé à la zone 2 | 1400 / 900 |

> Les valeurs ci-dessus sont un snapshot ; la source de vérité reste le code.

### Conventions de repère

- **X** : miroité pour le jaune — `robot.changeMatchX(x_bleu)` retourne `x` ou `3000 - x`.
- **Y** : **jamais** miroité (la table n'a qu'une symétrie en X).
- **`theta_def`** (orientation au point de défense, dos à l'adversaire) :
  **180° en BLEU**, **0° en JAUNE** — calculé `theta_def_deg = yellow ? 0.0f : 180.0f`.

Pour recalibrer : modifier uniquement les valeurs BLEU, le miroir jaune est dérivé
automatiquement. Pour ajouter une zone, dupliquer un bloc `ZONE/DEF` et étendre le
test d'inclusion (voir étapes 3-5 ci-dessous).

---

## Position de l'adversaire

`defense_if_needed()` lit l'adversaire en **repère TABLE (mm)** via
`robot.asserv().pos_getAdvPosition()` (type `ROBOTPOSITION`).

- La plomberie est **en place** : `Sensors` publie la position via
  `Asserv::setAdvPosCentre` après projection balise → table
  (cf [SENSORS_DETECTION_MIGRATION.md](SENSORS_DETECTION_MIGRATION.md) et
  [DETECTION_ADV_CONVENTION.md](DETECTION_ADV_CONVENTION.md)).
- Valeur d'init `(-100, -100)` = **pas encore de détection valide** → à traiter
  comme un no-op (`return true`) lors de la finalisation.

---

## Plan d'activation

Les 6 étapes sont déjà décrites en commentaire `TODO` dans le corps de
`defense_if_needed()`. Résumé :

1. **Câbler la position adv** — déjà disponible via `pos_getAdvPosition()`.
2. **Garde adv invalide** — si `adv.x < 0 || adv.y < 0` → `return true` (no-op).
3. **Rectangles en repère réel** — appliquer le miroir X :
   `z1xmin = min(changeMatchX(ZONE1_XMIN_B), changeMatchX(ZONE1_XMAX_B))`,
   `z1xmax = max(...)` ; Y inchangé. Idem zone 2.
4. **Test d'inclusion** — `in_z1`, `in_z2` : adv dans le rectangle.
5. **Mouvement** — si `in_z1 || in_z2` :
   `def_x = changeMatchX(in_z1 ? DEF1_X_B : DEF2_X_B)`,
   `def_y = in_z1 ? DEF1_Y_B : DEF2_Y_B`,
   `theta_def_deg = yellow ? 0.0f : 180.0f`,
   `Navigator nav(&robot, &robot.ia().iAbyPath());`
   `nav.moveForwardToAndRotateAbsDeg(def_x, def_y, theta_def_deg);`
   `utils::sleep_for_secs(5);` — sinon no-op.
6. **`return true`** — le runner enchaîne sur la task suivante.

**V2 (plus tard)** : test du vecteur d'approche de l'adversaire (double échantillon
~300 ms + signe de `vx` selon couleur) pour éviter le détour quand l'adversaire est
immobile ou s'éloigne.

---

## Usage dans le JSON de stratégie

- `defense_if_needed` s'insère **à la fin de chaque instruction push** où une
  défense a du sens. C'est un **no-op si l'adversaire est hors zone** : aucun
  risque à le placer largement.
- **Aucun retour à la position pré-défense n'est géré** : c'est le `PATH_TO` de la
  task suivante qui repositionne le robot. Toutes les instructions push doivent
  donc commencer par un mouvement.

Voir le tableau des actions dans [STRATEGY_JSON_FORMAT.md](STRATEGY_JSON_FORMAT.md).
