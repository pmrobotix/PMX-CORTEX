#ifndef BOT_OPOS6UL_STRATEGY_ACTIONS_2026_HPP_
#define BOT_OPOS6UL_STRATEGY_ACTIONS_2026_HPP_

#include <cstdint>
#include <string>

class ActionRegistry;
class OPOS6UL_RobotExtended;

/*!
 * \brief Enregistre toutes les actions MANIPULATION specifiques au reglement
 *        Coupe de France 2026 dans l'ActionRegistry du runner JSON.
 *
 * Chaque action est associee a un identifiant (`action_id`) utilise dans les
 * fichiers JSON de strategie (cf. STRATEGY_JSON_FORMAT.md, type=MANIPULATION).
 *
 * Conventions retour :
 *  - true  : action reussie, le runner continue la task suivante
 *  - false : action echouee, la task MANIPULATION echoue et l'instruction
 *            courante est abort (run() passe a la suivante OU s'arrete, selon
 *            la politique du runner).
 *
 * Annee suivante : copier ce fichier en StrategyActions2027.{hpp,cpp},
 * ajuster les actions/zones puis mettre a jour l'include dans
 * O_State_DecisionMakerIA.cpp. L'infrastructure (ActionRegistry,
 * StrategyJsonRunner, IAbyPath) reste inchangee.
 */
void registerStrategyActions2026(ActionRegistry& registry, OPOS6UL_RobotExtended& robot);

/*!
 * \brief Declare les zones d'activite du terrain et leurs actions associees
 *        (via IAbyPath::ia_createZone + ia_addAction).
 *
 * \param robot    robot cible (sert a acceder a IAbyPath).
 * \param strategy nom de strategie (ex: "all", "tabletest") recu via robot.strategy().
 *                 "tabletest" active un layout decale pour tests sur petite table.
 *
 * Egalement 2026-specifique : les coordonnees, la liste des zones et les
 * callbacks lies sont figes dans StrategyActions2026.cpp.
 */
void setupActivitiesZone2026(OPOS6UL_RobotExtended& robot, const std::string& strategy);

/*!
 * \brief Manipulation push generique (avance pousse + recul de degagement).
 *
 * Logique : selon couleur, applique le swap idx (SWAP_COLOR_IDX) et le flip
 * de suffixe horizontal (P3/P4/P13/P14 : sensInverse <-> non-inverse en YELLOW),
 * lit dist dans distDirecte/distInverse, ajoute zoneOffset, puis avance la
 * distance + recul D_RETREAT. Cf doc PUSH_ELEMENTS_2026.md.
 *
 * \param backward Si true, pousse rear-first (arriere) : nav.line(-dist) au
 *                 lieu de nav.line(+dist), face arriere a 130mm du centre
 *                 (vs 115mm cote avant), et ajustement empirique -25mm sur
 *                 la distance brute pour calibration. Le faceTo initial du
 *                 robot reste a la charge de la strategie (idem forward).
 *                 Default false = comportement historique forward inchange.
 *
 * \return true si l'avance a abouti, false si abort avant pousse.
 */
bool push_elements_zone(uint8_t pickupIdx, const char* zoneName, bool sensInverse, bool backward = false);

/*!
 * \brief API exposee pour O_PushElementsTest (pas pour usage strategie).
 *
 * Permet au test C++ de valider la logique de combinaison
 * (mapping idx, swap couleur, flip suffixe horizontal, offsets P4/P14)
 * sans dependre des valeurs de calibration D1..D4 (le test compare a
 * distDirecteAt(idx) etc., donc reste vert quand on retouche la calibration).
 */
namespace push_elements_test_api {

/// Symetrie couleur appliquee sur l'idx pickup en YELLOW : (0,1)(2,3)(4,5).
extern const uint8_t SWAP_COLOR_IDX[6];

/// Lit la table directe a l'index idx (0..5). Retourne <0 si idx invalide.
float distDirecteAt(uint8_t idx);

/// Lit la table inverse a l'index idx (0..5). Retourne <0 si idx invalide.
float distInverseAt(uint8_t idx);

/// Offset specifique de la zone (kZoneOffset_P4 / _P14). Retourne 0 sinon.
float zoneOffsetFor(const char* zoneName);

/*!
 * \brief Override du terme dist[idx] par zone PHYSIQUE + config.
 *
 * Cherche zoneName dans la table kZoneDistOverride. Si une cellule est definie
 * pour la config idx (0..5), retourne sa valeur (elle remplace distDirecte/
 * distInverse[idx]) ; sinon retourne la sentinelle kNoOverride (= 1e9).
 * zoneName == nullptr ou idx > 5 -> kNoOverride.
 *
 * L'idx attendu est l'idx POST-swap (meme indexation que distDirecte/Inverse).
 * Cf robot/md/PUSH_ELEMENTS_2026.md.
 */
float distOverrideFor(const char* zoneName, uint8_t idx);

/// Constante D_RETREAT du recul de degagement post-pousse.
float retreatMm();

/// Constante D_BASE = distance de base de la pousse (mm), ajoutee a dist[idx].
float dBaseMm();

/*!
 * \brief Calcul pur de la distance d'avance, sans bouger le robot.
 *
 * Reproduit exactement la logique de combinaison de push_elements_zone() :
 *   yellow ? swap_idx + flip_horiz_suffix : passthrough
 *   distDirecteAt or distInverseAt + zoneOffsetFor.
 *
 * \return distance en mm (>0) ou <0 sur erreur (idx invalide ou
 *         dist resultante <= 0 apres offset).
 */
float computeDistance(uint8_t pickupIdx, const char* zoneName,
                      bool sensInverse, bool yellow);

/// True si la zone est horizontale (P3, P4, P13, P14) -> suffixe se flip en YELLOW.
bool isHorizontalZone(const char* zoneName);

/*!
 * \brief Resolution du pickup_P{N} adapte a la couleur match.
 *
 * En BLEU : retourne la valeur pickupP{N}() de la zone demandee.
 * En YELLOW : retourne la valeur de la zone miroir physique
 * (P1<->P11, P2<->P12, P3<->P13, P4<->P14), car le miroir Asserv place le
 * robot sur la zone miroir de la table. C'est la 3eme transformation de la
 * convention couleur (cf PUSH_ELEMENTS_2026.md), en plus du swap idx et du
 * flip suffixe horizontal.
 *
 * \return idx 0..5 lu sur le robot pour la zone resolue, ou 0 si zone inconnue.
 */
uint8_t resolvePickupForZone(const char* zoneName, bool yellow);

} // namespace push_elements_test_api

#endif
