#ifndef BOT_OPOS6UL_STRATEGY_ACTIONS_2026_HPP_
#define BOT_OPOS6UL_STRATEGY_ACTIONS_2026_HPP_

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

#endif
