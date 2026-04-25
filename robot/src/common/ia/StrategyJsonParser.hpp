#ifndef COMMON_IA_STRATEGY_JSON_PARSER_HPP_
#define COMMON_IA_STRATEGY_JSON_PARSER_HPP_

#include <array>
#include <optional>
#include <string>
#include <vector>

/*!
 * \brief Une task JSON (unite d'execution atomique).
 *
 * Tous les champs optionnels, remplis selon le couple (type, subtype).
 * Voir robot/md/STRATEGY_JSON_FORMAT.md pour la spec complete.
 */
struct StrategyTask
{
    std::string type;                       // MOVEMENT, MANIPULATION, ELEMENT, SPEED, WAIT
    std::string subtype;                    // GO_TO, LINE, PATH_TO, ...
    std::optional<std::string> desc;
    std::optional<std::string> needed_flag;
    int timeout_ms = -1;

    std::optional<float> position_x, position_y;
    std::optional<float> dist;
    std::optional<float> angle_deg;
    std::optional<float> final_angle_deg, rotate_rel_deg;
    std::optional<float> face_x, face_y;
    std::optional<bool>  forward, turn_right;
    std::optional<std::string> action_id;
    std::optional<std::string> item_id;
    std::optional<int>   speed_percent;
    std::optional<int>   duration_ms;
    std::vector<std::array<float, 2>> waypoints;
};

/*!
 * \brief Bloc d'instructions : sequence de tasks avec metadonnees.
 */
struct StrategyInstruction
{
    int id = 0;
    std::string desc;
    std::vector<StrategyTask> tasks;
    std::optional<std::string> needed_flag;
    std::optional<std::string> action_flag;
    std::vector<std::string> clear_flags;

    // Metadonnees pour la couche decision (phase 7).
    // priority : plus eleve = choisi en premier. Defaut 0 = neutre. < 0 = desactivee.
    float priority = 0.0f;
    std::optional<int>   points;
    std::optional<float> estimatedDurationSec;
};

/*!
 * \brief Parse un fichier de strategie JSON et trie par priorite descendante.
 *
 * \param path chemin du fichier JSON a charger
 * \param out vecteur d'instructions en sortie (vide si echec ou JSON vide)
 * \return true si le parsing a reussi, false sinon (fichier absent, JSON malforme...)
 *
 * Apres succes, \p out contient les instructions triees par priority descendante
 * (stable_sort : ordre JSON d'origine preserve a priorite egale).
 *
 * Aucune dependance runtime (Robot, Asserv, Navigator) : utilisable en test
 * unitaire isole.
 */
bool parseStrategyFromFile(const std::string& path, std::vector<StrategyInstruction>& out);

/*!
 * \brief Donnees lues depuis init<Name>.json.
 *
 * Champs obligatoires : x, y, theta (rad). Le parser convertit theta en degres.
 * Champ optionnel : setpos_tasks[] (tasks jouees AVANT la tirette par
 * O_State_NewInit::setPos()).
 *
 * Defaults : (300, 130, 90 deg, []) si fichier minimal.
 */
struct InitData
{
    float x = 300.0f;
    float y = 130.0f;
    float thetaDeg = 90.0f;
    std::vector<StrategyTask> setposTasks;
};

bool parseInitFromFile(const std::string& path, InitData& out);

#endif
