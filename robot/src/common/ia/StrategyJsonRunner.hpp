#ifndef COMMON_IA_STRATEGY_JSON_RUNNER_HPP_
#define COMMON_IA_STRATEGY_JSON_RUNNER_HPP_

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "interface/AAsservDriver.hpp"
#include "log/LoggerFactory.hpp"

class Robot;
class IAbyPath;
class ActionRegistry;

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
};

/*!
 * \brief Execute un fichier de strategie JSON via Navigator + Robot.
 *
 * Phase 1 : subtypes MOVEMENT = LINE, GO_TO, PATH_TO, FACE_TO, ROTATE_DEG,
 * MOVE_FORWARD_TO_AND_ROTATE_ABS_DEG. Autres types : WAIT, SPEED/SET_SPEED.
 * ELEMENT et MANIPULATION loggent comme stubs (Phase 2 : ActionRegistry).
 *
 * En cas de JSON malforme ou d'erreur fatale : log + arret propre (l'appelant
 * fera freeMotion()).
 */
class StrategyJsonRunner
{
public:
    StrategyJsonRunner(Robot* robot, IAbyPath* iap, ActionRegistry* actions = nullptr);

    bool loadFromFile(const std::string& path);
    void run();

private:
    static inline const logs::Logger& logger()
    {
        static const logs::Logger& instance = logs::LoggerFactory::logger("StrategyJsonRunner");
        return instance;
    }

    bool executeInstruction(const StrategyInstruction& instr);
    TRAJ_STATE executeTask(const StrategyTask& task);

    Robot*          robot_;
    IAbyPath*       iap_;
    ActionRegistry* actions_;   ///< Optional : nullptr -> MANIPULATION loggue comme stub.
    std::vector<StrategyInstruction> instructions_;
    std::string path_;
};

#endif
