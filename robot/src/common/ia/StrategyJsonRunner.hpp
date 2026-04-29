#ifndef COMMON_IA_STRATEGY_JSON_RUNNER_HPP_
#define COMMON_IA_STRATEGY_JSON_RUNNER_HPP_

#include <string>
#include <vector>

#include "ia/StrategyJsonParser.hpp"
#include "interface/AAsservDriver.hpp"
#include "log/LoggerFactory.hpp"

class Robot;
class IAbyPath;
class ActionRegistry;
class FlagManager;

/*!
 * \brief Execute un fichier de strategie JSON via Navigator + Robot.
 *
 * Le parsing proprement dit est delegue a parseStrategyFromFile() (cf.
 * StrategyJsonParser.hpp), isole des dependances runtime pour pouvoir etre
 * teste en common-test.
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
    StrategyJsonRunner(Robot* robot, IAbyPath* iap, ActionRegistry* actions = nullptr,
                       FlagManager* flags = nullptr);

    bool loadFromFile(const std::string& path);
    /*!
     * \brief Boucle d'execution. Retourne true si toutes les instructions ont
     * ete traitees (exec OK ou skip legitime), false si abort sur echec d'une
     * instruction (comportement Phase 1 : premier echec stoppe le run).
     */
    bool run();

    /*!
     * \brief Execute un tableau de tasks isole (sans wrapper instruction).
     * Utile pour les setpos_tasks d'init<Name>.json : on construit en interne
     * une pseudo-instruction (id=0, desc=label) et on la passe au moteur.
     */
    bool runTasks(const std::vector<StrategyTask>& tasks, const std::string& label = "tasks");

    const std::vector<StrategyInstruction>& instructions() const { return instructions_; }

private:
    static inline const logs::Logger& logger()
    {
        static const logs::Logger& instance = logs::LoggerFactory::logger("StrategyJsonRunner");
        return instance;
    }

    bool executeInstruction(const StrategyInstruction& instr);
    TRAJ_STATE executeTask(const StrategyTask& task);

    /*!
     * \brief Variante de executeTask qui envoie la cmd au driver SANS attendre
     *        sa fin (mode chain : la Nucleo empile dans sa queue motion).
     *        Retourne false si le subtype n'est pas chainable. La detection
     *        ToF/balise est suspendue jusqu'au prochain wait. Subtypes pris en
     *        charge : LINE, GO_TO, GO_BACK_TO, FACE_TO, FACE_BACK_TO,
     *        ROTATE_DEG, ROTATE_ABS_DEG, ORBITAL_TURN_DEG. Refuse les
     *        composites et PATH_TO* (necessitent du calcul/wait cote OPOS6UL).
     */
    bool executeTaskSendOnly(const StrategyTask& task);

    Robot*          robot_;
    IAbyPath*       iap_;
    ActionRegistry* actions_;   ///< Optional : nullptr -> MANIPULATION loggue comme stub.
    FlagManager*    flags_;     ///< Optional : nullptr -> needed_flag/action_flag/clear_flags ignores.
    std::vector<StrategyInstruction> instructions_;
    std::string path_;
};

#endif
