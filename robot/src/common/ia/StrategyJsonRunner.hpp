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
 * \brief Resultat d'execution d'une instruction (cf. STRATEGY_DECISION_RUNNER.md).
 *
 * Populated par run() : un element par instruction de la strategie, dans
 * l'ordre chronologique d'evaluation (= ordre priority desc apres parser).
 */
struct InstructionOutcome
{
    enum class Status {
        FINISHED,             ///< Toutes les tasks OK ; flags leves/effaces.
        SKIPPED_PRIORITY,     ///< instr.priority < 0 (desactivee).
        SKIPPED_NEEDED_FLAG,  ///< needed_flag absent.
        SKIPPED_MAX_TIME,     ///< max_match_sec depasse (gate haut chrono).
        SKIPPED_OBSTACLE,     ///< TRAJ_NEAR_OBSTACLE (adv bloque, retries epuisses).
        SKIPPED_COLLISION,    ///< TRAJ_COLLISION (asserv bloquee).
        SKIPPED_IMPOSSIBLE,   ///< TRAJ_IMPOSSIBLE (A* introuvable, cmd refusee).
        SKIPPED_ERROR         ///< TRAJ_ERROR (timeout cmd, ack manquant) ou inconnu.
    };
    int id = 0;
    Status status = Status::FINISHED;
    TRAJ_STATE lastTs = TRAJ_IDLE;  ///< ts brut renvoye par executeInstruction si SKIPPED_*.
};

/*!
 * \brief Execute un fichier de strategie JSON via Navigator + Robot.
 *
 * Le parsing proprement dit est delegue a parseStrategyFromFile() (cf.
 * StrategyJsonParser.hpp), isole des dependances runtime pour pouvoir etre
 * teste en common-test.
 *
 * Comportement (cf. robot/md/STRATEGY_DECISION_RUNNER.md) : skip+continue.
 * Une instruction qui echoue (TRAJ != TRAJ_FINISHED) est sautee, on enchaine
 * avec la suivante. La trace complete est disponible via outcomes() apres run().
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
     * \brief Boucle d'execution skip+continue. Toujours true une fois la
     * boucle consommee ; la distinction succes/skip se lit via outcomes() ou
     * countByStatus(). Retourne false uniquement si JSON non charge prealable.
     */
    bool run();

    /*!
     * \brief Execute un tableau de tasks isole (sans wrapper instruction).
     * Utile pour les setpos_tasks d'init<Name>.json : on construit en interne
     * une pseudo-instruction (id=0, desc=label) et on la passe au moteur.
     * Garde la semantique abort : retourne false si la sequence n'aboutit pas
     * a TRAJ_FINISHED (l'appelant -- O_State_NewInit -- doit pouvoir reagir).
     */
    bool runTasks(const std::vector<StrategyTask>& tasks, const std::string& label = "tasks");

    const std::vector<StrategyInstruction>& instructions() const { return instructions_; }

    /*!
     * \brief Resultats de la derniere execution de run(). Vide avant run().
     */
    const std::vector<InstructionOutcome>& outcomes() const { return outcomes_; }

    /*!
     * \brief Compte les outcomes du status donne dans la derniere execution.
     */
    size_t countByStatus(InstructionOutcome::Status s) const;

    /*!
     * \brief Validation anticipee des coordonnees de la strategie chargee
     *        contre les zones PERMANENTES du playground (bordures, grenier,
     *        depart adverse). A appeler apres loadFromFile() et apres
     *        IAbyPath::initPlayground(), AVANT le match.
     *
     * Retourne true si toutes les coordonnees (position, face, waypoints)
     * sont hors zones permanentes. Si false, des erreurs precises ont ete
     * loggees (instruction, task, champ) et la strategie ne doit pas etre
     * lancee : une cible dans une zone permanente est definitivement
     * inatteignable et provoquerait un SK_IMPS au runtime.
     */
    bool validateAgainstPlayground(IAbyPath* iap) const;

private:
    static inline const logs::Logger& logger()
    {
        static const logs::Logger& instance = logs::LoggerFactory::logger("StrategyJsonRunner");
        return instance;
    }

    /*!
     * \brief Execute une instruction (ses tasks). Retourne TRAJ_FINISHED si
     * toutes les tasks ont reussi (et leve alors action_flag / clear_flags),
     * sinon le ts brut de la task ayant echoue (TRAJ_NEAR_OBSTACLE,
     * TRAJ_IMPOSSIBLE, TRAJ_COLLISION, TRAJ_ERROR). Les checks priority<0 /
     * needed_flag d'instruction sont gerees une marche au-dessus dans run().
     */
    TRAJ_STATE executeInstruction(const StrategyInstruction& instr);
    TRAJ_STATE executeTask(const StrategyTask& task);

    /*!
     * \brief Bloque tant que le chrono match du Robot < target_sec.
     *        No-op (avec log warn) si t deja >= target_sec.
     *        Poll a 50ms : assez reactif sans saturer le CPU.
     */
    void waitUntilMatchSec(float target_sec);

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
    std::vector<InstructionOutcome>  outcomes_;
    std::string path_;
};

#endif
