#include "StrategyJsonRunner.hpp"

#include <chrono>
#include <thread>

#include "asserv/Asserv.hpp"
#include "ia/ActionRegistry.hpp"
#include "ia/FlagManager.hpp"
#include "ia/IAbyPath.hpp"
#include "interface/AAsservDriver.hpp"
#include "navigator/Navigator.hpp"
#include "Robot.hpp"

StrategyJsonRunner::StrategyJsonRunner(Robot* robot, IAbyPath* iap, ActionRegistry* actions,
                                       FlagManager* flags)
    : robot_(robot), iap_(iap), actions_(actions), flags_(flags)
{
}

bool StrategyJsonRunner::loadFromFile(const std::string& path)
{
    path_ = path;
    if (!parseStrategyFromFile(path, instructions_)) {
        logger().error() << "loadFromFile: parseStrategyFromFile failed for " << path << logs::end;
        return false;
    }
    logger().info() << "Loaded " << instructions_.size() << " instructions from " << path
                    << logs::end;
    return true;
}

bool StrategyJsonRunner::runTasks(const std::vector<StrategyTask>& tasks, const std::string& label)
{
    // Pseudo-instruction synthetique pour reutiliser le moteur d'execution.
    // Pas de priority/needed_flag/min_match_sec/max_match_sec → executeInstruction
    // se concentre uniquement sur les tasks. Semantique abort conservee :
    // false si non TRAJ_FINISHED, pour que O_State_NewInit puisse reagir.
    StrategyInstruction instr;
    instr.id = 0;
    instr.desc = label;
    instr.tasks = tasks;
    return executeInstruction(instr) == TRAJ_FINISHED;
}

bool StrategyJsonRunner::run()
{
    outcomes_.clear();
    outcomes_.reserve(instructions_.size());
    logger().info() << "run: start (" << instructions_.size() << " instructions)" << logs::end;

    for (const auto& instr : instructions_) {
        InstructionOutcome o;
        o.id = instr.id;

        if (instr.priority < 0.0f) {
            o.status = InstructionOutcome::Status::SKIPPED_PRIORITY;
            outcomes_.push_back(o);
            logger().info() << "[instr " << instr.id << "] SKIP (priority="
                            << instr.priority << " < 0 -> desactivee)" << logs::end;
            continue;
        }
        if (flags_ && instr.needed_flag && !flags_->has(*instr.needed_flag)) {
            o.status = InstructionOutcome::Status::SKIPPED_NEEDED_FLAG;
            outcomes_.push_back(o);
            logger().info() << "[instr " << instr.id << "] SKIP (needed_flag '"
                            << *instr.needed_flag << "' not set)" << logs::end;
            continue;
        }
        // Gate haut chrono : si max_match_sec depasse, on skip.
        if (instr.max_match_sec) {
            float t = robot_->chrono().getElapsedTimeInSec();
            if (t >= *instr.max_match_sec) {
                o.status = InstructionOutcome::Status::SKIPPED_MAX_TIME;
                outcomes_.push_back(o);
                logger().info() << "[instr " << instr.id << "] SKIP (max_match_sec="
                                << *instr.max_match_sec << " depasse, t=" << t << ")"
                                << logs::end;
                continue;
            }
        }

        TRAJ_STATE ts = executeInstruction(instr);
        o.lastTs = ts;
        if (ts == TRAJ_FINISHED) {
            o.status = InstructionOutcome::Status::FINISHED;
            outcomes_.push_back(o);
            continue;
        }

        switch (ts) {
            case TRAJ_NEAR_OBSTACLE:
                o.status = InstructionOutcome::Status::SKIPPED_OBSTACLE; break;
            case TRAJ_COLLISION:
                o.status = InstructionOutcome::Status::SKIPPED_COLLISION; break;
            case TRAJ_IMPOSSIBLE:
                o.status = InstructionOutcome::Status::SKIPPED_IMPOSSIBLE; break;
            default:
                o.status = InstructionOutcome::Status::SKIPPED_ERROR; break;
        }
        outcomes_.push_back(o);
        logger().warn() << "[instr " << instr.id << "] SKIP ts=" << ts
                        << " -> instruction suivante" << logs::end;

        // Apres un echec trajectoire, l'emergencyStop_ Asserv reste actif
        // (cf. Navigator::executeWithRetry qui ne reset pas sur le dernier
        // essai, volontairement, pour laisser la strat decider). Sans ce
        // reset, l'instruction suivante echouerait immediatement (Nucleo
        // STOP detecte). On reprend ici la main pour la suite du run.
        robot_->asserv().resetEmergencyOnTraj("StrategyJsonRunner skip instruction");
    }

    size_t finished = countByStatus(InstructionOutcome::Status::FINISHED);
    logger().info() << "run: done. finished=" << finished
                    << "/" << instructions_.size() << logs::end;
    return true;
}

size_t StrategyJsonRunner::countByStatus(InstructionOutcome::Status s) const
{
    size_t n = 0;
    for (const auto& o : outcomes_) if (o.status == s) ++n;
    return n;
}

bool StrategyJsonRunner::validateAgainstPlayground(IAbyPath* iap) const
{
    if (iap == nullptr) {
        logger().error() << "validateAgainstPlayground: iap NULL" << logs::end;
        return false;
    }
    int errors = 0;
    auto checkXY = [&](float x, float y, const char* what,
                       int instrId, size_t taskIdx) {
        if (iap->pointInPermanentZone(x, y)) {
            logger().error() << "validateAgainstPlayground: "
                             << what << "=(" << x << "," << y << ") DANS zone PERMANENTE"
                             << " (instr=" << instrId << " taskIdx=" << taskIdx
                             << ") -> cible inatteignable" << logs::end;
            ++errors;
        }
    };
    for (const auto& instr : instructions_) {
        for (size_t k = 0; k < instr.tasks.size(); k++) {
            const auto& t = instr.tasks[k];
            if (t.position_x && t.position_y) {
                checkXY(*t.position_x, *t.position_y, "position", instr.id, k);
            }
            if (t.face_x && t.face_y) {
                checkXY(*t.face_x, *t.face_y, "face", instr.id, k);
            }
            for (size_t i = 0; i < t.waypoints.size(); i++) {
                checkXY(t.waypoints[i][0], t.waypoints[i][1], "waypoint",
                        instr.id, k);
            }
        }
    }
    if (errors > 0) {
        logger().error() << "validateAgainstPlayground: " << errors
                         << " coord(s) hors permanent zone -> ABORT" << logs::end;
        return false;
    }
    logger().info() << "validateAgainstPlayground: OK ("
                    << instructions_.size() << " instructions)" << logs::end;
    return true;
}

TRAJ_STATE StrategyJsonRunner::executeInstruction(const StrategyInstruction& instr)
{
    // Gate bas chrono : attend que le chrono atteigne min_match_sec avant de
    // demarrer. Les checks priority<0 / needed_flag / max_match_sec sont
    // gerees une marche au-dessus dans run() pour pouvoir distinguer les
    // statuts d'outcome. runTasks() passe une instr synthetique sans gate.
    if (instr.min_match_sec) {
        waitUntilMatchSec(*instr.min_match_sec);
    }

    logger().info() << "[instr " << instr.id << "] " << instr.desc << logs::end;
    bool firstTask = true;
    bool prevChain = false;
    for (const auto& task : instr.tasks) {
        if (flags_ && task.needed_flag && !flags_->has(*task.needed_flag)) {
            logger().info() << "  task " << task.type << "/" << task.subtype
                            << " SKIP (needed_flag '" << *task.needed_flag << "' not set)"
                            << logs::end;
            continue;
        }
        // Petit sleep entre 2 tasks consecutives : la Nucleo CBOR peut perdre
        // une commande envoyee trop vite apres la fin de la precedente
        // (overflow buffer Rx pendant que onTimer asserv finalise la regulation).
        // Symptome sans ce sleep : target_cmd=N+1 received_cmd=N -> warn ACK
        // dans Asserv::sendCborMotionWithRetry, puis cmd appliquee tardivement
        // par la Nucleo causant des comportements erratiques.
        // 50 ms = ~1/2 frame CBOR a 10 Hz : assez court pour limiter la
        // derive d'asserv residuelle entre 2 cmds (qui se reporte sur LINE
        // / GO_TO suivants), assez long pour eviter l'overflow Rx. A monter
        // si "no ACK" reapparait dans les logs sendCborMotionWithRetry.
        // chain -> chain : sleep raccourci a 20 ms (la Nucleo empile dans sa
        // queue, pas de fenetre IDLE intermediaire a respecter).
        if (!firstTask) {
            const int sleep_us = (prevChain && task.chain) ? 20000 : 50000;
            utils::sleep_for_micros(sleep_us);
        }
        firstTask = false;

        if (task.chain) {
            if (!executeTaskSendOnly(task)) {
                logger().error() << "[instr " << instr.id << "] task chain "
                                 << task.type << "/" << task.subtype
                                 << " send FAILED" << logs::end;
                return TRAJ_ERROR;
            }
            prevChain = true;
            continue;
        }

        TRAJ_STATE ts = executeTask(task);
        if (ts != TRAJ_FINISHED) {
            logger().error() << "[instr " << instr.id << "] task "
                             << task.type << "/" << task.subtype
                             << " -> ts=" << ts << logs::end;
            return ts;
        }
        prevChain = false;
    }

    // Si la derniere task etait chainee, attendre la fin de la queue Nucleo
    // (status IDLE + cmd_id ACK). Type derive de la derniere task pour
    // activer la detection FORWARD/BACKWARD durant ce wait final.
    if (prevChain) {
        const StrategyTask& last = instr.tasks.back();
        Asserv::MovementType type = Asserv::ROTATION;
        if (last.subtype == "LINE" && last.dist) {
            type = (*last.dist >= 0.0f) ? Asserv::FORWARD : Asserv::BACKWARD;
        } else if (last.subtype == "GO_TO" || last.subtype == "MOVE_FORWARD_TO") {
            type = Asserv::FORWARD;
        } else if (last.subtype == "GO_BACK_TO" || last.subtype == "MOVE_BACKWARD_TO") {
            type = Asserv::BACKWARD;
        }
        TRAJ_STATE ts = robot_->asserv().waitEndOfTrajWithDetection(type);
        if (ts != TRAJ_FINISHED) {
            logger().error() << "[instr " << instr.id << "] chain final wait -> ts="
                             << ts << logs::end;
            return ts;
        }
    }

    if (flags_) {
        if (instr.action_flag) flags_->set(*instr.action_flag);
        for (const auto& f : instr.clear_flags) flags_->clear(f);
    }
    return TRAJ_FINISHED;
}

TRAJ_STATE StrategyJsonRunner::executeTask(const StrategyTask& t)
{
    logger().info() << "  task " << t.type << "/" << t.subtype
                    << (t.desc ? (" (" + *t.desc + ")") : std::string()) << logs::end;

    Navigator nav(robot_, iap_);

    if (t.type == "MOVEMENT") {
        // --- Primitives ---
        if (t.subtype == "LINE" && t.dist) {
            return nav.line(*t.dist);
        }
        if (t.subtype == "GO_TO" && t.position_x && t.position_y) {
            return nav.goTo(*t.position_x, *t.position_y);
        }
        if (t.subtype == "GO_BACK_TO" && t.position_x && t.position_y) {
            return nav.goBackTo(*t.position_x, *t.position_y);
        }
        if (t.subtype == "MOVE_FORWARD_TO" && t.position_x && t.position_y) {
            return nav.moveForwardTo(*t.position_x, *t.position_y);
        }
        if (t.subtype == "MOVE_BACKWARD_TO" && t.position_x && t.position_y) {
            return nav.moveBackwardTo(*t.position_x, *t.position_y);
        }
        if (t.subtype == "PATH_TO" && t.position_x && t.position_y) {
            return nav.pathTo(*t.position_x, *t.position_y);
        }
        if (t.subtype == "PATH_BACK_TO" && t.position_x && t.position_y) {
            return nav.pathBackTo(*t.position_x, *t.position_y);
        }
        if (t.subtype == "FACE_TO" && t.position_x && t.position_y) {
            return nav.faceTo(*t.position_x, *t.position_y);
        }
        if (t.subtype == "FACE_BACK_TO" && t.position_x && t.position_y) {
            return nav.faceBackTo(*t.position_x, *t.position_y);
        }
        if (t.subtype == "ROTATE_DEG" && t.angle_deg) {
            return nav.rotateDeg(*t.angle_deg);
        }
        if (t.subtype == "ROTATE_ABS_DEG" && t.angle_deg) {
            return nav.rotateAbsDeg(*t.angle_deg);
        }
        if (t.subtype == "ORBITAL_TURN_DEG" && t.angle_deg && t.forward && t.turn_right) {
            return nav.orbitalTurnDeg(*t.angle_deg, *t.forward, *t.turn_right);
        }

        // --- Composites (deplacement puis rotation, abort si deplacement echoue) ---
        if (t.subtype == "GO_TO_AND_ROTATE_ABS_DEG"
            && t.position_x && t.position_y && t.final_angle_deg) {
            return nav.goToAndRotateAbsDeg(*t.position_x, *t.position_y, *t.final_angle_deg);
        }
        if (t.subtype == "GO_TO_AND_FACE_TO"
            && t.position_x && t.position_y && t.face_x && t.face_y) {
            return nav.goToAndFaceTo(*t.position_x, *t.position_y, *t.face_x, *t.face_y);
        }
        if (t.subtype == "GO_TO_AND_FACE_BACK_TO"
            && t.position_x && t.position_y && t.face_x && t.face_y) {
            return nav.goToAndFaceBackTo(*t.position_x, *t.position_y, *t.face_x, *t.face_y);
        }
        if (t.subtype == "MOVE_FORWARD_TO_AND_ROTATE_ABS_DEG"
            && t.position_x && t.position_y && t.final_angle_deg) {
            return nav.moveForwardToAndRotateAbsDeg(*t.position_x, *t.position_y, *t.final_angle_deg);
        }
        if (t.subtype == "MOVE_FORWARD_TO_AND_ROTATE_REL_DEG"
            && t.position_x && t.position_y && t.rotate_rel_deg) {
            return nav.moveForwardToAndRotateRelDeg(*t.position_x, *t.position_y, *t.rotate_rel_deg);
        }
        if (t.subtype == "MOVE_FORWARD_TO_AND_FACE_TO"
            && t.position_x && t.position_y && t.face_x && t.face_y) {
            return nav.moveForwardToAndFaceTo(*t.position_x, *t.position_y, *t.face_x, *t.face_y);
        }
        if (t.subtype == "MOVE_FORWARD_TO_AND_FACE_BACK_TO"
            && t.position_x && t.position_y && t.face_x && t.face_y) {
            return nav.moveForwardToAndFaceBackTo(*t.position_x, *t.position_y, *t.face_x, *t.face_y);
        }
        if (t.subtype == "PATH_TO_AND_ROTATE_ABS_DEG"
            && t.position_x && t.position_y && t.final_angle_deg) {
            return nav.pathToAndRotateAbsDeg(*t.position_x, *t.position_y, *t.final_angle_deg);
        }
        if (t.subtype == "PATH_TO_AND_FACE_TO"
            && t.position_x && t.position_y && t.face_x && t.face_y) {
            return nav.pathToAndFaceTo(*t.position_x, *t.position_y, *t.face_x, *t.face_y);
        }

        logger().error() << "MOVEMENT subtype unsupported : " << t.subtype << logs::end;
        return TRAJ_ERROR;
    }
    if (t.type == "WAIT") {
        if (t.until_match_sec) {
            waitUntilMatchSec(*t.until_match_sec);
            return TRAJ_FINISHED;
        }
        if (t.duration_ms) {
            std::this_thread::sleep_for(std::chrono::milliseconds(*t.duration_ms));
            return TRAJ_FINISHED;
        }
        logger().error() << "WAIT sans duration_ms ni until_match_sec : ignore" << logs::end;
        return TRAJ_FINISHED;
    }
    if (t.type == "SPEED" && t.subtype == "SET_SPEED" && t.speed_percent) {
        // API unifiee : delegue a Asserv::setSpeed. La semantique est
        // ajustable centralement dans Asserv::setSpeed (cap PWM seul,
        // scale acc/dec seul, ou les 2) sans modifier les appelants.
        robot_->asserv().setSpeed(*t.speed_percent);
        return TRAJ_FINISHED;
    }
    if (t.type == "ELEMENT") {
        logger().info() << "  [STUB Phase 2] ELEMENT/" << t.subtype
                        << " item_id=" << (t.item_id ? *t.item_id : "?") << logs::end;
        return TRAJ_FINISHED;
    }
    if (t.type == "MANIPULATION") {
        const std::string& aid = t.action_id ? *t.action_id : std::string("?");
        if (actions_ == nullptr) {
            logger().info() << "  [no ActionRegistry] MANIPULATION action_id=" << aid << logs::end;
            return TRAJ_FINISHED;
        }
        if (!actions_->has(aid)) {
            logger().error() << "  TODO MANIPULATION \"" << aid
                             << "\" dans StrategyActions2026.cpp." << logs::end;
            return TRAJ_FINISHED;
        }
        bool ok = actions_->call(aid);
        return ok ? TRAJ_FINISHED : TRAJ_ERROR;
    }

    logger().error() << "Unknown task type: " << t.type << logs::end;
    return TRAJ_ERROR;
}

bool StrategyJsonRunner::executeTaskSendOnly(const StrategyTask& t)
{
    logger().info() << "  task " << t.type << "/" << t.subtype << " [chain]"
                    << (t.desc ? (" (" + *t.desc + ")") : std::string()) << logs::end;

    if (t.type != "MOVEMENT") {
        logger().error() << "  chain: type " << t.type
                         << " non chainable (seul MOVEMENT primitif l'est)" << logs::end;
        return false;
    }

    Asserv& a = robot_->asserv();

    if (t.subtype == "LINE" && t.dist) {
        a.lineSend(*t.dist);
        return true;
    }
    if (t.subtype == "GO_TO" && t.position_x && t.position_y) {
        a.goToSend(*t.position_x, *t.position_y);
        return true;
    }
    if (t.subtype == "GO_BACK_TO" && t.position_x && t.position_y) {
        a.goBackToSend(*t.position_x, *t.position_y);
        return true;
    }
    if (t.subtype == "FACE_TO" && t.position_x && t.position_y) {
        a.faceToSend(*t.position_x, *t.position_y);
        return true;
    }
    if (t.subtype == "FACE_BACK_TO" && t.position_x && t.position_y) {
        a.faceBackToSend(*t.position_x, *t.position_y);
        return true;
    }
    if (t.subtype == "ROTATE_DEG" && t.angle_deg) {
        a.rotateDegSend(*t.angle_deg);
        return true;
    }
    if (t.subtype == "ROTATE_ABS_DEG" && t.angle_deg) {
        a.rotateAbsDegSend(*t.angle_deg);
        return true;
    }
    if (t.subtype == "ORBITAL_TURN_DEG" && t.angle_deg && t.forward && t.turn_right) {
        a.orbitalTurnDegSend(*t.angle_deg, *t.forward, *t.turn_right);
        return true;
    }

    // Composites (GO_TO_AND_*, MOVE_FORWARD_TO_AND_*) et PATH_TO* font du
    // calcul OPOS6UL ou des waits internes -> pas chainables tels quels.
    logger().error() << "  chain: subtype " << t.subtype
                     << " non chainable en V1 (composite ou path)" << logs::end;
    return false;
}

void StrategyJsonRunner::waitUntilMatchSec(float target_sec)
{
    auto& chrono = robot_->chrono();
    float t_now = chrono.getElapsedTimeInSec();
    if (t_now >= target_sec) {
        logger().warn() << "waitUntilMatchSec(" << target_sec
                        << "s) deja depasse (t=" << t_now << "s) -> skip"
                        << logs::end;
        return;
    }
    logger().info() << "waitUntilMatchSec : t=" << t_now << "s, target="
                    << target_sec << "s, attente " << (target_sec - t_now)
                    << "s" << logs::end;
    while (chrono.getElapsedTimeInSec() < target_sec) {
        utils::sleep_for_micros(50000);  // 50ms : reactif sans saturer
    }
}
