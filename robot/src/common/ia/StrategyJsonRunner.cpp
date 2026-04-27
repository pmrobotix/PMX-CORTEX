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
    StrategyInstruction instr;
    instr.id = 0;
    instr.desc = label;
    instr.tasks = tasks;
    return executeInstruction(instr);
}

bool StrategyJsonRunner::run()
{
    logger().info() << "run: start (" << instructions_.size() << " instructions)" << logs::end;
    for (const auto& instr : instructions_) {
        if (!executeInstruction(instr)) {
            logger().error() << "run: instruction id=" << instr.id << " FAILED, abort" << logs::end;
            return false;
        }
    }
    logger().info() << "run: done" << logs::end;
    return true;
}

bool StrategyJsonRunner::executeInstruction(const StrategyInstruction& instr)
{
    if (instr.priority < 0.0f) {
        logger().info() << "[instr " << instr.id << "] SKIP (priority="
                        << instr.priority << " < 0 -> desactivee)" << logs::end;
        return true;
    }
    if (flags_ && instr.needed_flag && !flags_->has(*instr.needed_flag)) {
        logger().info() << "[instr " << instr.id << "] SKIP (needed_flag '"
                        << *instr.needed_flag << "' not set)" << logs::end;
        return true;
    }

    logger().info() << "[instr " << instr.id << "] " << instr.desc << logs::end;
    bool firstTask = true;
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
        // 200ms = 2 frames CBOR (la Nucleo emet a 10 Hz).
        if (!firstTask) {
            utils::sleep_for_micros(200000);
        }
        firstTask = false;
        TRAJ_STATE ts = executeTask(task);
        if (ts != TRAJ_FINISHED) {
            logger().error() << "[instr " << instr.id << "] task "
                             << task.type << "/" << task.subtype
                             << " -> ts=" << ts << logs::end;
            return false;
        }
    }

    if (flags_) {
        if (instr.action_flag) flags_->set(*instr.action_flag);
        for (const auto& f : instr.clear_flags) flags_->clear(f);
    }
    return true;
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
    if (t.type == "WAIT" && t.duration_ms) {
        std::this_thread::sleep_for(std::chrono::milliseconds(*t.duration_ms));
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
