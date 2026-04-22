#include "StrategyJsonRunner.hpp"

#include <chrono>
#include <fstream>
#include <thread>

#include "asserv/Asserv.hpp"
#include "ia/ActionRegistry.hpp"
#include "ia/IAbyPath.hpp"
#include "interface/AAsservDriver.hpp"
#include "navigator/Navigator.hpp"
#include "Robot.hpp"
#include "utils/json.hpp"

using json = nlohmann::json;

namespace
{

template <typename T>
static std::optional<T> optField(const json& j, const char* key)
{
    auto it = j.find(key);
    if (it == j.end() || it->is_null()) return std::nullopt;
    return it->get<T>();
}

static StrategyTask parseTask(const json& j)
{
    StrategyTask t;
    t.type    = j.value("type", "");
    t.subtype = j.value("subtype", "");
    t.desc          = optField<std::string>(j, "desc");
    t.needed_flag   = optField<std::string>(j, "needed_flag");
    t.timeout_ms    = j.value("timeout", -1);

    t.position_x    = optField<float>(j, "position_x");
    t.position_y    = optField<float>(j, "position_y");
    t.dist          = optField<float>(j, "dist");
    t.angle_deg     = optField<float>(j, "angle_deg");
    t.final_angle_deg = optField<float>(j, "final_angle_deg");
    t.rotate_rel_deg  = optField<float>(j, "rotate_rel_deg");
    t.face_x        = optField<float>(j, "face_x");
    t.face_y        = optField<float>(j, "face_y");
    t.forward       = optField<bool>(j, "forward");
    t.turn_right    = optField<bool>(j, "turn_right");
    t.action_id     = optField<std::string>(j, "action_id");
    t.item_id       = optField<std::string>(j, "item_id");
    t.speed_percent = optField<int>(j, "speed_percent");
    t.duration_ms   = optField<int>(j, "duration_ms");

    auto wpIt = j.find("waypoints");
    if (wpIt != j.end() && wpIt->is_array()) {
        for (const auto& w : *wpIt) {
            if (w.is_array() && w.size() >= 2) {
                t.waypoints.push_back({w[0].get<float>(), w[1].get<float>()});
            }
        }
    }
    return t;
}

static StrategyInstruction parseInstruction(const json& j)
{
    StrategyInstruction instr;
    instr.id   = j.value("id", 0);
    instr.desc = j.value("desc", "");
    instr.needed_flag = optField<std::string>(j, "needed_flag");
    instr.action_flag = optField<std::string>(j, "action_flag");
    auto cf = j.find("clear_flags");
    if (cf != j.end() && cf->is_array()) {
        for (const auto& s : *cf) instr.clear_flags.push_back(s.get<std::string>());
    }
    auto tt = j.find("tasks");
    if (tt != j.end() && tt->is_array()) {
        for (const auto& t : *tt) instr.tasks.push_back(parseTask(t));
    }
    return instr;
}

} // namespace

StrategyJsonRunner::StrategyJsonRunner(Robot* robot, IAbyPath* iap, ActionRegistry* actions)
    : robot_(robot), iap_(iap), actions_(actions)
{
}

bool StrategyJsonRunner::loadFromFile(const std::string& path)
{
    path_ = path;
    std::ifstream f(path);
    if (!f.is_open()) {
        logger().error() << "loadFromFile: cannot open " << path << logs::end;
        return false;
    }

    json root;
    try {
        f >> root;
    } catch (const std::exception& e) {
        logger().error() << "loadFromFile: parse error in " << path << " : " << e.what() << logs::end;
        return false;
    }

    if (!root.is_array()) {
        logger().error() << "loadFromFile: root must be an array in " << path << logs::end;
        return false;
    }

    instructions_.clear();
    for (const auto& j : root) {
        instructions_.push_back(parseInstruction(j));
    }
    logger().info() << "Loaded " << instructions_.size() << " instructions from " << path << logs::end;
    return true;
}

void StrategyJsonRunner::run()
{
    logger().info() << "run: start (" << instructions_.size() << " instructions)" << logs::end;
    for (const auto& instr : instructions_) {
        if (!executeInstruction(instr)) {
            logger().error() << "run: instruction id=" << instr.id << " FAILED, abort" << logs::end;
            return;
        }
    }
    logger().info() << "run: done" << logs::end;
}

bool StrategyJsonRunner::executeInstruction(const StrategyInstruction& instr)
{
    logger().info() << "[instr " << instr.id << "] " << instr.desc << logs::end;
    for (const auto& task : instr.tasks) {
        TRAJ_STATE ts = executeTask(task);
        if (ts != TRAJ_FINISHED) {
            logger().error() << "[instr " << instr.id << "] task "
                             << task.type << "/" << task.subtype
                             << " -> ts=" << ts << logs::end;
            return false;
        }
    }
    return true;
}

TRAJ_STATE StrategyJsonRunner::executeTask(const StrategyTask& t)
{
    logger().info() << "  task " << t.type << "/" << t.subtype
                    << (t.desc ? (" (" + *t.desc + ")") : std::string()) << logs::end;

    Navigator nav(robot_, iap_);

    if (t.type == "MOVEMENT") {
        if (t.subtype == "LINE" && t.dist) {
            return nav.line(*t.dist);
        }
        if (t.subtype == "GO_TO" && t.position_x && t.position_y) {
            return nav.goTo(*t.position_x, *t.position_y);
        }
        if (t.subtype == "PATH_TO" && t.position_x && t.position_y) {
            return nav.pathTo(*t.position_x, *t.position_y);
        }
        if (t.subtype == "FACE_TO" && t.position_x && t.position_y) {
            return nav.faceTo(*t.position_x, *t.position_y);
        }
        if (t.subtype == "ROTATE_DEG" && t.angle_deg) {
            return nav.rotateDeg(*t.angle_deg);
        }
        if (t.subtype == "MOVE_FORWARD_TO_AND_ROTATE_ABS_DEG"
            && t.position_x && t.position_y && t.final_angle_deg) {
            return nav.moveForwardToAndRotateAbsDeg(*t.position_x, *t.position_y, *t.final_angle_deg);
        }
        logger().error() << "MOVEMENT subtype unsupported (Phase 1): " << t.subtype << logs::end;
        return TRAJ_ERROR;
    }
    if (t.type == "WAIT" && t.duration_ms) {
        std::this_thread::sleep_for(std::chrono::milliseconds(*t.duration_ms));
        return TRAJ_FINISHED;
    }
    if (t.type == "SPEED" && t.subtype == "SET_SPEED" && t.speed_percent) {
        robot_->asserv().setMaxSpeed(true, *t.speed_percent, *t.speed_percent);
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
            logger().error() << "  MANIPULATION unknown action_id='" << aid
                             << "' - skipping (not fatal)" << logs::end;
            return TRAJ_FINISHED;
        }
        bool ok = actions_->call(aid);
        return ok ? TRAJ_FINISHED : TRAJ_ERROR;
    }

    logger().error() << "Unknown task type: " << t.type << logs::end;
    return TRAJ_ERROR;
}
