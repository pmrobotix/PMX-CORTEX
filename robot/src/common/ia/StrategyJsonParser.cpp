#include "StrategyJsonParser.hpp"

#include <algorithm>
#include <fstream>

#include "log/LoggerFactory.hpp"
#include "utils/json.hpp"

using json = nlohmann::json;

namespace
{

static const logs::Logger& logger()
{
    static const logs::Logger& instance = logs::LoggerFactory::logger("StrategyJsonParser");
    return instance;
}

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
    instr.priority             = j.value("priority", 0.0f);
    instr.points               = optField<int>(j, "points");
    instr.estimatedDurationSec = optField<float>(j, "estimatedDurationSec");
    auto tt = j.find("tasks");
    if (tt != j.end() && tt->is_array()) {
        for (const auto& t : *tt) instr.tasks.push_back(parseTask(t));
    }
    return instr;
}

} // namespace

bool parseStrategyFromFile(const std::string& path, std::vector<StrategyInstruction>& out)
{
    out.clear();

    std::ifstream f(path);
    if (!f.is_open()) {
        logger().error() << "parseStrategyFromFile: cannot open " << path << logs::end;
        return false;
    }

    json root;
    try {
        f >> root;
    } catch (const std::exception& e) {
        logger().error() << "parseStrategyFromFile: parse error in " << path << " : "
                         << e.what() << logs::end;
        return false;
    }

    if (!root.is_array()) {
        logger().error() << "parseStrategyFromFile: root must be an array in " << path
                         << logs::end;
        return false;
    }

    for (const auto& j : root) {
        out.push_back(parseInstruction(j));
    }
    // Tri stable par priorite descendante. Les instructions sans priorite (0.0)
    // conservent leur ordre d'ecriture dans le JSON. Les priorites < 0 finissent
    // en fin de liste (et sont skippees a l'execution par StrategyJsonRunner).
    std::stable_sort(out.begin(), out.end(),
                     [](const StrategyInstruction& a, const StrategyInstruction& b) {
                         return a.priority > b.priority;
                     });
    logger().info() << "parseStrategyFromFile: loaded " << out.size() << " instructions from "
                    << path << " (tri priority desc applique)" << logs::end;
    return true;
}

bool parseInitFromFile(const std::string& path, InitData& out)
{
    out = InitData{};

    std::ifstream f(path);
    if (!f.is_open()) {
        logger().error() << "parseInitFromFile: cannot open " << path << logs::end;
        return false;
    }

    json root;
    try {
        f >> root;
    } catch (const std::exception& e) {
        logger().error() << "parseInitFromFile: parse error in " << path << " : "
                         << e.what() << logs::end;
        return false;
    }

    if (!root.is_object()) {
        logger().error() << "parseInitFromFile: root must be an object in " << path << logs::end;
        return false;
    }

    out.x = root.value("x", out.x);
    out.y = root.value("y", out.y);
    // theta en RADIANS dans le JSON (format Esial historique), conversion en degres ici.
    float thetaRad = root.value("theta", 1.5707963f);
    out.thetaDeg = thetaRad * 180.0f / 3.14159265358979323846f;

    auto sp = root.find("setpos_tasks");
    if (sp != root.end() && sp->is_array()) {
        for (const auto& t : *sp) out.setposTasks.push_back(parseTask(t));
    }
    logger().info() << "parseInitFromFile: pose=(" << out.x << "," << out.y << ","
                    << out.thetaDeg << " deg), " << out.setposTasks.size()
                    << " setpos_tasks loaded from " << path << logs::end;
    return true;
}
