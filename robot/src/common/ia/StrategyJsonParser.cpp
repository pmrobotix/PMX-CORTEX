#include "StrategyJsonParser.hpp"

#include <algorithm>
#include <cstdio>
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

// Bornes table competition 2026 : 3000 x 2000 mm (convention PMX, origine
// bas-gauche). Toute coord JSON hors de ce rectangle est consideree invalide
// et provoque un fail du parser : safer-fail au chargement plutot que
// comportement imprevisible au match.
static constexpr float TABLE_X_MIN = 0.0f;
static constexpr float TABLE_X_MAX = 3000.0f;
static constexpr float TABLE_Y_MIN = 0.0f;
static constexpr float TABLE_Y_MAX = 2000.0f;

static bool checkXY(float x, float y, const char* what, int taskIdx, int instrId)
{
    if (x < TABLE_X_MIN || x > TABLE_X_MAX || y < TABLE_Y_MIN || y > TABLE_Y_MAX) {
        logger().error() << "Coord hors table : " << what << "=(" << x << "," << y << ")"
                         << " bornes=[" << TABLE_X_MIN << ".." << TABLE_X_MAX << ","
                         << TABLE_Y_MIN << ".." << TABLE_Y_MAX << "]"
                         << " (instr=" << instrId << " taskIdx=" << taskIdx << ")"
                         << logs::end;
        return false;
    }
    return true;
}

static bool validateTask(const StrategyTask& t, int taskIdx, int instrId)
{
    bool ok = true;
    if (t.position_x && t.position_y) {
        ok &= checkXY(*t.position_x, *t.position_y, "position", taskIdx, instrId);
    }
    if (t.face_x && t.face_y) {
        ok &= checkXY(*t.face_x, *t.face_y, "face", taskIdx, instrId);
    }
    for (size_t i = 0; i < t.waypoints.size(); i++) {
        const auto& w = t.waypoints[i];
        char label[32];
        std::snprintf(label, sizeof(label), "waypoint[%zu]", i);
        ok &= checkXY(w[0], w[1], label, taskIdx, instrId);
    }
    return ok;
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
    t.until_match_sec = optField<float>(j, "until_match_sec");
    if (t.duration_ms && t.until_match_sec) {
        logger().error() << "WAIT : duration_ms ET until_match_sec presents -> "
                         << "duration_ms ignore (until_match_sec prioritaire)" << logs::end;
        t.duration_ms = std::nullopt;
    }
    t.chain         = j.value("chain", false);

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
    instr.min_match_sec        = optField<float>(j, "min_match_sec");
    instr.max_match_sec        = optField<float>(j, "max_match_sec");
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
        // ignore_comments=true : autorise // ligne et /* bloc */ dans les JSON
        // strategie. Pratique pour commenter/decommenter des tasks (SET_SPEED
        // vs SET_ACC_DEC_PERCENT lors des tests A/B).
        root = json::parse(f, nullptr, /*allow_exceptions=*/true, /*ignore_comments=*/true);
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
    // Validation des coordonnees : refus au chargement de toute task ayant
    // une position (position, face, waypoints) hors table. Une typo dans le
    // JSON pre-match (ex: "23000" au lieu de "2300") doit etre detectee
    // avant que le robot tente d'y aller. Fail rapide ici, pas en course.
    bool allValid = true;
    for (const auto& instr : out) {
        for (size_t k = 0; k < instr.tasks.size(); k++) {
            if (!validateTask(instr.tasks[k], (int)k, instr.id)) {
                allValid = false;
            }
        }
    }
    if (!allValid) {
        logger().error() << "parseStrategyFromFile: validation coords echouee dans "
                         << path << " - charge ABORT" << logs::end;
        out.clear();
        return false;
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
        // ignore_comments=true : autorise // et /* */ dans les JSON init.
        root = json::parse(f, nullptr, /*allow_exceptions=*/true, /*ignore_comments=*/true);
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
    // theta en DEGRES dans le JSON. Default 90 deg = robot face Y+ (Nord).
    out.thetaDeg = root.value("theta", 90.0f);

    auto sp = root.find("setpos_tasks");
    if (sp != root.end() && sp->is_array()) {
        for (const auto& t : *sp) out.setposTasks.push_back(parseTask(t));
    }
    // Validation pose initiale + setpos_tasks (mêmes bornes table que la
    // strategie). Une init avec coord hors table = robot demarre n'importe ou.
    if (!checkXY(out.x, out.y, "init.pose", -1, 0)) {
        out = InitData{};
        return false;
    }
    bool allValid = true;
    for (size_t k = 0; k < out.setposTasks.size(); k++) {
        if (!validateTask(out.setposTasks[k], (int)k, 0)) {
            allValid = false;
        }
    }
    if (!allValid) {
        logger().error() << "parseInitFromFile: validation coords setpos_tasks echouee dans "
                         << path << " - charge ABORT" << logs::end;
        out = InitData{};
        return false;
    }
    logger().info() << "parseInitFromFile: pose=(" << out.x << "," << out.y << ","
                    << out.thetaDeg << " deg), " << out.setposTasks.size()
                    << " setpos_tasks loaded from " << path << logs::end;
    return true;
}
