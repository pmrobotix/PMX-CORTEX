/*!
 * \file
 * \brief Implementation du test de scenarios detection adversaire (SIMU).
 *
 * Les scenarios sont definis en JSON aligne avec STRATEGY_JSON_FORMAT.md.
 * Un mini-dispatcher local gere les tasks MOVEMENT dont le test a besoin.
 * Quand le StrategyJsonRunner officiel sera implemente (roadmap Phase 1),
 * il remplacera ce dispatcher sans changer le format JSON.
 */

#include "O_DetectionScenarioTest.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits.h>
#include <sstream>
#include <string>
#include <unistd.h>

#include "utils/json_3.11.2.hpp"

#include "action/Sensors.hpp"
#include "asserv/Asserv.hpp"
#include "interface/AAsservDriver.hpp"
#include "navigator/Navigator.hpp"
#include "navigator/RetryPolicy.hpp"
#include "utils/Chronometer.hpp"
#include "Robot.hpp"
#include "OPOS6UL_ActionsExtended.hpp"
#include "OPOS6UL_RobotExtended.hpp"

using json = nlohmann::json;

namespace {

// Parametres de visualisation SVG (centre-a-centre, cf IsOnPathTest)
constexpr float ROBOT_DIAMETER_MM  = 280.0f;
constexpr float ADV_DIAMETER_MM    = 400.0f;
constexpr float CORRIDOR_WIDTH_MM  = ROBOT_DIAMETER_MM + ADV_DIAMETER_MM;  // 680
constexpr float SLOW_DIST_MM       = 620.0f;
constexpr float STOP_DIST_MM       = 460.0f;

// =============================================================================
// Scenarios du Niveau A en JSON (format aligne STRATEGY_JSON_FORMAT)
//
// Chaque instruction : { name, start, adv, expected, tasks: [ { type, subtype, ...} ] }
// Champs test-specific : name, start, adv, expected.
// Les tasks sont executees sequentiellement par le mini-dispatcher du test.
// =============================================================================

const char* SCENARIOS_LEVEL_A = R"JSON(
[
  {
    "name": "A1_GoToClearPath",
    "desc": "Robot va tout droit sans adv",
    "start": {"x": 200, "y": 200, "theta_deg": 0},
    "adv": null,
    "expected": "FINISHED",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "GO_TO", "position_x": 800, "position_y": 200}
    ]
  },
  {
    "name": "A2_GoToAdvOnPath",
    "desc": "Adv plante a 400mm du robot sur le segment (pas de chevauchement initial)",
    "start": {"x": 200, "y": 200, "theta_deg": 0},
    "adv": {"x": 600, "y": 200},
    "expected": "NEAR_OBSTACLE",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "GO_TO", "position_x": 1000, "position_y": 200}
    ]
  },
  {
    "name": "A3_GoToAdvBeside",
    "desc": "Adv lateral (hors couloir isOnPath, mais dans le cone)",
    "start": {"x": 200, "y": 200, "theta_deg": 0},
    "adv": {"x": 500, "y": 600},
    "expected": "FINISHED",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "GO_TO", "position_x": 800, "position_y": 200}
    ]
  },
  {
    "name": "A4_GoToAdvBehindTargetClose",
    "desc": "Adv a 300mm au-dela de la cible : le cone ToF doit voir l'adv a l'arrivee (level 4) -> STOP",
    "start": {"x": 200, "y": 200, "theta_deg": 0},
    "adv": {"x": 1100, "y": 200},
    "expected": "NEAR_OBSTACLE",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "GO_TO", "position_x": 800, "position_y": 200}
    ]
  },
  {
    "name": "A4b_GoToAdvFarBehindTarget",
    "desc": "Adv a 1000mm au-dela de la cible : pas dans la zone de detection a l'arrivee",
    "start": {"x": 200, "y": 200, "theta_deg": 0},
    "adv": {"x": 1800, "y": 200},
    "expected": "FINISHED",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "GO_TO", "position_x": 800, "position_y": 200}
    ]
  },
  {
    "name": "A5_RotateAdvInFront",
    "desc": "Rotation pure 90 degres avec adv a 600mm devant (hors chevauchement, ROTATION bypass naturel)",
    "start": {"x": 200, "y": 200, "theta_deg": 0},
    "adv": {"x": 800, "y": 200},
    "expected": "FINISHED",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "ROTATE_DEG", "angle_deg": 90}
    ]
  },
  {
    "name": "A6_MoveForwardToAdvOnDiagPath",
    "desc": "moveForwardTo diagonale, adv a 141mm du segment (dans couloir 340mm)",
    "start": {"x": 200, "y": 200, "theta_deg": 0},
    "adv": {"x": 400, "y": 600},
    "expected": "NEAR_OBSTACLE",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "MOVE_FORWARD_TO", "position_x": 800, "position_y": 800}
    ]
  },
  {
    "name": "A7_GoBackToAdvBehind",
    "desc": "Marche arriere, adv a 400mm derriere (pas de chevauchement initial)",
    "start": {"x": 800, "y": 200, "theta_deg": 180},
    "adv": {"x": 400, "y": 200},
    "expected": "NEAR_OBSTACLE",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "GO_BACK_TO", "position_x": 100, "position_y": 200}
    ]
  },
  {
    "name": "A8_GoBackToAdvBeside",
    "desc": "Marche arriere avec adv lateral (hors couloir mais dans cone)",
    "start": {"x": 800, "y": 200, "theta_deg": 180},
    "adv": {"x": 500, "y": 600},
    "expected": "FINISHED",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "GO_BACK_TO", "position_x": 200, "position_y": 200}
    ]
  }
]
)JSON";

// =============================================================================
// Mini-dispatcher : execute une task MOVEMENT. Reproduit le sous-ensemble du
// futur StrategyJsonRunner dont le test a besoin. Migration triviale le jour ou
// on branche le runner officiel.
// =============================================================================

// Applique la politique d'ignore avant chaque task (mime le comportement match) :
//   - marche AVANT  : arriere ignore, front centre actif
//   - marche ARRIERE: front ignore, arriere centre actif
//   - rotation pure (FACE_TO, FACE_BACK_TO, ROTATE_*) : inchange (mode ROTATION bypass)
// L/R non utilises pour l'instant (ignores systematiquement true).
void applyIgnoreForTask(Sensors& sensors, const json& task)
{
    std::string subtype = task.value("subtype", "");

    bool forward = false;
    bool backward = false;

    if (subtype == "GO_TO" || subtype == "MOVE_FORWARD_TO") {
        forward = true;
    } else if (subtype == "GO_BACK_TO" || subtype == "MOVE_BACKWARD_TO") {
        backward = true;
    } else if (subtype == "LINE") {
        float d = task.value("distance_mm", 0.0f);
        forward  = (d >= 0);
        backward = (d <  0);
    }
    // FACE_TO / FACE_BACK_TO / ROTATE_DEG / ROTATE_ABS_DEG : rotation pure, pas de changement.

    if (forward) {
        sensors.setIgnoreFrontNearObstacle(true, false, true);  // front centre actif
        sensors.setIgnoreBackNearObstacle(true, true, true);    // arriere ignore
    } else if (backward) {
        sensors.setIgnoreFrontNearObstacle(true, true, true);   // front ignore
        sensors.setIgnoreBackNearObstacle(true, false, true);   // arriere centre actif
    }
}

TRAJ_STATE executeTask(Navigator& nav, Sensors& sensors, const json& task)
{
    std::string type = task.value("type", "");
    std::string subtype = task.value("subtype", "");

    RetryPolicy policy = RetryPolicy::noRetry();

    applyIgnoreForTask(sensors, task);

    if (type == "MOVEMENT") {
        if (subtype == "GO_TO") {
            return nav.goTo(task["position_x"], task["position_y"], policy);
        }
        if (subtype == "GO_BACK_TO") {
            return nav.goBackTo(task["position_x"], task["position_y"], policy);
        }
        if (subtype == "MOVE_FORWARD_TO") {
            return nav.moveForwardTo(task["position_x"], task["position_y"], policy);
        }
        if (subtype == "LINE") {
            return nav.line(task["distance_mm"], policy);
        }
        if (subtype == "ROTATE_DEG") {
            return nav.rotateDeg(task["angle_deg"], policy);
        }
        if (subtype == "ROTATE_ABS_DEG") {
            return nav.rotateAbsDeg(task["angle_deg"], policy);
        }
        if (subtype == "FACE_TO") {
            return nav.faceTo(task["position_x"], task["position_y"], policy);
        }
        if (subtype == "FACE_BACK_TO") {
            return nav.faceBackTo(task["position_x"], task["position_y"], policy);
        }
    }

    return TRAJ_ERROR;
}

// Recupere la position cible (x, y) de la derniere task du scenario,
// pour le SVG (depart -> cible).
void extractTarget(const json& tasks, float& tx, float& ty)
{
    tx = 0; ty = 0;
    if (tasks.empty()) return;
    const auto& last = tasks.back();
    std::string subtype = last.value("subtype", "");
    if (subtype == "ROTATE_DEG" || subtype == "ROTATE_ABS_DEG" || subtype == "LINE") {
        return;  // pas de cible xy
    }
    tx = last.value("position_x", 0.0f);
    ty = last.value("position_y", 0.0f);
}

} // namespace

// =============================================================================
// run — point d'entree
// =============================================================================

void O_DetectionScenarioTest::run(int /*argc*/, char** /*argv*/)
{
    logger().info() << "========================================" << logs::end;
    logger().info() << "O_DetectionScenarioTest — Niveau A" << logs::end;
    logger().info() << "========================================" << logs::end;

    passed_ = 0;
    warnCrossed_ = 0;
    failed_ = 0;
    scenesA_.clear();

    setupSensorsForTest();

    runLevelA();

    teardownSensorsForTest();

    logger().info() << "========================================" << logs::end;
    logger().info() << "Resultat Niveau A : " << passed_ << " PASS / "
                    << warnCrossed_ << " WARN_CROSSED / "
                    << failed_ << " FAIL"
                    << " sur " << scenesA_.size() << " scenarios" << logs::end;
    logger().info() << "========================================" << logs::end;

    for (const auto& sc : scenesA_) {
        logScene(sc);
    }

    writeSvgFile();
}

// =============================================================================
// Config capteurs pour le test : activer detection centre avant + arriere
// =============================================================================

void O_DetectionScenarioTest::setupSensorsForTest()
{
    OPOS6UL_RobotExtended& robot = OPOS6UL_RobotExtended::instance();
    auto& sensors = robot.actions().sensors();

    // Activer les capteurs centre en hardware (enable). Les ignore flags sont
    // gerees par task dans applyIgnoreForTask (avant/arriere selon le move).
    sensors.addConfigFront(false, true, false);
    sensors.addConfigBack(false, true, false);

    // Par defaut tout ignore jusqu'a ce que la premiere task fixe la politique.
    sensors.setIgnoreFrontNearObstacle(true, true, true);
    sensors.setIgnoreBackNearObstacle(true, true, true);

    // Demarrer le thread capteurs qui publiera DetectionEvent toutes les 20ms
    sensors.startSensorsThread(20);

    // Laisser 200ms pour que le thread se stabilise
    utils::sleep_for_micros(200000);
}

void O_DetectionScenarioTest::teardownSensorsForTest()
{
    OPOS6UL_RobotExtended& robot = OPOS6UL_RobotExtended::instance();
    robot.actions().sensors().clearInjectedAdv();
    // Note : SensorsThread reste actif (arrete par le shutdown du robot)
}

// =============================================================================
// runLevelA
// =============================================================================

void O_DetectionScenarioTest::runLevelA()
{
    executeScenariosFromJson(SCENARIOS_LEVEL_A, scenesA_);

    for (const auto& sc : scenesA_) {
        switch (sc.verdict) {
            case VERDICT_PASS:             passed_++; break;
            case VERDICT_WARN_CROSSED_ADV: warnCrossed_++; break;
            case VERDICT_FAIL_STATE:       failed_++; break;
        }
    }
}

// =============================================================================
// executeScenariosFromJson — parse JSON, execute chaque scenario
// =============================================================================

void O_DetectionScenarioTest::executeScenariosFromJson(const char* jsonText,
                                                        std::vector<SceneResult>& out)
{
    OPOS6UL_RobotExtended& robot = OPOS6UL_RobotExtended::instance();
    auto& sensors = robot.actions().sensors();

    json root;
    try {
        root = json::parse(jsonText);
    } catch (const json::parse_error& e) {
        logger().error() << "JSON parse error : " << e.what() << logs::end;
        return;
    }

    if (!root.is_array()) {
        logger().error() << "JSON doit etre un array de scenarios" << logs::end;
        return;
    }

    for (const auto& sc : root) {
        SceneResult res;
        res.name        = sc.value("name", "(noname)");
        res.startX      = sc["start"]["x"];
        res.startY      = sc["start"]["y"];
        res.startThetaDeg = sc["start"].value("theta_deg", 0.0f);
        res.hasAdv      = !sc["adv"].is_null();
        if (res.hasAdv) {
            res.advX = sc["adv"]["x"];
            res.advY = sc["adv"]["y"];
        } else {
            res.advX = NAN; res.advY = NAN;
        }
        res.expected    = parseTrajExpected(sc.value("expected", "FINISHED"));

        const auto& tasks = sc["tasks"];
        res.command = tasks.empty() ? "?" : tasks.back().value("subtype", "?");
        extractTarget(tasks, res.targetX, res.targetY);

        logger().info() << "=== " << res.name << " ===" << logs::end;

        // 0. Sanity check : pas de chevauchement physique au depart.
        //    Deux robots ne peuvent pas occuper le meme espace en match reel.
        if (res.hasAdv) {
            float dxs = res.advX - res.startX;
            float dys = res.advY - res.startY;
            float dstart = std::sqrt(dxs * dxs + dys * dys);
            if (dstart < 340.0f) {
                logger().error() << "  SCENARIO INVALIDE : chevauchement initial (dist="
                                 << dstart << "mm < 340mm). Scenario ignore." << logs::end;
                res.actual = TRAJ_ERROR;
                res.verdict = VERDICT_FAIL_STATE;
                res.endX = res.startX; res.endY = res.startY;
                out.push_back(res);
                continue;
            }
        }

        // 1. Reset robot (position + orientation)
        robot.asserv().setPositionAndColor(res.startX, res.startY, res.startThetaDeg, false);
        utils::sleep_for_micros(200000);

        // 2. Inject adv (ou clear)
        sensors.clearInjectedAdv();
        if (res.hasAdv) {
            sensors.setInjectedAdv(res.advX, res.advY);
            logger().info() << "  adv injecte (" << res.advX << "," << res.advY << ")" << logs::end;
        }

        // 3. Stabilisation detection
        utils::sleep_for_micros(200000);

        // 4. Execution des tasks
        Navigator nav(&robot);
        utils::Chronometer chrono("scenario");
        chrono.start();

        TRAJ_STATE ts = TRAJ_FINISHED;
        for (const auto& task : tasks) {
            ts = executeTask(nav, sensors, task);
            if (ts != TRAJ_FINISHED) break;   // abort sur premier echec
        }

        chrono.stop();
        res.actual = ts;
        res.durationMs = chrono.getElapsedTimeInMilliSec();

        // 5. Position finale
        ROBOTPOSITION p = robot.asserv().pos_getPosition();
        res.endX = p.x;
        res.endY = p.y;
        res.endThetaDeg = p.theta * 180.0f / (float)M_PI;

        // 6. Check geometrique : la trajectoire (start->end) a-t-elle physiquement
        //    traverse l'adv ? (SIMU n'a pas de physique de collision donc le robot
        //    peut passer au travers si la detection ne trigger pas).
        //    On ne verifie que pour les mouvements translationnels.
        res.crossesAdv = false;
        if (res.hasAdv && res.command != "ROTATE_DEG" && res.command != "ROTATE_ABS_DEG"
            && res.command != "FACE_TO" && res.command != "FACE_BACK_TO") {
            float dmin = distPointToSegment(res.advX, res.advY,
                                             res.startX, res.startY,
                                             res.endX, res.endY);
            const float MIN_SAFE_DIST = 340.0f;  // robot_radius + adv_radius
            res.crossesAdv = (dmin < MIN_SAFE_DIST);
        }

        res.verdict = computeVerdict(res);

        // 7. Cleanup
        // Si le scenario a echoue (FAIL_STATE / WARN_CROSSED), on force un reset
        // complet du driver pour que le scenario suivant redemarre proprement :
        // - setEmergencyStop flush la queue + interrompt la commande en cours
        // - resetEmergencyOnTraj retire le flag
        // En cas de PASS, on laisse l'asserv dans son etat naturel (comme en match).
        if (res.verdict != VERDICT_PASS) {
            robot.asserv().setEmergencyStop();
            utils::sleep_for_micros(50000);
            robot.asserv().resetEmergencyOnTraj("scenario FAIL: reset complet");
        } else {
            // PASS : reset emergency flag seulement si deja set
            robot.asserv().resetEmergencyOnTraj("end scenario OK");
        }
        sensors.clearInjectedAdv();
        utils::sleep_for_micros(100000);

        logger().info() << "  " << verdictName(res.verdict)
                        << " expected=" << trajName(res.expected)
                        << " actual=" << trajName(res.actual)
                        << " end=(" << res.endX << "," << res.endY << ")"
                        << " crossed=" << (res.crossesAdv ? "yes" : "no")
                        << " dur=" << res.durationMs << "ms" << logs::end;

        out.push_back(res);
    }
}

// =============================================================================
// Helpers geometrie + verdict
// =============================================================================

float O_DetectionScenarioTest::distPointToSegment(float px, float py,
                                                   float ax, float ay,
                                                   float bx, float by)
{
    // Projection orthogonale clampee sur [0,1]
    float dx = bx - ax;
    float dy = by - ay;
    float len_sq = dx * dx + dy * dy;
    if (len_sq < 1.0f) {
        // Segment nul : distance point-point
        float px0 = px - ax;
        float py0 = py - ay;
        return std::sqrt(px0 * px0 + py0 * py0);
    }
    float t = ((px - ax) * dx + (py - ay) * dy) / len_sq;
    t = std::max(0.0f, std::min(1.0f, t));
    float cx = ax + t * dx;
    float cy = ay + t * dy;
    float dxp = px - cx;
    float dyp = py - cy;
    return std::sqrt(dxp * dxp + dyp * dyp);
}

O_DetectionScenarioTest::Verdict O_DetectionScenarioTest::computeVerdict(const SceneResult& sc)
{
    // 1) L'etat de trajectoire ne correspond pas a l'attendu -> bug franc.
    if (sc.actual != sc.expected) {
        return VERDICT_FAIL_STATE;
    }
    // 2) Le robot a physiquement traverse l'adv (cercles qui se chevauchent).
    //    C'est un FAIL peu importe le TRAJ_STATE : meme si on attendait un STOP
    //    et qu'on a eu un STOP, si le robot s'est arrete DANS l'adv, la detection
    //    a reagi trop tard (pas assez de marge). En match reel ca serait
    //    une collision physique.
    if (sc.hasAdv && sc.crossesAdv) {
        return VERDICT_WARN_CROSSED_ADV;
    }
    return VERDICT_PASS;
}

// =============================================================================
// Helpers
// =============================================================================

TRAJ_STATE O_DetectionScenarioTest::parseTrajExpected(const std::string& s)
{
    if (s == "FINISHED")       return TRAJ_FINISHED;
    if (s == "NEAR_OBSTACLE")  return TRAJ_NEAR_OBSTACLE;
    if (s == "COLLISION")      return TRAJ_COLLISION;
    if (s == "INTERRUPTED")    return TRAJ_INTERRUPTED;
    if (s == "IMPOSSIBLE")     return TRAJ_IMPOSSIBLE;
    if (s == "ERROR")          return TRAJ_ERROR;
    return TRAJ_FINISHED;
}

const char* O_DetectionScenarioTest::verdictName(Verdict v)
{
    switch (v) {
        case VERDICT_PASS:              return "PASS";
        case VERDICT_FAIL_STATE:        return "FAIL_STATE";
        case VERDICT_WARN_CROSSED_ADV:  return "WARN_CROSSED_ADV";
    }
    return "?";
}

const char* O_DetectionScenarioTest::trajName(TRAJ_STATE ts)
{
    switch (ts) {
        case TRAJ_FINISHED:      return "FINISHED";
        case TRAJ_NEAR_OBSTACLE: return "NEAR_OBSTACLE";
        case TRAJ_COLLISION:     return "COLLISION";
        case TRAJ_INTERRUPTED:   return "INTERRUPTED";
        case TRAJ_IMPOSSIBLE:    return "IMPOSSIBLE";
        case TRAJ_IDLE:          return "IDLE";
        case TRAJ_ERROR:         return "ERROR";
        default:                 return "?";
    }
}

void O_DetectionScenarioTest::logScene(const SceneResult& sc)
{
    logger().info() << sc.name
                    << "  cmd=" << sc.command
                    << "  start=(" << sc.startX << "," << sc.startY << ")"
                    << "  target=(" << sc.targetX << "," << sc.targetY << ")"
                    << (sc.hasAdv ? "  adv=(" : "  adv=none")
                    << (sc.hasAdv ? std::to_string(sc.advX) + "," + std::to_string(sc.advY) + ")" : "")
                    << "  expected=" << trajName(sc.expected)
                    << "  actual=" << trajName(sc.actual)
                    << "  end=(" << sc.endX << "," << sc.endY << ")"
                    << "  crossed=" << (sc.crossesAdv ? "yes" : "no")
                    << "  dur=" << sc.durationMs << "ms"
                    << "  [" << verdictName(sc.verdict) << "]" << logs::end;
}

// =============================================================================
// SVG de synthese
// =============================================================================

std::string O_DetectionScenarioTest::exeDirectory()
{
    char buf[PATH_MAX];
    ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0) return ".";
    buf[n] = '\0';
    std::string path(buf);
    size_t slash = path.find_last_of('/');
    return (slash == std::string::npos) ? std::string(".") : path.substr(0, slash);
}

std::string O_DetectionScenarioTest::svgScene(const SceneResult& sc, float scale, float cellW, float cellH)
{
    std::ostringstream svg;

    svg << "<rect x='0' y='0' width='" << cellW << "' height='" << cellH
        << "' fill='#f8f8f8' stroke='#ccc' stroke-width='1'/>\n";

    float minX = std::min({sc.startX, sc.targetX, sc.hasAdv ? sc.advX : sc.startX}) - 400;
    float maxX = std::max({sc.startX, sc.targetX, sc.hasAdv ? sc.advX : sc.startX}) + 400;
    float minY = std::min({sc.startY, sc.targetY, sc.hasAdv ? sc.advY : sc.startY}) - 400;
    float maxY = std::max({sc.startY, sc.targetY, sc.hasAdv ? sc.advY : sc.startY}) + 400;

    float spanX = (maxX - minX) * scale;
    float spanY = (maxY - minY) * scale;
    float margin = 40.0f;
    float offX = margin + (cellW - 2*margin - spanX) * 0.5f - minX * scale;
    float offY = margin + (cellH - 2*margin - spanY) * 0.5f - minY * scale;

    svg << "<g transform='translate(" << offX << "," << offY << ") scale(" << scale << ")'>\n";

    // --- Couloir isOnPath (statique, le long du segment start -> cible) ---
    // Rectangle pointille bleu-gris de largeur corridor_width autour du segment.
    // Represente la zone ou un adv bloquerait isOnPath (T8).
    float dx = sc.targetX - sc.startX;
    float dy = sc.targetY - sc.startY;
    float len = std::sqrt(dx*dx + dy*dy);
    if (len > 1.0f && sc.command != "ROTATE_DEG" && sc.command != "ROTATE_ABS_DEG") {
        float angleDeg = std::atan2(dy, dx) * 180.0f / (float)M_PI;
        float halfW = CORRIDOR_WIDTH_MM * 0.5f;

        svg << "<g transform='translate(" << sc.startX << "," << sc.startY
            << ") rotate(" << angleDeg << ")'>\n";
        svg << "<rect x='0' y='" << -halfW
            << "' width='" << len << "' height='" << CORRIDOR_WIDTH_MM
            << "' fill='#4A90E2' fill-opacity='0.05' stroke='#8888aa' stroke-width='2' stroke-dasharray='15,8'/>\n";
        svg << "</g>\n";
    }

    // --- Zones de reaction dynamiques, ancrees a la position FINALE du robot ---
    // Zone rouge : adv dedans -> STOP (emergency stop, NEAR_OBSTACLE)
    // Zone jaune : adv dedans -> SLOW (vitesse reduite)
    // Orientation : direction du mouvement (start -> end), pour le rendu visuel.
    // Pour ROTATE_*, les zones n'ont pas de sens directionnel -> on n'affiche rien.
    if (sc.command != "ROTATE_DEG" && sc.command != "ROTATE_ABS_DEG") {
        float ex = sc.endX - sc.startX;
        float ey = sc.endY - sc.startY;
        float elen = std::sqrt(ex*ex + ey*ey);
        float angleEndDeg;
        if (elen > 1.0f) {
            angleEndDeg = std::atan2(ey, ex) * 180.0f / (float)M_PI;
        } else {
            // Robot pas bouge : utiliser theta depart
            angleEndDeg = sc.startThetaDeg;
        }
        // Pour les mouvements en marche arriere, la zone "avant" est derriere (180°).
        bool isBackward = (sc.command == "GO_BACK_TO" || sc.command == "LINE_BACK"
                           || sc.command == "FACE_BACK_TO");
        if (isBackward) angleEndDeg += 180.0f;

        float halfW = CORRIDOR_WIDTH_MM * 0.5f;

        svg << "<g transform='translate(" << sc.endX << "," << sc.endY
            << ") rotate(" << angleEndDeg << ")'>\n";
        // Zone SLOW (jaune) entre stop et slow
        svg << "<rect x='" << STOP_DIST_MM << "' y='" << -halfW
            << "' width='" << (SLOW_DIST_MM - STOP_DIST_MM) << "' height='" << CORRIDOR_WIDTH_MM
            << "' fill='#FFD700' fill-opacity='0.25' stroke='#FFA500' stroke-width='1'/>\n";
        // Zone STOP (rouge) de 0 a stop_distance
        svg << "<rect x='0' y='" << -halfW
            << "' width='" << STOP_DIST_MM << "' height='" << CORRIDOR_WIDTH_MM
            << "' fill='#DC143C' fill-opacity='0.22' stroke='#8B0000' stroke-width='1'/>\n";
        svg << "</g>\n";
    }

    // Segment depart -> cible (pointille bleu)
    if (sc.command != "ROTATE_DEG" && sc.command != "ROTATE_ABS_DEG") {
        svg << "<line x1='" << sc.startX << "' y1='" << sc.startY
            << "' x2='" << sc.targetX << "' y2='" << sc.targetY
            << "' stroke='#0066cc' stroke-width='3' stroke-dasharray='10,5'/>\n";
    }

    // Trajectoire reelle (start -> end). Couleur selon verdict :
    //   PASS            -> vert (comportement correct)
    //   WARN_CROSSED    -> rouge (a traverse l'adv : detection cassee)
    //   FAIL_STATE      -> orange (expected != actual)
    const char* trajStroke = "#1a7c1a";
    const char* endFill    = "#1a7c1a";
    if (sc.verdict == VERDICT_WARN_CROSSED_ADV) { trajStroke = "#DC143C"; endFill = "#DC143C"; }
    else if (sc.verdict == VERDICT_FAIL_STATE)   { trajStroke = "#FF8C00"; endFill = "#FF8C00"; }

    svg << "<line x1='" << sc.startX << "' y1='" << sc.startY
        << "' x2='" << sc.endX << "' y2='" << sc.endY
        << "' stroke='" << trajStroke << "' stroke-width='5'/>\n";

    // Indicateur explicite de collision physique (traversee de l'adv) :
    // cercle rouge vif au point de plus proche approche sur la trajectoire.
    if (sc.hasAdv && sc.crossesAdv) {
        float dxt = sc.endX - sc.startX;
        float dyt = sc.endY - sc.startY;
        float len_sq = dxt * dxt + dyt * dyt;
        if (len_sq > 1.0f) {
            float t = ((sc.advX - sc.startX) * dxt + (sc.advY - sc.startY) * dyt) / len_sq;
            t = std::max(0.0f, std::min(1.0f, t));
            float cpX = sc.startX + t * dxt;
            float cpY = sc.startY + t * dyt;
            // Croix rouge a la position de plus proche approche
            float s = 45.0f;
            svg << "<line x1='" << (cpX - s) << "' y1='" << (cpY - s)
                << "' x2='" << (cpX + s) << "' y2='" << (cpY + s)
                << "' stroke='#DC143C' stroke-width='8'/>\n";
            svg << "<line x1='" << (cpX - s) << "' y1='" << (cpY + s)
                << "' x2='" << (cpX + s) << "' y2='" << (cpY - s)
                << "' stroke='#DC143C' stroke-width='8'/>\n";
            // Segment rouge du point de collision vers l'adv
            svg << "<line x1='" << cpX << "' y1='" << cpY
                << "' x2='" << sc.advX << "' y2='" << sc.advY
                << "' stroke='#DC143C' stroke-width='3' stroke-dasharray='6,4'/>\n";
        }
    }

    // Position finale robot (cercle semi-transparent, couleur selon verdict)
    svg << "<circle cx='" << sc.endX << "' cy='" << sc.endY
        << "' r='" << (ROBOT_DIAMETER_MM*0.5f) << "' fill='" << endFill << "' fill-opacity='0.20'"
        << " stroke='" << endFill << "' stroke-width='3'/>\n";

    // Position depart robot (cercle bleu)
    svg << "<circle cx='" << sc.startX << "' cy='" << sc.startY
        << "' r='" << (ROBOT_DIAMETER_MM*0.5f) << "' fill='#4A90E2' fill-opacity='0.25'"
        << " stroke='#0066cc' stroke-width='3'/>\n";
    svg << "<circle cx='" << sc.startX << "' cy='" << sc.startY
        << "' r='15' fill='#0066cc'/>\n";
    float arrowLen = 120.0f;
    float rad = sc.startThetaDeg * (float)M_PI / 180.0f;
    svg << "<line x1='" << sc.startX << "' y1='" << sc.startY
        << "' x2='" << (sc.startX + arrowLen*std::cos(rad)) << "' y2='" << (sc.startY + arrowLen*std::sin(rad))
        << "' stroke='#0066cc' stroke-width='4'/>\n";

    // Cible (croix bleue)
    if (sc.command != "ROTATE_DEG" && sc.command != "ROTATE_ABS_DEG") {
        float s = 30.0f;
        svg << "<line x1='" << (sc.targetX - s) << "' y1='" << (sc.targetY - s)
            << "' x2='" << (sc.targetX + s) << "' y2='" << (sc.targetY + s)
            << "' stroke='#0066cc' stroke-width='4'/>\n";
        svg << "<line x1='" << (sc.targetX - s) << "' y1='" << (sc.targetY + s)
            << "' x2='" << (sc.targetX + s) << "' y2='" << (sc.targetY - s)
            << "' stroke='#0066cc' stroke-width='4'/>\n";
    }

    // Adversaire
    if (sc.hasAdv) {
        svg << "<circle cx='" << sc.advX << "' cy='" << sc.advY
            << "' r='" << (ADV_DIAMETER_MM*0.5f) << "' fill='#DC143C' fill-opacity='0.25'"
            << " stroke='#8B0000' stroke-width='3'/>\n";
        svg << "<circle cx='" << sc.advX << "' cy='" << sc.advY
            << "' r='15' fill='#8B0000'/>\n";
    }

    svg << "</g>\n";

    // Labels
    const char* vcolor = "#228B22";  // vert par defaut (PASS)
    if (sc.verdict == VERDICT_WARN_CROSSED_ADV) vcolor = "#DC143C";
    else if (sc.verdict == VERDICT_FAIL_STATE)  vcolor = "#FF8C00";

    svg << "<text x='" << margin << "' y='22' font-family='monospace' font-size='13' font-weight='bold'>"
        << sc.name << "</text>\n";
    svg << "<text x='" << margin << "' y='" << (cellH - 50)
        << "' font-family='monospace' font-size='11'>"
        << "cmd=" << sc.command
        << "  dur=" << (int)sc.durationMs << "ms"
        << (sc.hasAdv && sc.crossesAdv ? "  <tspan fill='#DC143C' font-weight='bold'>[TRAVERSE ADV]</tspan>" : "")
        << "</text>\n";
    svg << "<text x='" << margin << "' y='" << (cellH - 32)
        << "' font-family='monospace' font-size='11'>"
        << "exp=" << trajName(sc.expected)
        << "  got=" << trajName(sc.actual)
        << "</text>\n";
    svg << "<text x='" << margin << "' y='" << (cellH - 14)
        << "' font-family='monospace' font-size='12' font-weight='bold' fill='" << vcolor << "'>"
        << "[" << verdictName(sc.verdict) << "]"
        << "</text>\n";

    return svg.str();
}

void O_DetectionScenarioTest::writeSvgFile()
{
    const int    cols   = 2;
    const int    rows   = (scenesA_.size() + cols - 1) / cols;
    const float  cellW  = 650.0f;
    const float  cellH  = 450.0f;
    const float  scale  = 0.28f;
    const float  totalW = cols * cellW;
    const float  totalH = rows * cellH + 80;

    std::ostringstream svg;
    svg << "<?xml version='1.0' encoding='UTF-8'?>\n";
    svg << "<svg xmlns='http://www.w3.org/2000/svg' width='" << totalW
        << "' height='" << totalH << "' viewBox='0 0 " << totalW << " " << totalH << "'>\n";

    svg << "<text x='10' y='28' font-family='monospace' font-size='20' font-weight='bold'>"
        << "Detection Scenarios — Niveau A (tactique)</text>\n";
    svg << "<text x='10' y='48' font-family='monospace' font-size='11'>"
        << "bleu=depart  vert=fin(OK)  orange=fail-state  rouge=traverse-adv"
        << "  couloir pointille=isOnPath(segment)"
        << "  zones rouge/jaune=STOP/SLOW a la position finale (dynamiques)"
        << "  (robot " << ROBOT_DIAMETER_MM << "mm  adv " << ADV_DIAMETER_MM << "mm"
        << "  stop=" << STOP_DIST_MM << "mm  slow=" << SLOW_DIST_MM << "mm)</text>\n";
    bool allGreen = (failed_ == 0 && warnCrossed_ == 0);
    svg << "<text x='10' y='68' font-family='monospace' font-size='13' font-weight='bold' fill='"
        << (allGreen ? "#228B22" : "#DC143C") << "'>"
        << passed_ << " PASS / "
        << warnCrossed_ << " WARN_CROSSED_ADV / "
        << failed_ << " FAIL_STATE"
        << "  sur " << scenesA_.size() << " scenarios"
        << "</text>\n";

    for (size_t i = 0; i < scenesA_.size(); i++) {
        int r = (int)(i / cols);
        int c = (int)(i % cols);
        float x = c * cellW;
        float y = 80 + r * cellH;
        svg << "<g transform='translate(" << x << "," << y << ")'>\n";
        svg << svgScene(scenesA_[i], scale, cellW, cellH);
        svg << "</g>\n";
    }

    svg << "</svg>\n";

    std::string path = exeDirectory() + "/test_detection_scenarios.svg";
    std::ofstream ofs(path);
    if (ofs) {
        ofs << svg.str();
        logger().info() << "SVG synthese ecrit : " << path << logs::end;
    } else {
        logger().error() << "impossible d'ecrire le SVG : " << path << logs::end;
    }
}
