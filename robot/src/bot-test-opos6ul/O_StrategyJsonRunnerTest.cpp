/*!
 * \file
 * \brief Test d'integration StrategyJsonRunner (SIMU).
 * Voir O_StrategyJsonRunnerTest.hpp pour la description generale.
 */

#include "O_StrategyJsonRunnerTest.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits.h>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>

#include "action/Sensors.hpp"
#include "asserv/Asserv.hpp"
#include "ia/ActionRegistry.hpp"
#include "ia/FlagManager.hpp"
#include "ia/IAbyPath.hpp"
#include "interface/AAsservDriver.hpp"
#include "navigator/Navigator.hpp"
#include "utils/Chronometer.hpp"
#include "Robot.hpp"
#include "OPOS6UL_ActionsExtended.hpp"
#include "OPOS6UL_IAExtended.hpp"
#include "OPOS6UL_RobotExtended.hpp"
#include "ia/StrategyJsonRunner.hpp"

namespace {

constexpr float ROBOT_DIAMETER_MM = 280.0f;
constexpr float ADV_DIAMETER_MM   = 400.0f;

// Specification d'un scenario : fichier JSON + conditions initiales + attendus.
// Les attendus sont hardcodes ici (le JSON reste conforme au format standard
// pour etre rejouable dans le simulateur web).
struct ScenarioSpec
{
    const char* name;
    const char* desc;
    const char* jsonFile;
    float       startX, startY, startThetaDeg;
    bool        hasAdv;
    float       advX, advY;
    std::vector<std::string> expectedTrace;
    std::set<std::string>    expectedFlags;
    // Outcomes attendus dans l'ordre chronologique d'evaluation par run().
    // Distingue v2 (skip+continue, vector complet) de v1 (abort, vector tronque).
    std::vector<InstructionOutcome::Status> expectedOutcomeStatuses;
};

const std::vector<ScenarioSpec> SCENARIOS = {
    {
        "SR01", "PrioritySort : ordre d'execution = priority desc (100>50>10)",
        "strategySR01.json",
        300, 300, 0,
        false, 0, 0,
        {"trace_i1", "trace_i3", "trace_i2"},
        {},
        {
            InstructionOutcome::Status::FINISHED,
            InstructionOutcome::Status::FINISHED,
            InstructionOutcome::Status::FINISHED,
        }
    },
    {
        "SR02", "FlagChain : action_flag A leve par i1, i2 needed=A OK, i3 needed=NEVER skip",
        "strategySR02.json",
        300, 300, 0,
        false, 0, 0,
        {"trace_i1", "trace_i2"},
        {"A"},
        {
            InstructionOutcome::Status::FINISHED,
            InstructionOutcome::Status::FINISHED,
            InstructionOutcome::Status::SKIPPED_NEEDED_FLAG,
        }
    },
    {
        "SR03", "PriorityNegDisable : i1 priority=-1 desactivee, seul i2 s'execute",
        "strategySR03.json",
        300, 300, 0,
        false, 0, 0,
        {"trace_i2"},
        {},
        {
            InstructionOutcome::Status::FINISHED,
            InstructionOutcome::Status::SKIPPED_PRIORITY,
        }
    },
    {
        "SR04", "TaskLevelFlag : task 2 skip par needed_flag (NEVER), tasks 1 et 3 OK",
        "strategySR04.json",
        300, 300, 0,
        false, 0, 0,
        {"trace_t1", "trace_t3"},
        {},
        {
            InstructionOutcome::Status::FINISHED,
        }
    },
    {
        "SR05", "ClearFlagsChain : i2 clear A, donc i3 needed=A skippee",
        "strategySR05.json",
        300, 300, 0,
        false, 0, 0,
        {"trace_i1", "trace_i2"},
        {},
        {
            InstructionOutcome::Status::FINISHED,
            InstructionOutcome::Status::FINISHED,
            InstructionOutcome::Status::SKIPPED_NEEDED_FLAG,
        }
    },
    {
        "SR06", "AdvBlocksI2_I3 : adv bloque i2 et i3 ; v2 -> outcomes=[OK,SKIP,SKIP], pas d'abort",
        "strategySR06.json",
        300, 300, 0,
        // Adv (2300,1100) sur la diagonale i2 (1500,400)->(2500,1400) : bloque i2.
        // L'adv reste injecte tout le scenario donc i3 est aussi bloque (cone
        // front detecte l'adv au demarrage du faceTo). C'est OK : ce qui
        // distingue v2 de v1, c'est que outcomes() renvoie 3 entrees (1 OK +
        // 2 SKIPPED_OBSTACLE) au lieu de 1 (et abort) en v1.
        true, 2300, 1100,
        {"trace_i1"},
        {"a_done"},
        {
            InstructionOutcome::Status::FINISHED,
            InstructionOutcome::Status::SKIPPED_OBSTACLE,
            InstructionOutcome::Status::SKIPPED_OBSTACLE,
        }
    },
    {
        "SR07", "AdvBlocksBackward : adv pile derriere bloque LINE -300, i2 fait GO_TO ailleurs",
        "strategySR07.json",
        // start (1500, 1500, theta=0) : robot oriente +X (Est). Devant=+X, derriere=-X.
        1500, 1500, 0,
        // Adv (1300, 1500) = 200mm pile derriere robot -> backLevel=-4 -> stop -> retry -> SKIP.
        // i2 GO_TO (1500, 800) : la trajectoire ne passe PAS par l'adv (sud), donc OK.
        true, 1300, 1500,
        {"trace_i2"},   // i1.LINE bloquee donc trace_i1 pas appele ; i2 OK donc trace_i2 ok.
        {},
        {
            InstructionOutcome::Status::SKIPPED_OBSTACLE,  // i1 LINE -300 bloquee (back)
            InstructionOutcome::Status::FINISHED,          // i2 GO_TO libre
        }
    },
    {
        "SR08", "MaxMatchSecGate : i2 max_match_sec=1 -> skip apres i1, i3 max_match_sec=999 -> OK",
        "strategySR08.json",
        300, 300, 0,
        false, 0, 0,
        {"trace_i1", "trace_i3"},
        {},
        {
            InstructionOutcome::Status::FINISHED,         // i1 OK (chrono ~0->2s)
            InstructionOutcome::Status::SKIPPED_MAX_TIME, // i2 max=1, t>1 -> skip
            InstructionOutcome::Status::FINISHED,         // i3 max=999 -> OK
        }
    },
    {
        "SR09", "PathImpossible : i2 PATH_TO (1800,1000) zone dyn active -> A* chemin vide -> SK_IMPS",
        "strategySR09.json",
        300, 300, 0,
        false, 0, 0,
        {"trace_i1", "trace_i3"},
        {},
        {
            InstructionOutcome::Status::FINISHED,           // i1 OK
            InstructionOutcome::Status::SKIPPED_IMPOSSIBLE, // i2 PATH_TO bordure
            InstructionOutcome::Status::FINISHED,           // i3 OK
        }
    }
};

// Helpers SVG : echapper du texte
std::string svgEscape(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (char c : s) {
        switch (c) {
            case '<': out += "&lt;"; break;
            case '>': out += "&gt;"; break;
            case '&': out += "&amp;"; break;
            case '"': out += "&quot;"; break;
            default:  out += c;
        }
    }
    return out;
}

std::string joinTrace(const std::vector<std::string>& v) {
    std::string out;
    for (size_t i = 0; i < v.size(); i++) {
        if (i > 0) out += ",";
        out += v[i];
    }
    return out;
}

std::string joinFlags(const std::set<std::string>& s) {
    std::string out;
    bool first = true;
    for (const auto& f : s) {
        if (!first) out += ",";
        out += f;
        first = false;
    }
    return out;
}

const char* statusName(InstructionOutcome::Status s) {
    using St = InstructionOutcome::Status;
    switch (s) {
        case St::FINISHED:            return "OK";
        case St::SKIPPED_PRIORITY:    return "SK_PRIO";
        case St::SKIPPED_NEEDED_FLAG: return "SK_FLAG";
        case St::SKIPPED_MAX_TIME:    return "SK_TIME";
        case St::SKIPPED_OBSTACLE:    return "SK_OBST";
        case St::SKIPPED_COLLISION:   return "SK_COLL";
        case St::SKIPPED_IMPOSSIBLE:  return "SK_IMPS";
        case St::SKIPPED_ERROR:       return "SK_ERR";
    }
    return "?";
}

std::string joinStatuses(const std::vector<InstructionOutcome::Status>& v) {
    std::string out;
    for (size_t i = 0; i < v.size(); i++) {
        if (i > 0) out += ",";
        out += statusName(v[i]);
    }
    return out;
}

} // namespace

// =============================================================================

O_StrategyJsonRunnerTest::O_StrategyJsonRunnerTest()
    : FunctionalTest("StrategyJsonRunner",
                     "Test integration StrategyJsonRunner (9 scenarios SR01..SR09).",
                     "jrun")
{
}

// =============================================================================

void O_StrategyJsonRunnerTest::run(int /*argc*/, char** /*argv*/)
{
    logger().info() << "========================================" << logs::end;
    logger().info() << "O_StrategyJsonRunnerTest : " << SCENARIOS.size() << " scenarios" << logs::end;
    logger().info() << "========================================" << logs::end;

    // Setup commun : sensors thread pour SR06 (adv). On l'active pour tous
    // pour simplifier — sur scenarios sans adv, le thread tourne mais ne
    // declenche aucun evenement.
    OPOS6UL_RobotExtended& robot = OPOS6UL_RobotExtended::instance();
    auto& sensors = robot.actions().sensors();
    sensors.addConfigFront(false, true, false);
    sensors.addConfigBack(false, true, false);
    sensors.setIgnoreFrontNearObstacle(true, false, true);   // front centre actif
    sensors.setIgnoreBackNearObstacle(true, false, true);    // back centre actif (aligne sur match)
    sensors.startSensorsThread(20);
    utils::sleep_for_micros(200000);

    // Init pathfinding : equivalent du flux production O_State_DecisionMakerIA::execute()
    // qui appelle robot.ia().initPlayground() avant de creer le StrategyJsonRunner.
    // Sans ca, IAbyPath::p_ reste NULL et tout PATH_TO crashe (cf. SR09).
    robot.ia().initPlayground();

    std::vector<SrResult> results;
    results.reserve(SCENARIOS.size());

    for (const auto& spec : SCENARIOS) {
        SrResult r;
        r.name                     = spec.name;
        r.desc                     = spec.desc;
        r.jsonFile                 = spec.jsonFile;
        r.startX                   = spec.startX;
        r.startY                   = spec.startY;
        r.startThetaDeg            = spec.startThetaDeg;
        r.hasAdv                   = spec.hasAdv;
        r.advX                     = spec.advX;
        r.advY                     = spec.advY;
        r.expectedTrace            = spec.expectedTrace;
        r.expectedFlags            = spec.expectedFlags;
        r.expectedOutcomeStatuses  = spec.expectedOutcomeStatuses;

        runScenario(r);
        // Abort attendu : on verifie que run() a bien abort quand attendu
        if (r.failReason.empty()) {
            // runScenario n'a pas flag d'erreur infra; on compare traces dans checkVerdict.
        }
        checkVerdict(r);
        logScenario(r);
        results.push_back(r);
    }

    // Teardown : clear adv, robot stop
    sensors.clearInjectedAdv();
    robot.freeMotion();

    int pass = 0;
    for (const auto& r : results) if (r.pass) pass++;
    logger().info() << "========================================" << logs::end;
    logger().info() << "Resultat : " << pass << " / " << results.size() << " PASS" << logs::end;
    logger().info() << "========================================" << logs::end;

    writeSvg(results);
}

// =============================================================================
// runScenario : execute un scenario via le vrai StrategyJsonRunner
// =============================================================================

void O_StrategyJsonRunnerTest::runScenario(SrResult& res)
{
    OPOS6UL_RobotExtended& robot = OPOS6UL_RobotExtended::instance();
    auto& sensors = robot.actions().sensors();

    logger().info() << "------ " << res.name << " : " << res.desc << " ------" << logs::end;

    // 1. Reset position + chrono match (necessaire pour max_match_sec dans SR08)
    robot.asserv().setPositionAndColor(res.startX, res.startY, res.startThetaDeg, false);
    robot.chrono().start();   // (re)demarre le chrono match a t=0 pour ce scenario
    utils::sleep_for_micros(150000);

    // 2. Adv (inject ou clear)
    sensors.clearInjectedAdv();
    if (res.hasAdv) {
        sensors.setInjectedAdv(res.advX, res.advY);
        logger().info() << "  adv injecte (" << res.advX << "," << res.advY << ")" << logs::end;
    }

    // 2b. SR09 specifique : on active la zone "area_test_blocker" du
    //     playground (zone dynamique non permanente declaree dans
    //     initPlayground, desactivee par defaut). Cible (1800,1000) du
    //     scenario tombe dedans -> A* renvoie cost=0 -> SK_IMPS.
    //     Desactivee pour tous les autres scenarios (aucun impact).
    bool enableTestBlocker = (std::string(res.name) == "SR09");
    robot.ia().iAbyPath().playground()->enable(
        robot.ia().area_test_blocker, enableTestBlocker);
    utils::sleep_for_micros(150000);

    // 3. ActionRegistry : actions mock qui loggent dans res.trace
    ActionRegistry actions;
    auto tracer = [&res](const std::string& id) {
        return [&res, id]() {
            res.trace.push_back(id);
            return true;
        };
    };
    actions.registerAction("trace_i1", tracer("trace_i1"));
    actions.registerAction("trace_i2", tracer("trace_i2"));
    actions.registerAction("trace_i3", tracer("trace_i3"));
    actions.registerAction("trace_t1", tracer("trace_t1"));
    actions.registerAction("trace_t2", tracer("trace_t2"));
    actions.registerAction("trace_t3", tracer("trace_t3"));

    // 4. FlagManager neuf pour chaque scenario
    FlagManager flags;

    // 5. Runner
    StrategyJsonRunner runner(&robot, &robot.ia().iAbyPath(), &actions, &flags);

    utils::Chronometer chrono("sr");
    chrono.start();

    if (!runner.loadFromFile(res.jsonFile)) {
        res.failReason = "loadFromFile a echoue (JSON introuvable ?)";
        chrono.stop();
        res.durationMs = chrono.getElapsedTimeInMilliSec();
        return;
    }
    // Validation anticipee identique au flux production (cf O_State_DecisionMakerIA).
    // Detecte au chargement les cibles tombant dans une zone permanente
    // (bordures, grenier...) avant le match. Les SR01..SR09 actuels ont des
    // cibles valides : aucune ne doit fail.
    if (!runner.validateAgainstPlayground(&robot.ia().iAbyPath())) {
        res.failReason = "validateAgainstPlayground a echoue (cible en zone permanente ?)";
        chrono.stop();
        res.durationMs = chrono.getElapsedTimeInMilliSec();
        return;
    }
    bool completed = runner.run();
    (void)completed;  // v2 : run() renvoie toujours true ; verifie via outcomes()

    chrono.stop();
    res.durationMs = chrono.getElapsedTimeInMilliSec();

    // 6. Snapshot final
    ROBOTPOSITION p = robot.asserv().pos_getPosition();
    res.endX = p.x;
    res.endY = p.y;
    res.endThetaDeg = p.theta * 180.0f / (float)M_PI;
    res.finalFlags = flags.all();
    res.outcomeStatuses.clear();
    for (const auto& o : runner.outcomes()) {
        res.outcomeStatuses.push_back(o.status);
    }

    // 7. Cleanup adv avant scenario suivant
    sensors.clearInjectedAdv();
}

// =============================================================================
// checkVerdict : compare trace + flags attendus vs reels
// =============================================================================

void O_StrategyJsonRunnerTest::checkVerdict(SrResult& res)
{
    if (!res.failReason.empty()) {
        res.pass = false;
        return;
    }
    if (res.trace != res.expectedTrace) {
        res.failReason = "trace mismatch : attendu=[" + joinTrace(res.expectedTrace)
                       + "] reel=[" + joinTrace(res.trace) + "]";
        res.pass = false;
        return;
    }
    if (res.finalFlags != res.expectedFlags) {
        res.failReason = "flags mismatch : attendu={" + joinFlags(res.expectedFlags)
                       + "} reel={" + joinFlags(res.finalFlags) + "}";
        res.pass = false;
        return;
    }
    if (res.outcomeStatuses != res.expectedOutcomeStatuses) {
        res.failReason = "outcomes mismatch : attendu=[" + joinStatuses(res.expectedOutcomeStatuses)
                       + "] reel=[" + joinStatuses(res.outcomeStatuses) + "]";
        res.pass = false;
        return;
    }
    res.pass = true;
}

// =============================================================================

void O_StrategyJsonRunnerTest::logScenario(const SrResult& res)
{
    const char* status = res.pass ? "PASS" : "FAIL";
    logger().info() << "  [" << status << "] " << res.name
                    << " trace=[" << joinTrace(res.trace) << "]"
                    << " flags={" << joinFlags(res.finalFlags) << "}"
                    << " outcomes=[" << joinStatuses(res.outcomeStatuses) << "]"
                    << " end=(" << (int)res.endX << "," << (int)res.endY << ")"
                    << " dur=" << (int)res.durationMs << "ms" << logs::end;
    if (!res.pass) {
        logger().error() << "    reason: " << res.failReason << logs::end;
    }
}

// =============================================================================
// SVG de synthese : grille 2x3, 1 cellule par scenario
// =============================================================================

std::string O_StrategyJsonRunnerTest::exeDirectory()
{
    char buf[PATH_MAX];
    ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0) return ".";
    buf[n] = '\0';
    std::string path(buf);
    size_t slash = path.find_last_of('/');
    return (slash == std::string::npos) ? std::string(".") : path.substr(0, slash);
}

void O_StrategyJsonRunnerTest::writeSvg(const std::vector<SrResult>& all)
{
    const int    cols   = 2;
    const int    rows   = (int)((all.size() + cols - 1) / cols);
    const float  cellW  = 820.0f;
    const float  cellH  = 420.0f;
    const float  totalW = cols * cellW;
    const float  totalH = rows * cellH + 60;

    std::ostringstream svg;
    svg << "<?xml version='1.0' encoding='UTF-8'?>\n";
    svg << "<svg xmlns='http://www.w3.org/2000/svg' width='" << totalW
        << "' height='" << totalH << "' viewBox='0 0 " << totalW << " " << totalH << "'>\n";

    int pass = 0;
    for (const auto& r : all) if (r.pass) pass++;

    svg << "<text x='10' y='28' font-family='monospace' font-size='20' font-weight='bold'>"
        << "StrategyJsonRunner Scenarios (6 SR)</text>\n";
    svg << "<text x='10' y='50' font-family='monospace' font-size='13' font-weight='bold' fill='"
        << (pass == (int)all.size() ? "#228B22" : "#DC143C") << "'>"
        << pass << " / " << all.size() << " PASS</text>\n";

    for (size_t i = 0; i < all.size(); i++) {
        int r = (int)(i / cols);
        int c = (int)(i % cols);
        float x = c * cellW;
        float y = 60 + r * cellH;
        svg << svgCell(all[i], x, y, cellW, cellH);
    }

    svg << "</svg>\n";

    std::string path = exeDirectory() + "/test_strategy_runner.svg";
    std::ofstream ofs(path);
    if (ofs) {
        ofs << svg.str();
        logger().info() << "SVG synthese ecrit : " << path << logs::end;
    } else {
        logger().error() << "impossible d'ecrire le SVG : " << path << logs::end;
    }
}

std::string O_StrategyJsonRunnerTest::svgCell(const SrResult& sc, float xOff, float yOff,
                                              float cellW, float cellH)
{
    std::ostringstream s;

    // Background cell
    s << "<g transform='translate(" << xOff << "," << yOff << ")'>\n";
    s << "<rect x='0' y='0' width='" << cellW << "' height='" << cellH
      << "' fill='#fafafa' stroke='#888' stroke-width='2'/>\n";

    // Titre + verdict
    s << "<text x='15' y='25' font-family='monospace' font-size='16' font-weight='bold'>"
      << sc.name << " &#8212; " << svgEscape(sc.desc) << "</text>\n";
    const char* vcol = sc.pass ? "#228B22" : "#DC143C";
    const char* vtxt = sc.pass ? "PASS" : "FAIL";
    s << "<text x='" << (cellW - 75) << "' y='25' font-family='monospace' font-size='18'"
      << " font-weight='bold' fill='" << vcol << "'>" << vtxt << "</text>\n";
    s << "<text x='15' y='42' font-family='monospace' font-size='11' fill='#666'>"
      << "fichier=" << sc.jsonFile << "  duree=" << (int)sc.durationMs << "ms</text>\n";

    // --- Gauche : table miniature avec robot/trajectoire/adv ---
    // Echelle table : 3000x2000 -> 460x310 (pixels)
    const float scale = 460.0f / 3000.0f;
    const float tblX  = 15.0f;
    const float tblY  = 55.0f;
    const float tblW  = 3000.0f * scale;
    const float tblH  = 2000.0f * scale;

    s << "<g transform='translate(" << tblX << "," << tblY << ")'>\n";
    s << "<rect x='0' y='0' width='" << tblW << "' height='" << tblH
      << "' fill='#e8e8e8' stroke='#333' stroke-width='1'/>\n";

    // PMX : origine bas-gauche, y vers le haut. SVG : origine haut-gauche, y vers le bas.
    // Lambda pour convertir (x, y) mm -> (X, Y) pixel local.
    auto tx = [scale](float x) { return x * scale; };
    auto ty = [scale, tblH](float y) { return tblH - y * scale; };

    // Trajectoire depart -> fin (segment simple si pas de waypoints intermediaires)
    s << "<line x1='" << tx(sc.startX) << "' y1='" << ty(sc.startY)
      << "' x2='" << tx(sc.endX) << "' y2='" << ty(sc.endY)
      << "' stroke='" << (sc.pass ? "#0055cc" : "#c0392b") << "' stroke-width='2.5'/>\n";

    // Robot depart (cercle + fleche orientation)
    s << "<circle cx='" << tx(sc.startX) << "' cy='" << ty(sc.startY) << "' r='"
      << (ROBOT_DIAMETER_MM * 0.5f * scale)
      << "' fill='#4A90E2' fill-opacity='0.25' stroke='#0066cc' stroke-width='2'/>\n";
    float rad = sc.startThetaDeg * (float)M_PI / 180.0f;
    float arL = 70.0f;
    s << "<line x1='" << tx(sc.startX) << "' y1='" << ty(sc.startY)
      << "' x2='" << tx(sc.startX + arL * std::cos(rad)) << "' y2='" << ty(sc.startY + arL * std::sin(rad))
      << "' stroke='#0066cc' stroke-width='2'/>\n";

    // Robot fin (cercle pointille)
    s << "<circle cx='" << tx(sc.endX) << "' cy='" << ty(sc.endY) << "' r='"
      << (ROBOT_DIAMETER_MM * 0.5f * scale)
      << "' fill='none' stroke='" << (sc.pass ? "#228B22" : "#c0392b")
      << "' stroke-width='2' stroke-dasharray='4,3'/>\n";

    // Adversaire (si present)
    if (sc.hasAdv) {
        s << "<circle cx='" << tx(sc.advX) << "' cy='" << ty(sc.advY) << "' r='"
          << (ADV_DIAMETER_MM * 0.5f * scale)
          << "' fill='#DC143C' fill-opacity='0.25' stroke='#8B0000' stroke-width='2'/>\n";
        s << "<circle cx='" << tx(sc.advX) << "' cy='" << ty(sc.advY)
          << "' r='4' fill='#8B0000'/>\n";
    }
    s << "</g>\n";

    // --- Droite : panel texte ---
    float px = tblX + tblW + 15;
    float py = 65;

    auto section = [&](float y, const std::string& title) {
        s << "<text x='" << px << "' y='" << y
          << "' font-family='monospace' font-size='12' font-weight='bold' fill='#444'>"
          << title << "</text>\n";
    };
    auto line = [&](float y, const std::string& text, const char* col = "#222") {
        s << "<text x='" << px << "' y='" << y
          << "' font-family='monospace' font-size='11' fill='" << col << "'>"
          << svgEscape(text) << "</text>\n";
    };

    section(py, "Trace attendue :"); py += 16;
    line(py, "  [" + joinTrace(sc.expectedTrace) + "]", "#555"); py += 20;

    section(py, "Trace reelle :"); py += 16;
    line(py, "  [" + joinTrace(sc.trace) + "]",
         sc.trace == sc.expectedTrace ? "#228B22" : "#c0392b"); py += 22;

    section(py, "Flags attendus :"); py += 16;
    line(py, "  {" + joinFlags(sc.expectedFlags) + "}", "#555"); py += 20;

    section(py, "Flags reels :"); py += 16;
    line(py, "  {" + joinFlags(sc.finalFlags) + "}",
         sc.finalFlags == sc.expectedFlags ? "#228B22" : "#c0392b"); py += 22;

    section(py, "Outcomes attendus :"); py += 16;
    line(py, "  [" + joinStatuses(sc.expectedOutcomeStatuses) + "]", "#555"); py += 20;

    section(py, "Outcomes reels :"); py += 16;
    line(py, "  [" + joinStatuses(sc.outcomeStatuses) + "]",
         sc.outcomeStatuses == sc.expectedOutcomeStatuses ? "#228B22" : "#c0392b"); py += 22;

    section(py, "Position finale :"); py += 16;
    char posbuf[64];
    std::snprintf(posbuf, sizeof(posbuf), "  (%d, %d) theta=%d deg",
                  (int)sc.endX, (int)sc.endY, (int)sc.endThetaDeg);
    line(py, posbuf); py += 22;

    if (!sc.pass && !sc.failReason.empty()) {
        section(py, "Raison echec :"); py += 16;
        line(py, "  " + sc.failReason, "#c0392b");
    }

    s << "</g>\n";
    return s.str();
}
