/*!
 * \file
 * \brief Test de scenarios detection adversaire (SIMU uniquement).
 *
 * Baseline comportementale du robot face a un adversaire injecte.
 * Voir robot/md/DETECTION_SCENARIO_TEST.md pour le plan detaille.
 *
 * Les scenarios sont definis en format JSON aligne avec STRATEGY_JSON_FORMAT.md
 * (array d'instructions avec tasks MOVEMENT), enrichi des champs specifiques
 * tests (start, adv, expected). Hardcode dans le .cpp en raw string pour
 * edition facile, parse via nlohmann/json.
 *
 * Niveau A — Tactique pure : 1 mouvement, 1 adv plante, mesure TRAJ_STATE.
 * Niveau B — (a venir) Retry Navigator.
 * Niveau C — (a venir) Mini-strategie IA.
 *
 * Sorties :
 *   - Log ASCII dans la console
 *   - SVG de synthese : build-{preset}/bin/test_detection_scenarios.svg
 *
 * Invocation : ./bot-opos6ul /k det
 */

#ifndef O_DETECTIONSCENARIOTEST_HPP
#define O_DETECTIONSCENARIOTEST_HPP

#include "utils/FunctionalTest.hpp"
#include "log/LoggerFactory.hpp"
#include "interface/AAsservDriver.hpp"

#include <string>
#include <vector>

class Navigator;

class O_DetectionScenarioTest: public FunctionalTest
{
public:

    // Verdict a 3 niveaux : plus expressif que PASS/FAIL binaire.
    enum Verdict
    {
        VERDICT_PASS,              // actual == expected, trajectoire physiquement valide
        VERDICT_FAIL_STATE,        // actual != expected (bug detection, ou expected mal configure)
        VERDICT_WARN_CROSSED_ADV,  // actual == expected MAIS robot a traverse l'adv physiquement
                                   // -> bug detection (la simu n'a pas de physique de collision)
    };

    // Resultat d'un scenario execute (rempli par le test)
    struct SceneResult
    {
        std::string name;
        std::string command;      // subtype de la task principale
        bool        hasAdv;
        float       startX, startY, startThetaDeg;
        float       targetX, targetY;
        float       advX, advY;
        TRAJ_STATE  expected;
        TRAJ_STATE  actual;
        float       endX, endY, endThetaDeg;
        float       durationMs;
        bool        crossesAdv;   // trajectoire (start -> end) passe a < 340mm du centre adv
        Verdict     verdict;
    };

    O_DetectionScenarioTest() :
            FunctionalTest("Detection_Scenario",
                    "Baseline detection adversaire : scenarios SIMU 3 niveaux.",
                    "det")
    {
        passed_ = 0;
        warnCrossed_ = 0;
        failed_ = 0;
    }

    virtual ~O_DetectionScenarioTest() {}

    virtual void run(int argc, char** argv);

private:

    static inline const logs::Logger& logger()
    {
        static const logs::Logger& instance = logs::LoggerFactory::logger("O_DetectionScenarioTest");
        return instance;
    }

    int passed_;
    int warnCrossed_;
    int failed_;
    std::vector<SceneResult> scenesA_;

    // ---- Niveaux ----
    void runLevelA();

    // ---- Config robot pour le test ----
    void setupSensorsForTest();
    void teardownSensorsForTest();

    // ---- Helpers ----
    // Parse le JSON des scenarios et execute chaque instruction
    void executeScenariosFromJson(const char* jsonText, std::vector<SceneResult>& out);
    const char* trajName(TRAJ_STATE ts);
    const char* verdictName(Verdict v);
    TRAJ_STATE parseTrajExpected(const std::string& s);
    void logScene(const SceneResult& sc);
    // Distance minimale entre le point (px,py) et le segment [(ax,ay)->(bx,by)].
    static float distPointToSegment(float px, float py,
                                     float ax, float ay,
                                     float bx, float by);
    // Calcul du verdict final d'un scenario a partir de expected/actual/crossesAdv.
    static Verdict computeVerdict(const SceneResult& sc);

    // ---- SVG de synthese ----
    void writeSvgFile();
    std::string svgScene(const SceneResult& sc, float scale, float cellW, float cellH);
    std::string exeDirectory();
};

#endif
