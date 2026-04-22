#ifndef O_STRATEGY_JSON_RUNNER_TEST_HPP
#define O_STRATEGY_JSON_RUNNER_TEST_HPP

#include "utils/FunctionalTest.hpp"
#include "log/LoggerFactory.hpp"

#include <set>
#include <string>
#include <vector>

/*!
 * \brief Test d'integration du StrategyJsonRunner (SIMU).
 *
 * Charge les 6 scenarios strategySR01..SR06.json et verifie end-to-end :
 *  - tri par priorite (SR01)
 *  - needed_flag / action_flag / clear_flags (SR02, SR05)
 *  - priority<0 desactive (SR03)
 *  - needed_flag au niveau task (SR04)
 *  - abort propre sur adversaire bloquant (SR06)
 *
 * Produit un SVG de synthese test_strategy_runner.svg (grille 2x3).
 *
 * CLI : ./bot-opos6ul t /n <num> (ou /k jrun).
 */
class O_StrategyJsonRunnerTest : public FunctionalTest
{
public:
    O_StrategyJsonRunnerTest();
    virtual ~O_StrategyJsonRunnerTest() {}

    virtual void run(int argc, char** argv) override;

private:

    struct SrResult
    {
        std::string name;            // SR01 .. SR06
        std::string desc;
        std::string jsonFile;        // "strategySR01.json"
        float       startX, startY, startThetaDeg;
        bool        hasAdv = false;
        float       advX = 0, advY = 0;

        // Trace des MANIPULATION appelees (order d'invocation)
        std::vector<std::string> trace;
        // Flags actifs en fin d'execution
        std::set<std::string>    finalFlags;
        // Position robot en fin de run
        float       endX = 0, endY = 0, endThetaDeg = 0;

        // Attendus (hardcodes par scenario dans le .cpp)
        std::vector<std::string> expectedTrace;
        std::set<std::string>    expectedFlags;

        bool        pass = false;
        std::string failReason;
        float       durationMs = 0;
    };

    static inline const logs::Logger& logger()
    {
        static const logs::Logger& instance = logs::LoggerFactory::logger("O_StrategyJsonRunnerTest");
        return instance;
    }

    void runScenario(SrResult& res);
    void checkVerdict(SrResult& res);
    void logScenario(const SrResult& res);
    void writeSvg(const std::vector<SrResult>& all);
    std::string svgCell(const SrResult& sc, float xOff, float yOff, float cellW, float cellH);
    std::string exeDirectory();
};

#endif
