/*!
 * \file
 * \brief Classe de base pour les tests scenarios detection adversaire (SIMU).
 *
 * Execute une liste de scenarios JSON (format aligne STRATEGY_JSON_FORMAT.md,
 * enrichi des champs test start/adv/expected) contre le SensorsDriverSimu +
 * AsservDriverSimu + Navigator. Produit un rapport ASCII + un SVG de synthese.
 *
 * Les sous-classes concretes fournissent simplement :
 *   - un code CLI (3 lettres)
 *   - un nom/description humain
 *   - la chaine JSON raw string des scenarios
 *   - un nom de fichier SVG
 *
 * Subclasses existantes :
 *   - O_DetectionForwardTest  (detf) : GO_TO + MOVE_FORWARD_TO + FACE_TO
 *   - O_DetectionBackwardTest (detb) : GO_BACK_TO + MOVE_BACKWARD_TO + FACE_BACK_TO
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
        int         advClearAtMs; // -1 = adv persistant, >=0 = adv cleare apres N ms (retry)
        Verdict     verdict;
    };

    virtual ~O_DetectionScenarioTest() {}

    virtual void run(int argc, char** argv);

protected:

    // Constructeur protected : instancie via les sous-classes concretes qui
    // fournissent les scenarios et le fichier SVG de sortie.
    O_DetectionScenarioTest(const std::string& name,
                             const std::string& desc,
                             const std::string& code,
                             const char* jsonText,
                             const std::string& svgFilename)
        : FunctionalTest(name, desc, code),
          scenariosJson_(jsonText),
          svgFilename_(svgFilename)
    {
        passed_ = 0;
        warnCrossed_ = 0;
        failed_ = 0;
    }

private:

    static inline const logs::Logger& logger()
    {
        static const logs::Logger& instance = logs::LoggerFactory::logger("O_DetectionScenarioTest");
        return instance;
    }

    const char* scenariosJson_;
    std::string svgFilename_;

    int passed_;
    int warnCrossed_;
    int failed_;
    std::vector<SceneResult> scenes_;

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
