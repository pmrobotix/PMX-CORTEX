/*!
 * \file
 * \brief Declaration de la classe O_NavigatorMovementTest.
 */

#ifndef O_NAVIGATORMOVEMENTTEST_HPP
#define O_NAVIGATORMOVEMENTTEST_HPP

#include "utils/FunctionalTest.hpp"
#include "log/LoggerFactory.hpp"
#include "interface/AAsservDriver.hpp"

/*!
 * \brief Test de non-regression de toutes les fonctions Navigator (forward).
 *
 * 10 boucles independantes, chacune revient a son point de depart.
 * On peut commenter/decommenter chaque boucle dans run() pour tester individuellement.
 * Pas de sensors, uniquement les moves.
 *
 * Boucle  1 : Mouvements directs (bleu)               — line, goTo, goBackTo, rotations, faceTo, faceBackTo
 * Boucle  2 : Combinaisons goTo+ (bleu)               — goToAndRotateAbsDeg/RelDeg/FaceTo/FaceBackTo
 * Boucle  3 : Combinaisons moveForwardTo+ (bleu)      — moveForwardToAndRotateAbsDeg/RelDeg/FaceTo/FaceBackTo
 * Boucle  4 : manualPath STOP (pas de trait)           — carre 200mm
 * Boucle  5 : manualPath CHAIN (vert continu)          — carre 300mm
 * Boucle  6 : manualPath CHAIN_NONSTOP (vert point.)   — carre 400mm
 * Boucle  7 : Modes compare triangle                   — STOP / CHAIN / CHAIN_NONSTOP
 * Boucle  8 : Orbital turn                             — pivot gauche/droit avant et arriere
 * Boucle  9 : Pathfinding (rouge)                      — pathTo + pathToAnd* (4 combos) + pathBackTo retour
 * Boucle 10 : Pathfinding evitement obstacle (rouge)
 *
 *   ./bot-opos6ul nav
 */
class O_NavigatorMovementTest: public FunctionalTest
{
private:

    static inline const logs::Logger & logger()
    {
        static const logs::Logger & instance = logs::LoggerFactory::logger("O_NavigatorMovementTest");
        return instance;
    }

    int passed_;
    int failed_;

    // Helpers
    void resetPosition(float x, float y, float angleDeg);
    bool checkPosition(float expectedX, float expectedY, float tolerance, const std::string& name);
    bool checkAngle(float expectedDeg, float tolerance, const std::string& name);
    bool checkResult(TRAJ_STATE ts, TRAJ_STATE expected, const std::string& name);

    // Boucles
    void loop1_DirectMoves();
    void loop2_GoToCombos();
    void loop3_MoveForwardCombos();
    void loop4_ManualPathStop();
    void loop5_ManualPathChain();
    void loop6_ManualPathChainNonstop();
    void loop7_ModesCompare();
    void loop8_OrbitalTurn();
    void initPlayground();
    void loop9_Pathfinding();
    void loop10_PathfindingAvoidObstacle();

public:

    O_NavigatorMovementTest() :
            FunctionalTest("Navigator_Movement", "Test non-regression Navigator (toutes fonctions).", "nav")
    {
        passed_ = 0;
        failed_ = 0;
    }

    virtual ~O_NavigatorMovementTest()
    {
    }

    std::string usageHelp() const override
    {
        return
            "        args: aucun (10 boucles autonomes lancees en sequence)\n"
            "        opts: /M multiplier (simu : 0=instantane, 1.0=temps reel, defaut 2.0)\n"
            "        ex:   nav        # lance les 10 boucles (carre, manualPath, pathfinding, ...)";
    }

    virtual void run(int argc, char** argv);
};

#endif
