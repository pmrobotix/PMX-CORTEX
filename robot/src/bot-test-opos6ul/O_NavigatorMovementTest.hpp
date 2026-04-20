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
 * \brief Test de non-regression de toutes les fonctions Navigator.
 *
 * 8 boucles independantes, chacune revient a son point de depart.
 * On peut commenter/decommenter chaque boucle dans run() pour tester individuellement.
 * Pas de sensors, uniquement les moves.
 *
 * Boucle 1 : Mouvements directs (bleu)             — carre 100mm a (300,300)
 * Boucle 2 : Combinaisons (bleu)                   — carre 200mm a (300,500)
 * Boucle 3 : manualPath STOP (pas de trait)         — carre 200mm a (300,800)
 * Boucle 4 : manualPath CHAIN (vert continu)        — carre 300mm a (600,300)
 * Boucle 5 : manualPath CHAIN_NONSTOP (vert point.) — carre 400mm a (600,700)
 * Boucle 6 : Modes compare triangle                 — triangle a (1100,300)
 * Boucle 7 : Pathfinding (rouge)                    — a (1100,700)
 * Boucle 8 : Pathfinding evitement obstacle (rouge)  — a (1100,700)
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
    void loop2_Combinations();
    void loop3_ManualPathStop();
    void loop4_ManualPathChain();
    void loop5_ManualPathChainNonstop();
    void loop6_ModesCompare();
    void loop9_OrbitalTurn();
    void initPlayground();
    void loop7_Pathfinding();
    void loop8_PathfindingAvoidObstacle();

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

    virtual void run(int argc, char** argv);
};

#endif
