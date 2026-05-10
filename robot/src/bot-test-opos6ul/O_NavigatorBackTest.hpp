/*!
 * \file
 * \brief Declaration de la classe O_NavigatorBackTest.
 */

#ifndef O_NAVIGATORBACKTEST_HPP
#define O_NAVIGATORBACKTEST_HPP

#include "utils/FunctionalTest.hpp"
#include "log/LoggerFactory.hpp"
#include "interface/AAsservDriver.hpp"

/*!
 * \brief Test de non-regression des fonctions Navigator en marche arriere.
 *
 * 10 boucles independantes en miroir de O_NavigatorMovementTest (forward).
 * Memes positions de depart, memes patterns mais commandes back/arriere.
 * On peut commenter/decommenter chaque boucle dans run() pour tester individuellement.
 *
 * Boucle  1 : Mouvements directs back (bleu)              — line(-x), moveBackwardTo, faceBackTo, faceTo
 * Boucle  2 : Combinaisons goBackTo+ (bleu)               — goBackToAndRotateAbsDeg/RelDeg/FaceTo/FaceBackTo
 * Boucle  3 : Combinaisons moveBackwardTo+ (bleu)         — moveBackwardToAndRotateAbsDeg/RelDeg/FaceTo/FaceBackTo
 * Boucle  4 : manualPath STOP back (pas de trait)          — carre 200mm waypoints back
 * Boucle  5 : manualPath CHAIN back (vert continu)         — carre 300mm waypoints back
 * Boucle  6 : manualPath CHAIN_NONSTOP back (vert point.)  — carre 400mm waypoints back
 * Boucle  7 : Modes compare triangle back                  — STOP / CHAIN / CHAIN_NONSTOP
 * Boucle  8 : Orbital turn                                — pivot gauche/droit avant et arriere
 * Boucle  9 : pathBackTo + pathBackToAnd* (rouge)          — 4 combos + pathTo retour
 * Boucle 10 : pathBackTo evitement obstacle (rouge)
 *
 *   ./bot-opos6ul navb
 */
class O_NavigatorBackTest: public FunctionalTest
{
private:

    static inline const logs::Logger & logger()
    {
        static const logs::Logger & instance = logs::LoggerFactory::logger("O_NavigatorBackTest");
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
    void loop1_DirectBack();
    void loop2_GoBackToCombos();
    void loop3_MoveBackwardCombos();
    void loop4_ManualPathStopBack();
    void loop5_ManualPathChainBack();
    void loop6_ManualPathChainNonstopBack();
    void loop7_ModesCompareBack();
    void loop8_OrbitalTurn();
    void initPlayground();
    void loop9_PathBackFinding();
    void loop10_PathBackAvoidObstacle();

public:

    O_NavigatorBackTest() :
            FunctionalTest("Navigator_Back", "Test non-regression Navigator marche arriere.", "navb")
    {
        passed_ = 0;
        failed_ = 0;
    }

    virtual ~O_NavigatorBackTest()
    {
    }

    std::string usageHelp() const override
    {
        return
            "        args: aucun (10 boucles autonomes en marche arriere)\n"
            "        opts: /M multiplier (simu : 0=instantane, 1.0=temps reel, defaut 2.0)\n"
            "        ex:   navb       # tests goBackTo, moveBackwardTo, pathBackTo + combos";
    }

    virtual void run(int argc, char** argv);
};

#endif
