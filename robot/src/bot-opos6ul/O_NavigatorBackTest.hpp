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
 * Boucles independantes, chacune revient a son point de depart.
 * On peut commenter/decommenter chaque boucle dans run() pour tester individuellement.
 *
 * Boucle 1 : Mouvements directs arriere (goBackTo, faceBackTo, moveBackwardTo)
 * Boucle 2 : Combinaisons avec FaceBackTo
 * Boucle 3 : manualPath mixte (waypoints avant + arriere)
 * Boucle 4 : manualPath tout arriere (CHAIN)
 * Boucle 5 : pathBackTo dans les 3 modes (STOP / CHAIN / CHAIN_NONSTOP)
 *
 *   ./bot-opos6ul t /n 11
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
    void loop2_CombinationsBack();
    void loop3_ManualPathMixed();
    void loop4_ManualPathAllBack();
    void initPlayground();
    void loop5_PathBackModes();

public:

    O_NavigatorBackTest() :
            FunctionalTest("Navigator_Back", "Test non-regression Navigator marche arriere.")
    {
        passed_ = 0;
        failed_ = 0;
    }

    virtual ~O_NavigatorBackTest()
    {
    }

    virtual void run(int argc, char** argv);
};

#endif
