/*!
 * \file
 * \brief Tests scenarios detection adversaire en marche AVANT (SIMU uniquement).
 *
 * Couvre GO_TO (primitive monolithique), MOVE_FORWARD_TO (decompose rotate+line)
 * et FACE_TO (rotation pure). Pour chaque primitive translationnelle on verifie
 * 5 cas canoniques : Clear / AdvOnPath / AdvBeside / AdvBehindTargetClose /
 * AdvFarBehindTarget.
 *
 * Invocation : ./bot-opos6ul /k detf
 * Sortie SVG : bin/test_detection_forward.svg
 */

#ifndef O_DETECTIONFORWARDTEST_HPP
#define O_DETECTIONFORWARDTEST_HPP

#include "O_DetectionScenarioTest.hpp"

class O_DetectionForwardTest : public O_DetectionScenarioTest
{
public:
    O_DetectionForwardTest();
    virtual ~O_DetectionForwardTest() {}

private:
    static const char* SCENARIOS_JSON;
};

#endif
