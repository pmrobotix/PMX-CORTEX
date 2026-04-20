/*!
 * \file
 * \brief Tests scenarios detection adversaire en marche ARRIERE (SIMU uniquement).
 *
 * Couvre GO_BACK_TO (primitive monolithique), MOVE_BACKWARD_TO (decompose
 * rotate+line) et FACE_BACK_TO (rotation pure). Miroir exact du fichier forward
 * (memes 5 cas canoniques par primitive translationnelle).
 *
 * Invocation : ./bot-opos6ul /k detb
 * Sortie SVG : bin/test_detection_backward.svg
 */

#ifndef O_DETECTIONBACKWARDTEST_HPP
#define O_DETECTIONBACKWARDTEST_HPP

#include "O_DetectionScenarioTest.hpp"

class O_DetectionBackwardTest : public O_DetectionScenarioTest
{
public:
    O_DetectionBackwardTest();
    virtual ~O_DetectionBackwardTest() {}

private:
    static const char* SCENARIOS_JSON;
};

#endif
