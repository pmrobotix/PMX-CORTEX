/*!
 * \file
 * \brief Tests retry Navigator en marche ARRIERE (SIMU uniquement).
 *
 * Miroir de O_DetectionRetryForwardTest, meme logique retry mais GO_BACK_TO
 * et MOVE_BACKWARD_TO.
 *
 * Invocation : ./bot-opos6ul /k detrb
 * Sortie SVG : bin/test_detection_retry_backward.svg
 */

#ifndef O_DETECTIONRETRYBACKWARDTEST_HPP
#define O_DETECTIONRETRYBACKWARDTEST_HPP

#include "O_DetectionScenarioTest.hpp"

class O_DetectionRetryBackwardTest : public O_DetectionScenarioTest
{
public:
    O_DetectionRetryBackwardTest();
    virtual ~O_DetectionRetryBackwardTest() {}

private:
    static const char* SCENARIOS_JSON;
};

#endif
