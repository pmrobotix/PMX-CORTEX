/*!
 * \file
 * \brief Tests retry Navigator en marche AVANT (SIMU uniquement).
 *
 * Valide la boucle Navigator::executeWithRetry avec :
 *   - RetryPolicy::quickTest (waitTempoUs=500ms, maxObstacleRetries=2)
 *   - adv_clear_at_ms : l'adv disparait apres N ms (simule un adv qui bouge)
 *
 * Scenarios :
 *   - RetrySuccess   : adv bloque, disparait pendant le wait -> retry -> FINISHED
 *   - RetryExhausted : adv reste, epuisement retries -> NEAR_OBSTACLE final
 *
 * Invocation : ./bot-opos6ul /k detrf
 * Sortie SVG : bin/test_detection_retry_forward.svg
 */

#ifndef O_DETECTIONRETRYFORWARDTEST_HPP
#define O_DETECTIONRETRYFORWARDTEST_HPP

#include "O_DetectionScenarioTest.hpp"

class O_DetectionRetryForwardTest : public O_DetectionScenarioTest
{
public:
    O_DetectionRetryForwardTest();
    virtual ~O_DetectionRetryForwardTest() {}

private:
    static const char* SCENARIOS_JSON;
};

#endif
