/*!
 * \file
 * \brief Scenarios retry backward.
 */

#include "O_DetectionRetryBackwardTest.hpp"

const char* O_DetectionRetryBackwardTest::SCENARIOS_JSON = R"JSON(
[
  {
    "name": "BR01_GoBackTo_RetrySuccess",
    "desc": "GO_BACK_TO adv derriere, disparait a 300ms, retry -> FINISHED",
    "start": {"x": 2800, "y": 200, "theta_deg": 0},
    "adv": {"x": 2400, "y": 200},
    "adv_clear_at_ms": 300,
    "retry": "quickTest",
    "expected": "FINISHED",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "GO_BACK_TO", "position_x": 2000, "position_y": 200}
    ]
  },
  {
    "name": "BR02_GoBackTo_RetryExhausted",
    "desc": "GO_BACK_TO adv derriere reste en place, 2 retries epuises -> NEAR_OBSTACLE",
    "start": {"x": 2800, "y": 200, "theta_deg": 0},
    "adv": {"x": 2400, "y": 200},
    "retry": "quickTest",
    "expected": "NEAR_OBSTACLE",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "GO_BACK_TO", "position_x": 2000, "position_y": 200}
    ]
  },
  {
    "name": "BR03_MoveBwd_RetrySuccess",
    "desc": "MOVE_BACKWARD_TO diagonale, adv disparait a 300ms -> FINISHED",
    "start": {"x": 2800, "y": 1800, "theta_deg": 0},
    "adv": {"x": 2600, "y": 1400},
    "adv_clear_at_ms": 300,
    "retry": "quickTest",
    "expected": "FINISHED",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "MOVE_BACKWARD_TO", "position_x": 2200, "position_y": 1200}
    ]
  },
  {
    "name": "BR04_MoveBwd_RetryExhausted",
    "desc": "MOVE_BACKWARD_TO diagonale, adv reste -> NEAR_OBSTACLE apres retries",
    "start": {"x": 2800, "y": 1800, "theta_deg": 0},
    "adv": {"x": 2600, "y": 1400},
    "retry": "quickTest",
    "expected": "NEAR_OBSTACLE",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "MOVE_BACKWARD_TO", "position_x": 2200, "position_y": 1200}
    ]
  }
]
)JSON";

O_DetectionRetryBackwardTest::O_DetectionRetryBackwardTest()
    : O_DetectionScenarioTest("Detection_Retry_Backward",
                               "Tests boucle retry Navigator en marche ARRIERE (adv qui bouge).",
                               "detrb",
                               SCENARIOS_JSON,
                               "test_detection_retry_backward.svg")
{
}
