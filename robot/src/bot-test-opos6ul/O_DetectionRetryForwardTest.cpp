/*!
 * \file
 * \brief Scenarios retry forward.
 */

#include "O_DetectionRetryForwardTest.hpp"

const char* O_DetectionRetryForwardTest::SCENARIOS_JSON = R"JSON(
[
  {
    "name": "FR01_GoTo_RetrySuccess",
    "desc": "GO_TO adv sur chemin, adv disparait a 300ms, retry a 500ms -> FINISHED",
    "start": {"x": 200, "y": 200, "theta_deg": 0},
    "adv": {"x": 600, "y": 200},
    "adv_clear_at_ms": 300,
    "retry": "quickTest",
    "expected": "FINISHED",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "GO_TO", "position_x": 1000, "position_y": 200}
    ]
  },
  {
    "name": "FR02_GoTo_RetryExhausted",
    "desc": "GO_TO adv sur chemin, reste en place, 2 retries epuises -> NEAR_OBSTACLE",
    "start": {"x": 200, "y": 200, "theta_deg": 0},
    "adv": {"x": 600, "y": 200},
    "retry": "quickTest",
    "expected": "NEAR_OBSTACLE",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "GO_TO", "position_x": 1000, "position_y": 200}
    ]
  },
  {
    "name": "FR03_MoveFwd_RetrySuccess",
    "desc": "MOVE_FORWARD_TO diagonale, adv disparait a 300ms -> FINISHED",
    "start": {"x": 200, "y": 200, "theta_deg": 0},
    "adv": {"x": 400, "y": 600},
    "adv_clear_at_ms": 300,
    "retry": "quickTest",
    "expected": "FINISHED",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "MOVE_FORWARD_TO", "position_x": 800, "position_y": 800}
    ]
  },
  {
    "name": "FR04_MoveFwd_RetryExhausted",
    "desc": "MOVE_FORWARD_TO diagonale, adv reste -> NEAR_OBSTACLE apres retries",
    "start": {"x": 200, "y": 200, "theta_deg": 0},
    "adv": {"x": 400, "y": 600},
    "retry": "quickTest",
    "expected": "NEAR_OBSTACLE",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "MOVE_FORWARD_TO", "position_x": 800, "position_y": 800}
    ]
  }
]
)JSON";

O_DetectionRetryForwardTest::O_DetectionRetryForwardTest()
    : O_DetectionScenarioTest("Detection_Retry_Forward",
                               "Tests boucle retry Navigator en marche AVANT (adv qui bouge).",
                               "detrf",
                               SCENARIOS_JSON,
                               "test_detection_retry_forward.svg")
{
}
