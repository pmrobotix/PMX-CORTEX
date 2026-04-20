/*!
 * \file
 * \brief Scenarios marche ARRIERE pour la detection adversaire.
 *
 * Miroir des scenarios forward : meme geometrie mais robot se dirige en
 * marche arriere. Positions calees pour rester dans la table (3000x2000).
 */

#include "O_DetectionBackwardTest.hpp"

const char* O_DetectionBackwardTest::SCENARIOS_JSON = R"JSON(
[
  {
    "name": "B01_GoBackTo_Clear",
    "desc": "GO_BACK_TO tout droit, sans adv",
    "start": {"x": 2800, "y": 200, "theta_deg": 0},
    "adv": null,
    "expected": "FINISHED",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "GO_BACK_TO", "position_x": 2200, "position_y": 200}
    ]
  },
  {
    "name": "B02_GoBackTo_AdvOnPath",
    "desc": "GO_BACK_TO avec adv a 400mm derriere sur le segment direct",
    "start": {"x": 2800, "y": 200, "theta_deg": 0},
    "adv": {"x": 2400, "y": 200},
    "expected": "NEAR_OBSTACLE",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "GO_BACK_TO", "position_x": 2000, "position_y": 200}
    ]
  },
  {
    "name": "B03_GoBackTo_AdvBesideLeft",
    "desc": "GO_BACK_TO avec adv lateral cote GAUCHE robot (x_rep<0, hors couloir)",
    "start": {"x": 2800, "y": 200, "theta_deg": 0},
    "adv": {"x": 2500, "y": 600},
    "expected": "FINISHED",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "GO_BACK_TO", "position_x": 2200, "position_y": 200}
    ]
  },
  {
    "name": "B03b_GoBackTo_AdvBesideRight",
    "desc": "GO_BACK_TO avec adv lateral cote DROIT robot (x_rep>0, hors couloir)",
    "start": {"x": 2800, "y": 1800, "theta_deg": 0},
    "adv": {"x": 2500, "y": 1400},
    "expected": "FINISHED",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "GO_BACK_TO", "position_x": 2200, "position_y": 1800}
    ]
  },
  {
    "name": "B03c_GoBackTo_AdvSlowLeft",
    "desc": "GO_BACK_TO adv lateral 420mm GAUCHE (zone JAUNE SLOW arriere, pas STOP)",
    "start": {"x": 2800, "y": 200, "theta_deg": 0},
    "adv": {"x": 2200, "y": 620},
    "expected": "FINISHED",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "GO_BACK_TO", "position_x": 1400, "position_y": 200}
    ]
  },
  {
    "name": "B03d_GoBackTo_AdvSlowRight",
    "desc": "GO_BACK_TO adv lateral 420mm DROITE (zone JAUNE SLOW arriere)",
    "start": {"x": 2800, "y": 1800, "theta_deg": 0},
    "adv": {"x": 2200, "y": 1380},
    "expected": "FINISHED",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "GO_BACK_TO", "position_x": 1400, "position_y": 1800}
    ]
  },
  {
    "name": "B04_GoBackTo_AdvBehindTargetClose",
    "desc": "GO_BACK_TO adv 300mm au-dela de la cible (cone ToF arriere voit adv a l'arrivee)",
    "start": {"x": 2800, "y": 200, "theta_deg": 0},
    "adv": {"x": 1900, "y": 200},
    "expected": "NEAR_OBSTACLE",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "GO_BACK_TO", "position_x": 2200, "position_y": 200}
    ]
  },
  {
    "name": "B05_GoBackTo_AdvFarBehindTarget",
    "desc": "GO_BACK_TO adv 1000mm au-dela de la cible, hors zone detection",
    "start": {"x": 2800, "y": 200, "theta_deg": 0},
    "adv": {"x": 1200, "y": 200},
    "expected": "FINISHED",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "GO_BACK_TO", "position_x": 2200, "position_y": 200}
    ]
  },
  {
    "name": "B06_MoveBwd_Clear",
    "desc": "MOVE_BACKWARD_TO diagonale sans adv",
    "start": {"x": 2800, "y": 1800, "theta_deg": 0},
    "adv": null,
    "expected": "FINISHED",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "MOVE_BACKWARD_TO", "position_x": 2200, "position_y": 1200}
    ]
  },
  {
    "name": "B07_MoveBwd_AdvOnPath",
    "desc": "MOVE_BACKWARD_TO diagonale, adv a 141mm du segment (dans couloir 340mm)",
    "start": {"x": 2800, "y": 1800, "theta_deg": 0},
    "adv": {"x": 2600, "y": 1400},
    "expected": "NEAR_OBSTACLE",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "MOVE_BACKWARD_TO", "position_x": 2200, "position_y": 1200}
    ]
  },
  {
    "name": "B08_MoveBwd_AdvBesideLeft",
    "desc": "MOVE_BACKWARD_TO diagonale, adv cote GAUCHE du chemin (hors couloir)",
    "start": {"x": 2800, "y": 1800, "theta_deg": 0},
    "adv": {"x": 2800, "y": 1200},
    "expected": "FINISHED",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "MOVE_BACKWARD_TO", "position_x": 2200, "position_y": 1200}
    ]
  },
  {
    "name": "B08b_MoveBwd_AdvBesideRight",
    "desc": "MOVE_BACKWARD_TO diagonale, adv cote DROIT du chemin (miroir de B08)",
    "start": {"x": 2800, "y": 1800, "theta_deg": 0},
    "adv": {"x": 2200, "y": 1800},
    "expected": "FINISHED",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "MOVE_BACKWARD_TO", "position_x": 2200, "position_y": 1200}
    ]
  },
  {
    "name": "B09_MoveBwd_AdvBehindTargetClose",
    "desc": "MOVE_BACKWARD_TO adv 300mm au-dela de la cible dans l'axe du move",
    "start": {"x": 2800, "y": 1800, "theta_deg": 0},
    "adv": {"x": 1988, "y": 988},
    "expected": "NEAR_OBSTACLE",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "MOVE_BACKWARD_TO", "position_x": 2200, "position_y": 1200}
    ]
  },
  {
    "name": "B10_MoveBwd_AdvFarBehindTarget",
    "desc": "MOVE_BACKWARD_TO adv 1000mm au-dela de la cible (hors detection)",
    "start": {"x": 2800, "y": 1800, "theta_deg": 0},
    "adv": {"x": 1493, "y": 493},
    "expected": "FINISHED",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "MOVE_BACKWARD_TO", "position_x": 2200, "position_y": 1200}
    ]
  },
  {
    "name": "B11_FaceBackTo_AdvInBackDir",
    "desc": "FACE_BACK_TO vers cible avec adv sur cette direction (rotation, detection bypass)",
    "start": {"x": 200, "y": 200, "theta_deg": 0},
    "adv": {"x": 800, "y": 800},
    "expected": "FINISHED",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "FACE_BACK_TO", "position_x": 800, "position_y": 800}
    ]
  }
]
)JSON";

O_DetectionBackwardTest::O_DetectionBackwardTest()
    : O_DetectionScenarioTest("Detection_Backward",
                               "Tests detection adv en marche ARRIERE (GO_BACK_TO + MOVE_BACKWARD_TO + FACE_BACK_TO).",
                               "detb",
                               SCENARIOS_JSON,
                               "test_detection_backward.svg")
{
}
