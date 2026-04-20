/*!
 * \file
 * \brief Scenarios marche AVANT pour la detection adversaire.
 */

#include "O_DetectionForwardTest.hpp"

const char* O_DetectionForwardTest::SCENARIOS_JSON = R"JSON(
[
  {
    "name": "F01_GoTo_Clear",
    "desc": "GO_TO tout droit, sans adv",
    "start": {"x": 200, "y": 200, "theta_deg": 0},
    "adv": null,
    "expected": "FINISHED",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "GO_TO", "position_x": 800, "position_y": 200}
    ]
  },
  {
    "name": "F02_GoTo_AdvOnPath",
    "desc": "GO_TO avec adv a 400mm devant sur le segment direct",
    "start": {"x": 200, "y": 200, "theta_deg": 0},
    "adv": {"x": 600, "y": 200},
    "expected": "NEAR_OBSTACLE",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "GO_TO", "position_x": 1000, "position_y": 200}
    ]
  },
  {
    "name": "F03_GoTo_AdvBesideLeft",
    "desc": "GO_TO avec adv lateral cote GAUCHE robot (x_rep<0, hors couloir)",
    "start": {"x": 200, "y": 200, "theta_deg": 0},
    "adv": {"x": 500, "y": 600},
    "expected": "FINISHED",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "GO_TO", "position_x": 800, "position_y": 200}
    ]
  },
  {
    "name": "F03b_GoTo_AdvBesideRight",
    "desc": "GO_TO avec adv lateral cote DROIT robot (x_rep>0, hors couloir)",
    "start": {"x": 200, "y": 1800, "theta_deg": 0},
    "adv": {"x": 500, "y": 1400},
    "expected": "FINISHED",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "GO_TO", "position_x": 800, "position_y": 1800}
    ]
  },
  {
    "name": "F03c_GoTo_AdvSlowLeft",
    "desc": "GO_TO adv lateral 420mm GAUCHE (zone JAUNE SLOW, pas STOP) - robot ralentit mais termine",
    "start": {"x": 200, "y": 200, "theta_deg": 0},
    "adv": {"x": 800, "y": 620},
    "expected": "FINISHED",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "GO_TO", "position_x": 1600, "position_y": 200}
    ]
  },
  {
    "name": "F03d_GoTo_AdvSlowRight",
    "desc": "GO_TO adv lateral 420mm DROITE (zone JAUNE SLOW, pas STOP)",
    "start": {"x": 200, "y": 1800, "theta_deg": 0},
    "adv": {"x": 800, "y": 1380},
    "expected": "FINISHED",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "GO_TO", "position_x": 1600, "position_y": 1800}
    ]
  },
  {
    "name": "F04_GoTo_AdvBehindTargetClose",
    "desc": "GO_TO adv 300mm au-dela de la cible, cone ToF voit adv a l'arrivee (STOP)",
    "start": {"x": 200, "y": 200, "theta_deg": 0},
    "adv": {"x": 1100, "y": 200},
    "expected": "NEAR_OBSTACLE",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "GO_TO", "position_x": 800, "position_y": 200}
    ]
  },
  {
    "name": "F05_GoTo_AdvFarBehindTarget",
    "desc": "GO_TO adv 1000mm au-dela de la cible, hors zone detection a l'arrivee",
    "start": {"x": 200, "y": 200, "theta_deg": 0},
    "adv": {"x": 1800, "y": 200},
    "expected": "FINISHED",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "GO_TO", "position_x": 800, "position_y": 200}
    ]
  },
  {
    "name": "F06_MoveFwd_Clear",
    "desc": "MOVE_FORWARD_TO diagonale sans adv",
    "start": {"x": 200, "y": 200, "theta_deg": 0},
    "adv": null,
    "expected": "FINISHED",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "MOVE_FORWARD_TO", "position_x": 800, "position_y": 800}
    ]
  },
  {
    "name": "F07_MoveFwd_AdvOnPath",
    "desc": "MOVE_FORWARD_TO diagonale, adv a 141mm du segment (dans couloir 340mm)",
    "start": {"x": 200, "y": 200, "theta_deg": 0},
    "adv": {"x": 400, "y": 600},
    "expected": "NEAR_OBSTACLE",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "MOVE_FORWARD_TO", "position_x": 800, "position_y": 800}
    ]
  },
  {
    "name": "F08_MoveFwd_AdvBesideLeft",
    "desc": "MOVE_FORWARD_TO diagonale, adv cote GAUCHE du chemin (hors couloir)",
    "start": {"x": 200, "y": 200, "theta_deg": 0},
    "adv": {"x": 200, "y": 800},
    "expected": "FINISHED",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "MOVE_FORWARD_TO", "position_x": 800, "position_y": 800}
    ]
  },
  {
    "name": "F08b_MoveFwd_AdvBesideRight",
    "desc": "MOVE_FORWARD_TO diagonale, adv cote DROIT du chemin (miroir de F08)",
    "start": {"x": 200, "y": 200, "theta_deg": 0},
    "adv": {"x": 800, "y": 200},
    "expected": "FINISHED",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "MOVE_FORWARD_TO", "position_x": 800, "position_y": 800}
    ]
  },
  {
    "name": "F09_MoveFwd_AdvBehindTargetClose",
    "desc": "MOVE_FORWARD_TO adv 300mm au-dela de la cible dans l'axe du move",
    "start": {"x": 200, "y": 200, "theta_deg": 0},
    "adv": {"x": 1012, "y": 1012},
    "expected": "NEAR_OBSTACLE",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "MOVE_FORWARD_TO", "position_x": 800, "position_y": 800}
    ]
  },
  {
    "name": "F10_MoveFwd_AdvFarBehindTarget",
    "desc": "MOVE_FORWARD_TO adv 1000mm au-dela de la cible (hors detection)",
    "start": {"x": 200, "y": 200, "theta_deg": 0},
    "adv": {"x": 1507, "y": 1507},
    "expected": "FINISHED",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "MOVE_FORWARD_TO", "position_x": 800, "position_y": 800}
    ]
  },
  {
    "name": "F11_FaceTo_AdvInTargetDir",
    "desc": "FACE_TO vers cible avec adv sur cette direction (rotation, detection bypass)",
    "start": {"x": 200, "y": 200, "theta_deg": 0},
    "adv": {"x": 800, "y": 800},
    "expected": "FINISHED",
    "tasks": [
      {"type": "MOVEMENT", "subtype": "FACE_TO", "position_x": 800, "position_y": 800}
    ]
  }
]
)JSON";

O_DetectionForwardTest::O_DetectionForwardTest()
    : O_DetectionScenarioTest("Detection_Forward",
                               "Tests detection adv en marche AVANT (GO_TO + MOVE_FORWARD_TO + FACE_TO).",
                               "detf",
                               SCENARIOS_JSON,
                               "test_detection_forward.svg")
{
}
