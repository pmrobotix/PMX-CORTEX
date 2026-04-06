/*!
 * \file
 * \brief Implémentation de la classe OPOS6UL_RobotExtended.
 */

#include "OPOS6UL_RobotExtended.hpp"

#include <string>

#include "action/LcdShield.hpp"
#include "action/LedBar.hpp"
#include "geometry/TableGeometry.hpp"
#include "utils/Chronometer.hpp"
#include "log/Logger.hpp"
#include "thread/Thread.hpp"
#include "O_State_DecisionMakerIA.hpp"
#include "O_State_Init.hpp"
#include "O_State_WaitEndOfMatch.hpp"
#include "OPOS6UL_ActionsExtended.hpp"
#include "OPOS6UL_AsservExtended.hpp"
#include "OPOS6UL_IAExtended.hpp"
#include "OPOS6UL_SvgWriterExtended.hpp"

OPOS6UL_RobotExtended::OPOS6UL_RobotExtended()
{
    id_ = "OPOS6UL_Robot";
    myColor_ = PMXNOCOLOR;
    cArgs_.setDescription("(c) PM-ROBOTIX OPOS6UL_Robot");

    // Table 3000x2000 avec marge de 90mm (Coupe de France 2026)
    setTableGeometry(new TableGeometry(3000, 2000, 90, sharedPosition_));

    //on ecrase les versions par default avec la version extended
    OPOS6UL_SvgWriterExtended *p_svg = new OPOS6UL_SvgWriterExtended(id_);
    setSVG(p_svg);

    //init SVG log file AVANT la création de l'asserv (le thread CBOR écrit dans le SVG dès le start)
    p_svg->beginHeader();

    OPOS6UL_AsservExtended * p_asserv = new OPOS6UL_AsservExtended(id_, this);
    //asserv_default_ = p_asserv_;
    setAsserv(p_asserv);

    p_actions_ = new OPOS6UL_ActionsExtended(id_, this);
    actions_default_ = p_actions_;

    p_ia_ = new OPOS6UL_IAExtended(id_, this);

    decisionMaker_ = NULL;

    points = 0;
    //2023
    force_end_of_match = false;
}

OPOS6UL_RobotExtended::~OPOS6UL_RobotExtended()
{
    this->asserv().endWhatTodo(); //on termine le thread d'asserv qui lie la position
    this->actions().stopExtra(); //extra devices
}

void OPOS6UL_RobotExtended::displayPoints()
{
    this->actions().lcd2x16().clear();
    this->actions().lcd2x16().setCursor(0, 0);
    this->actions().lcd2x16().print("Points = ");
    this->actions().lcd2x16().print(this->points);



}

void OPOS6UL_RobotExtended::stopExtraActions() {
    this->actions().stopExtra(); //extra devices
}

void OPOS6UL_RobotExtended::begin(int argc, char** argv)
{
    Robot::begin(argc, argv);

    logger().debug() << "OPOS6UL_RobotExtended::begin" << logs::end;

    //specific match cases and strategies
    if (cArgs_["type"] == "m" or cArgs_["type"] == "M") {
        this->isEmpty(true);

        decisionMaker_ = new O_State_DecisionMakerIA(*this);

        IAutomateState* stateInit = new O_State_Init();
        IAutomateState* stateWaitEndOfMatch = new O_State_WaitEndOfMatch();
        stateInit->addState("WaitEndOfMatch", stateWaitEndOfMatch);

        decisionMaker_->start("O_State_DecisionMakerIA", 40);

        // Start the automate and wait for its return
        automate_.run(*this, stateInit);

        //attente du thread decisionMaker
        decisionMaker_->waitForEnd();
    }

    logger().info() << "PMX " << this->getID() << " Happy End - " << this->chrono().getElapsedTimeInSec() << " sec" << logs::end;
}

void OPOS6UL_RobotExtended::resetDisplayTS()
{
    actions().ledBar().resetAll();
}
//display des statuts de trajectoire
void OPOS6UL_RobotExtended::displayTS(TRAJ_STATE ts)
{

    if (ts == TRAJ_NEAR_OBSTACLE) {

        actions().ledBar().setOn(4);
        svgPrintPosition(3);
    }
    if (ts == TRAJ_COLLISION) {
        actions().ledBar().setOn(2);
        actions().ledBar().setOn(1);
        svgPrintPosition(5);
    }
    if (ts == TRAJ_FINISHED) {
        actions().ledBar().setOn(0);
        svgPrintPosition(3);
    }
}

void OPOS6UL_RobotExtended::resetDisplayObstacle()
{
    actions().ledBar().resetAll();

}

void OPOS6UL_RobotExtended::displayObstacle(int level) //TODO Front ou back ?
{
    if (level == 1)

        actions().ledBar().setOn(5);
    if (level == 2)
    {
        actions().ledBar().setOn(6);
        actions().ledBar().setOn(7);
    }
}
