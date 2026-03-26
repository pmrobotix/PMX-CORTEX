//TODO: reimplementer la strategie pour 2026 (ancien code PMX 2025 : 979 lignes)

#include "O_State_DecisionMakerIA.hpp"
#include "Robot.hpp"
#include "log/Logger.hpp"
#include "OPOS6UL_RobotExtended.hpp"

O_State_DecisionMakerIA::O_State_DecisionMakerIA(Robot &robot) :
		robot_(robot)
{
}

void O_State_DecisionMakerIA::execute()
{
	OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
	Robot::logger().info() << "O_State_DecisionMakerIA::execute() - TODO strategie 2026" << logs::end;
}
