/*!
 * \file
 * \brief Point d'entree des tests manuels (verification visuelle/physique).
 *
 * Ces tests necessitent un operateur humain pour valider le resultat
 * (LEDs, servos, sons, etc.). A executer sur le robot en atelier.
 */

#include "../../src/common/thread/Thread.hpp"
#include "../suite/UnitTestSuite.hpp"

#include "LedManualTest.hpp"
#include "SwitchManualTest.hpp"
#include "ActionManagerTimerManualTest.hpp"
#include "AsservDriverManualTest.hpp"
#include "ButtonDriverManualTest.hpp"
#include "LcdShieldDriverManualTest.hpp"
#include "ServoDriverManualTest.hpp"
#include "ColorDriverManualTest.hpp"
#include "SensorDriverManualTest.hpp"

int main(int, char**)
{
	utils::set_realtime_priority(3, "manual-test");

	UnitTestSuite suite;

	suite.addTest(new test::LedManualTest());
	suite.addTest(new test::SwitchManualTest());
	suite.addTest(new test::ActionManagerTimerManualTest());
	suite.addTest(new test::AsservDriverManualTest());
	suite.addTest(new test::LcdShieldDriverManualTest());
	suite.addTest(new test::ServoDriverManualTest());
	suite.addTest(new test::ColorDriverManualTest());
	suite.addTest(new test::SensorDriverManualTest());
	suite.addTest(new test::ButtonDriverManualTest());

	suite.run();

	return 0;
}
