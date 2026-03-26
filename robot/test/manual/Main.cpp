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

int main(int, char**)
{
	utils::set_realtime_priority(3, "manual-test");

	UnitTestSuite suite;

	suite.addTest(new test::LedManualTest());
	suite.addTest(new test::SwitchManualTest());
	suite.addTest(new test::ActionManagerTimerManualTest());

	suite.run();

	return 0;
}
