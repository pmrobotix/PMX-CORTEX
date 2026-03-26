/*!
 * \file
 * \brief Implémentation de la classe ServoDriverManualTest.
 */

#include "ServoDriverManualTest.hpp"


void test::ServoDriverManualTest::suite()
{
	this->firstTest();

}

void test::ServoDriverManualTest::firstTest()
{

	servodriver_->hold(0);

	this->assert(true, "OK");
}
