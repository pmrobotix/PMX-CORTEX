/*!
 * \file
 * \brief Implémentation de la classe SensorDriverManualTest.
 */

#include "SensorDriverManualTest.hpp"

#include <unistd.h>
#include <cstdint>

void test::SensorDriverManualTest::suite()
{
	this->firstTest();

}

void test::SensorDriverManualTest::firstTest()
{

	sensordriver->frontLeft();

	this->assert(true, "OK");
}
