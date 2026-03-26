/*!
 * \file
 * \brief Implémentation de la classe ColorDriverManualTest.
 */

#include "ColorDriverManualTest.hpp"

#include <unistd.h>

#include "log/Logger.hpp"

void test::ColorDriverManualTest::suite()
{
	this->firstTest();

}

void test::ColorDriverManualTest::firstTest()
{

	for (int i = 0; i < 10; i++)
	{
		bool connected = colordriver->readRGB();
		if (connected)
		{
			//logger().info() << "x: " << colordriver->getTX() << " \ty: " << colordriver->getTY()
			//	<< logs::end;
			printf(" %f %f\n", colordriver->getTX(), colordriver->getTY());
			//usleep(100000);
		}
	}

	this->assert(true, "OK");
}
