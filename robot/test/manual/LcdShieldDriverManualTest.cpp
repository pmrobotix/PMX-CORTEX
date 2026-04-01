/*!
 * \file
 * \brief Implémentation de la classe LcdShieldDriverManualTest.
 */

#include "LcdShieldDriverManualTest.hpp"

#include <unistd.h>
#include <cstdint>

void test::LcdShieldDriverManualTest::suite()
{
	this->test();

}

void test::LcdShieldDriverManualTest::test()
{

	lcdshielddriver->setBacklightOn();
	lcdshielddriver->clear();
	lcdshielddriver->home();
	lcdshielddriver->print_content_string("PMX", 0, 0);
	sleep(2);
	lcdshielddriver->clear();
	lcdshielddriver->setBacklightOff();

	this->assert(true, "OK");
}
