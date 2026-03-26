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
	print("PMX");
	sleep(2);
	lcdshielddriver->clear();
	lcdshielddriver->setBacklightOff();

	this->assert(true, "OK");
}

size_t test::LcdShieldDriverManualTest::print(const std::string &s)
{
	size_t n = 0;
	for (size_t i = 0; i < s.length(); i++)
	{
		n += lcdshielddriver->write(s[i]);
	}
	return n;
}
