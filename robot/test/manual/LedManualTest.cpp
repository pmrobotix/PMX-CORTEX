/*!
 * \file
 * \brief Implementation du test manuel visuel des LEDs.
 */

#include "LedManualTest.hpp"

#include <unistd.h>

void test::LedManualTest::suite()
{
	testBlinkEach();
	testAlternatePattern();
}

void test::LedManualTest::testBlinkEach()
{
	logger().info() << "testBlinkEach : allume/eteint chaque LED (verification visuelle)" << logs::end;

	for (int i = 0; i < 8; i++) {
		for (int n = 0; n < 2; n++) {
			leddriver->setBit(i, LED_GREEN);
			usleep(50000);
			leddriver->setBit(i, LED_OFF);
			usleep(50000);
		}
	}
}

void test::LedManualTest::testAlternatePattern()
{
	logger().info() << "testAlternatePattern : patterns alternants 0xAA/0x55 (verification visuelle)" << logs::end;

	for (int n = 0; n < 4; n++) {
		leddriver->setBytes(0xAA, LED_GREEN);
		usleep(100000);
		leddriver->setBytes(0x55, LED_GREEN);
		usleep(100000);
	}
	leddriver->setBytes(0x00, LED_OFF);
}
