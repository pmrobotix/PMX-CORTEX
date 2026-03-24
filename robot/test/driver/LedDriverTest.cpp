/*!
 * \file
 * \brief Implementation du test unitaire du driver de LEDs.
 *
 * Verifie le comportement via l'interface ALedDriver (getBit).
 * Les tests visuels (clignotement) sont dans test/manual/LedManualTest.
 */

#include "LedDriverTest.hpp"

void test::LedDriverTest::suite()
{
	testCreate();
	testSetBit();
	testSetBytes();
	testReset();
}

void test::LedDriverTest::testCreate()
{
	this->assert(leddriver != nullptr, "create() retourne une instance non nulle");
}

void test::LedDriverTest::testSetBit()
{
	// RAZ
	leddriver->setBytes(0x00, LED_OFF);

	// Allume la LED 3 en vert
	leddriver->setBit(3, LED_GREEN);

	if (leddriver->getBit(0) != -1) {
		// Plateforme avec lecture (SIMU)
		this->assert(leddriver->getBit(3) == LED_GREEN, "setBit(3, GREEN) => getBit(3) == GREEN");
		this->assert(leddriver->getBit(0) == 0, "setBit(3) ne touche pas index 0");
		this->assert(leddriver->getBit(7) == 0, "setBit(3) ne touche pas index 7");

		// Eteint la LED 3
		leddriver->setBit(3, LED_OFF);
		this->assert(leddriver->getBit(3) == LED_OFF, "setBit(3, OFF) => getBit(3) == OFF");
	} else {
		// ARM : pas de lecture, on verifie juste que ca ne crash pas
		leddriver->setBit(3, LED_OFF);
		this->assert(true, "setBit ARM sans crash");
	}
}

void test::LedDriverTest::testSetBytes()
{
	// Pattern 0xAA = 10101010 : LEDs 1,3,5,7 allumees
	leddriver->setBytes(0xAA, LED_RED);

	if (leddriver->getBit(0) != -1) {
		this->assert(leddriver->getBit(0) == 0, "setBytes(0xAA) => index 0 eteinte");
		this->assert(leddriver->getBit(1) == LED_RED, "setBytes(0xAA) => index 1 RED");
		this->assert(leddriver->getBit(2) == 0, "setBytes(0xAA) => index 2 eteinte");
		this->assert(leddriver->getBit(3) == LED_RED, "setBytes(0xAA) => index 3 RED");
		this->assert(leddriver->getBit(7) == LED_RED, "setBytes(0xAA) => index 7 RED");

		// Pattern 0x55 = 01010101 : LEDs 0,2,4,6 allumees
		leddriver->setBytes(0x55, LED_ORANGE);
		this->assert(leddriver->getBit(0) == LED_ORANGE, "setBytes(0x55) => index 0 ORANGE");
		this->assert(leddriver->getBit(1) == 0, "setBytes(0x55) => index 1 eteinte");
		this->assert(leddriver->getBit(4) == LED_ORANGE, "setBytes(0x55) => index 4 ORANGE");
	} else {
		this->assert(true, "setBytes ARM sans crash");
	}
}

void test::LedDriverTest::testReset()
{
	leddriver->setBytes(0xFF, LED_GREEN);
	leddriver->setBytes(0x00, LED_OFF);

	if (leddriver->getBit(0) != -1) {
		for (int i = 0; i < 8; i++) {
			this->assert(leddriver->getBit(i) == 0, "apres reset 0x00, index eteinte");
		}
	} else {
		this->assert(true, "reset ARM sans crash");
	}
}
