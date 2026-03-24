/*!
 * \file
 * \brief Implementation du test unitaire du driver de switchs.
 *
 * Tests fonctionnels avec assertions reelles.
 * Le test manuel de la tirette est dans test/manual/SwitchManualTest.
 */

#include "SwitchDriverTest.hpp"

void test::SwitchDriverTest::suite()
{
	testConnected();
	testDefaultValues();
}

void test::SwitchDriverTest::testConnected()
{
	bool connected = switchdriver->is_connected();
	this->assert(connected, "is_connected() retourne true");
}

void test::SwitchDriverTest::testDefaultValues()
{
	// En SIMU : tirette retourne toujours 0 (retiree), switchs retournent 0
	// En ARM sans hardware : retourne -1 (non connecte) ou la valeur reelle
	int tirette = switchdriver->tirettePressed();
	int backLeft = switchdriver->backLeftPressed();
	int backRight = switchdriver->backRightPressed();

	logger().debug() << "tirette=" << tirette
			<< " backLeft=" << backLeft
			<< " backRight=" << backRight << logs::end;

	// Les valeurs doivent etre 0 ou 1 (pas de valeur aberrante)
	this->assert(tirette >= 0 && tirette <= 1, "tirettePressed() retourne 0 ou 1");
	this->assert(backLeft >= 0 && backLeft <= 1, "backLeftPressed() retourne 0 ou 1");
	this->assert(backRight >= 0 && backRight <= 1, "backRightPressed() retourne 0 ou 1");
}
