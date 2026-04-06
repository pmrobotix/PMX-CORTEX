/*!
 * \file
 * \brief Implementation du test unitaire du driver de capteurs.
 *
 * Tests fonctionnels du contrat ASensorsDriver via l'implementation simu.
 */

#include "SensorsDriverTest.hpp"

void test::SensorsDriverTest::suite()
{
	testCreate();
	testIsConnected();
	testSyncReturnsZero();
	testClearPositions();
	testAddPosition();
	testFrontSensorsDefaults();
	testBackSensorsDefaults();
	testSideSensors();
	testDisplayNumberNoCrash();
	testBeaconSeqDefault();
}

void test::SensorsDriverTest::testCreate()
{
	this->assert(sensorsdriver != nullptr, "create() retourne une instance non nulle");
}

void test::SensorsDriverTest::testIsConnected()
{
	// En SIMU : toujours connecte
	this->assert(sensorsdriver->is_connected(), "is_connected() retourne true en simu");
}

void test::SensorsDriverTest::testSyncReturnsZero()
{
	int ret = sensorsdriver->sync();
	this->assert(ret == 0, "sync() retourne 0 (pas d'erreur) en simu");
}

void test::SensorsDriverTest::testClearPositions()
{
	sensorsdriver->clearPositionsAdv();
	ASensorsDriver::bot_positions positions = sensorsdriver->getvPositionsAdv();
	this->assert(positions.empty(), "apres clearPositionsAdv(), getvPositionsAdv() est vide");
}

void test::SensorsDriverTest::testAddPosition()
{
	sensorsdriver->clearPositionsAdv();
	sensorsdriver->addvPositionsAdv(200.0f, 1000.0f);

	ASensorsDriver::bot_positions positions = sensorsdriver->getvPositionsAdv();
	this->assert(positions.size() == 1, "apres add, size == 1");
	if (positions.size() == 1) {
		this->assert(positions[0].nbDetectedBots == 1, "nbDetectedBots == 1");
		this->assert(positions[0].t_us == 0, "t_us == 0 en simu (pas de beacon)");
	}

	// Nettoyage pour les tests suivants
	sensorsdriver->clearPositionsAdv();
}

void test::SensorsDriverTest::testFrontSensorsDefaults()
{
#ifdef SIMU
	// En SIMU : tous les capteurs avant retournent 999 (rien detecte)
	this->assert(sensorsdriver->frontLeft() == 999, "frontLeft() == 999 en simu");
	this->assert(sensorsdriver->frontCenter() == 999, "frontCenter() == 999 en simu");
	this->assert(sensorsdriver->frontRight() == 999, "frontRight() == 999 en simu");
#else
	// En ARM : les valeurs reelles dependent du hardware, on verifie juste > 0
	this->assert(sensorsdriver->frontLeft() > 0, "frontLeft() > 0 en arm");
	this->assert(sensorsdriver->frontRight() > 0, "frontRight() > 0 en arm");
#endif
}

void test::SensorsDriverTest::testBackSensorsDefaults()
{
#ifdef SIMU
	this->assert(sensorsdriver->backLeft() == 999, "backLeft() == 999 en simu");
	this->assert(sensorsdriver->backCenter() == 999, "backCenter() == 999 en simu");
	this->assert(sensorsdriver->backRight() == 999, "backRight() == 999 en simu");
#else
	// En ARM : stub retourne -1, on verifie que ca ne crash pas
	sensorsdriver->backLeft();
	sensorsdriver->backCenter();
	sensorsdriver->backRight();
	this->assert(true, "backLeft/Center/Right() ne crash pas en arm");
#endif
}

void test::SensorsDriverTest::testSideSensors()
{
	this->assert(sensorsdriver->rightSide() == 400, "rightSide() == 400");
	this->assert(sensorsdriver->leftSide() == 400, "leftSide() == 400");
}

void test::SensorsDriverTest::testDisplayNumberNoCrash()
{
	sensorsdriver->displayNumber(42);
	this->assert(true, "displayNumber(42) sans crash");
}

void test::SensorsDriverTest::testBeaconSeqDefault()
{
	this->assert(sensorsdriver->getBeaconSeq() == 0, "getBeaconSeq() == 0 (pas de beacon)");
}
