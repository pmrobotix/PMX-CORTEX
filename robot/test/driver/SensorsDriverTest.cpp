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

	testInjectionPersistsAcrossSync();
	testInjectionOverride();
	testClearInjectedAdv();
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
	// En SIMU, sync() retourne >=0 (pas d'erreur). La valeur > 0 signifie
	// "nouvelle frame beacon disponible" et declenche le filtrage dans
	// SensorsThread::sensorOnTimer().
	this->assert(ret >= 0, "sync() retourne >=0 (pas d'erreur) en simu");
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

// ========== Tests injection persistante (SIMU uniquement) ==========

void test::SensorsDriverTest::testInjectionPersistsAcrossSync()
{
	// setInjectedAdv(x, y) : la position doit etre republiee par vadv_ a chaque sync()
	// (contrairement a addvPositionsAdv qui disparait au sync suivant).
	sensorsdriver->clearInjectedAdv();
	sensorsdriver->clearPositionsAdv();

	sensorsdriver->setInjectedAdv(800.0f, 500.0f);

	for (int i = 0; i < 5; i++) {
		sensorsdriver->sync();
		ASensorsDriver::bot_positions pos = sensorsdriver->getvPositionsAdv();
		this->assert(pos.size() == 1, "injection persiste apres sync() (iteration i)");
	}

	sensorsdriver->clearInjectedAdv();
}

void test::SensorsDriverTest::testInjectionOverride()
{
	// Repositionner l'adv injecte : toujours 1 seule entree, la derniere.
	sensorsdriver->clearInjectedAdv();

	sensorsdriver->setInjectedAdv(500.0f, 500.0f);
	sensorsdriver->sync();
	this->assert(sensorsdriver->getvPositionsAdv().size() == 1,
			"apres premier setInjectedAdv + sync, 1 adv");

	sensorsdriver->setInjectedAdv(1200.0f, 800.0f);
	sensorsdriver->sync();
	this->assert(sensorsdriver->getvPositionsAdv().size() == 1,
			"apres override setInjectedAdv + sync, toujours 1 adv");

	sensorsdriver->clearInjectedAdv();
}

void test::SensorsDriverTest::testClearInjectedAdv()
{
	// clearInjectedAdv() : plus de republication au sync suivant.
	sensorsdriver->setInjectedAdv(500.0f, 500.0f);
	sensorsdriver->sync();
	this->assert(sensorsdriver->getvPositionsAdv().size() == 1,
			"injection visible apres sync");

	sensorsdriver->clearInjectedAdv();
	sensorsdriver->sync();
	this->assert(sensorsdriver->getvPositionsAdv().empty(),
			"apres clearInjectedAdv + sync, positions vides");
}
