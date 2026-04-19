/*!
 * \file
 * \brief Test unitaire du driver de capteurs via l'interface ASensorsDriver.
 *
 * Verifie le contrat de l'interface : factory, connexion, sync,
 * positions adversaires et capteurs de proximite.
 * En SIMU, teste SensorsDriverSimu.
 */

#ifndef TEST_SENSORSDRIVERTEST_HPP
#define TEST_SENSORSDRIVERTEST_HPP

#include "../suite/UnitTest.hpp"
#include "interface/ASensorsDriver.hpp"
#include "interface/ARobotPositionShared.hpp"
#include "log/LoggerFactory.hpp"

namespace test {

/*!
 * \brief Test unitaire fonctionnel du driver de capteurs.
 */
class SensorsDriverTest : public UnitTest
{
private:

	static inline const logs::Logger & logger()
	{
		static const logs::Logger & instance = logs::LoggerFactory::logger("SensorsDriverTest");
		return instance;
	}

	ARobotPositionShared* sharedPos_;

public:

	ASensorsDriver* sensorsdriver;

	SensorsDriverTest() : UnitTest("SensorsDriverTest")
	{
		sharedPos_ = ARobotPositionShared::create();
		sensorsdriver = ASensorsDriver::create("SensorsDriverTest", sharedPos_);
	}

	virtual ~SensorsDriverTest()
	{
		delete sensorsdriver;
		delete sharedPos_;
	}

	virtual void suite();

	/*!
	 * \brief Verifie que create() retourne une instance valide.
	 */
	void testCreate();

	/*!
	 * \brief Verifie que is_connected() retourne true en simu.
	 */
	void testIsConnected();

	/*!
	 * \brief Verifie que sync() ne retourne pas d'erreur en simu.
	 */
	void testSyncReturnsZero();

	/*!
	 * \brief Verifie que clearPositionsAdv() vide le vecteur.
	 */
	void testClearPositions();

	/*!
	 * \brief Verifie que addvPositionsAdv() ajoute une entree.
	 */
	void testAddPosition();

	/*!
	 * \brief Verifie les valeurs par defaut des capteurs avant (999 en simu).
	 */
	void testFrontSensorsDefaults();

	/*!
	 * \brief Verifie les valeurs par defaut des capteurs arriere (999 en simu).
	 */
	void testBackSensorsDefaults();

	/*!
	 * \brief Verifie les valeurs par defaut des capteurs lateraux (400 en simu).
	 */
	void testSideSensors();

	/*!
	 * \brief Verifie que displayNumber() ne crash pas.
	 */
	void testDisplayNumberNoCrash();

	/*!
	 * \brief Verifie que getBeaconSeq() retourne 0 (pas de beacon en test).
	 */
	void testBeaconSeqDefault();

	// Tests de l'injection persistante (setInjectedAdv / clearInjectedAdv)
	void testInjectionPersistsAcrossSync();
	void testInjectionOverride();
	void testClearInjectedAdv();
};

}

#endif
