/*!
 * \file
 * \brief Test unitaire de la classe metier Sensors (simulation uniquement).
 *
 * Teste les setters/getters, les filtres de niveau (filtre_levelInFront/Back)
 * et la delegation vers le driver. Necessite un Robot + Asserv stub minimal.
 */

#ifndef TEST_SENSORSTEST_HPP
#define TEST_SENSORSTEST_HPP

#ifdef SIMU

#include "../suite/UnitTest.hpp"
#include "log/LoggerFactory.hpp"

namespace test {

/*!
 * \brief Test unitaire de la couche metier Sensors.
 */
class SensorsTest : public UnitTest {
private:

	static inline const logs::Logger& logger()
	{
		static const logs::Logger &instance = logs::LoggerFactory::logger("SensorsTest");
		return instance;
	}

public:
	SensorsTest() : UnitTest("SensorsTest") {}
	virtual ~SensorsTest() {}

	virtual void suite();

	// Cycle de vie
	void testConstructor();
	void testIsConnected();

	// Configuration
	void testAvailableFlagsFront();
	void testAvailableFlagsBack();
	void testSetIgnoreAll();

	// Delegation vers driver
	void testClearPositionsAdv();
	void testSyncInvalidName();

	// filtre_levelInFront : 4 niveaux + outside
	void testFiltreFront_Level1_RightClose();
	void testFiltreFront_Level2_LeftClose();
	void testFiltreFront_Level3_MidZone();
	void testFiltreFront_Level4_DeadFront();
	void testFiltreFront_Outside_Behind();

	// filtre_levelInBack : symetrique
	void testFiltreBack_Level4_DeadBehind();
	void testFiltreBack_Outside_InFront();

	// Cas limites
	void testFiltreFront_OverlapLevel1and4();
	void testFiltreFront_ZeroPosition();

private:
	void initRobot();
};

}

#endif // SIMU
#endif
