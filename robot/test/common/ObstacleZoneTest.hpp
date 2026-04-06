/*!
 * \file
 * \brief Test unitaire de la classe ObstacleZone.
 *
 * Teste la configuration capteurs (enable/ignore/available),
 * les seuils de distance, et les filtres de classification
 * (filtre_levelInFront/Back). Logique pure, aucun driver requis.
 */

#ifndef TEST_OBSTACLEZONETEST_HPP
#define TEST_OBSTACLEZONETEST_HPP

#include "../suite/UnitTest.hpp"
#include "log/LoggerFactory.hpp"

namespace test {

/*!
 * \brief Test unitaire de la classification d'obstacles par zones.
 */
class ObstacleZoneTest : public UnitTest
{
private:

	static inline const logs::Logger & logger()
	{
		static const logs::Logger & instance = logs::LoggerFactory::logger("ObstacleZoneTest");
		return instance;
	}

public:

	ObstacleZoneTest() : UnitTest("ObstacleZoneTest") {}
	virtual ~ObstacleZoneTest() {}

	virtual void suite();

	// Configuration : flags available
	void testAvailableFlagsFront();
	void testAvailableFlagsBack();
	void testSetIgnoreAll();

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
};

}

#endif
