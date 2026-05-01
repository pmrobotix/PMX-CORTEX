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

	// Chaine complete balise -> filtre : valide la convention canonique
	// (x=avant, y=lateral gauche>0) en partant des grandeurs balise (dist, angle).
	void testChaineBalise_AdvDevant();
	void testChaineBalise_AdvDroite();
	void testChaineBalise_AdvGauche();
	void testChaineBalise_AdvArriere();
	void testChaineBalise_AdvDroitePur();        // 90 deg vers droite (lateral pur)
	void testChaineBalise_AdvGauchePur();        // -90 deg vers gauche (lateral pur)
	void testChaineBalise_AdvArriereDroit();     // 180+45 deg arriere-droit
	void testChaineBalise_AdvArriereGauche();    // 180-45 deg arriere-gauche

	// Frontieres / zones exclues (no-mix front <-> back, pas de trou logique)
	void testFiltreFront_TooLateralRight();
	void testFiltreFront_TooLateralLeft();
	void testFiltreFront_TooDeep();
	void testFiltreBack_TooDeep();
	void testFiltreBack_DeadBehind_LimitLR();
	void testFiltreBoth_AdvAtX0_Lateral();       // x=0 -> ni front ni back
};

}

#endif
