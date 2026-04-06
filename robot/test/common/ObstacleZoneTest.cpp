/*!
 * \file
 * \brief Implementation du test unitaire ObstacleZone.
 *
 * Tests de la logique pure de classification d'obstacles :
 * configuration, flags available/ignore, filtres de detection
 * multi-niveaux. Aucun driver requis.
 *
 * Migre depuis SensorsTest (ancien driver-test) vers common-test.
 */

#include "ObstacleZoneTest.hpp"

#include "../../src/common/geometry/ObstacleZone.hpp"

void test::ObstacleZoneTest::suite()
{
	testAvailableFlagsFront();
	testAvailableFlagsBack();
	testSetIgnoreAll();

	testFiltreFront_Level1_RightClose();
	testFiltreFront_Level2_LeftClose();
	testFiltreFront_Level3_MidZone();
	testFiltreFront_Level4_DeadFront();
	testFiltreFront_Outside_Behind();

	testFiltreBack_Level4_DeadBehind();
	testFiltreBack_Outside_InFront();

	testFiltreFront_OverlapLevel1and4();
	testFiltreFront_ZeroPosition();
}

// ---------- Configuration : flags available ----------

void test::ObstacleZoneTest::testAvailableFlagsFront()
{
	ObstacleZone oz;

	// enable=true, ignore=false => available true
	oz.addConfigFront(false, true, false);
	oz.setIgnoreFrontNearObstacle(false, false, false);
	this->assert(oz.getAvailableFrontCenter() == true,
			"enableFrontCenter=true, ignore=false => available true");

	// enable=true, ignore=true => available false
	oz.setIgnoreFrontNearObstacle(false, true, false);
	this->assert(oz.getAvailableFrontCenter() == false,
			"enable=true, ignore=true => available false");

	// enable=false => available false
	oz.addConfigFront(false, false, false);
	oz.setIgnoreFrontNearObstacle(false, false, false);
	this->assert(oz.getAvailableFrontCenter() == false,
			"enable=false => available false");
}

void test::ObstacleZoneTest::testAvailableFlagsBack()
{
	ObstacleZone oz;

	oz.addConfigBack(false, true, false);
	oz.setIgnoreBackNearObstacle(false, false, false);
	this->assert(oz.getAvailableBackCenter() == true,
			"enableBackCenter=true, ignore=false => available true");

	oz.setIgnoreBackNearObstacle(false, true, false);
	this->assert(oz.getAvailableBackCenter() == false,
			"enable=true, ignore=true => available false");
}

void test::ObstacleZoneTest::testSetIgnoreAll()
{
	ObstacleZone oz;

	oz.addConfigFront(true, true, true);
	oz.setIgnoreAllFrontNearObstacle(true);
	this->assert(oz.getAvailableFrontCenter() == false,
			"setIgnoreAllFront(true) => frontCenter unavailable");

	oz.setIgnoreAllFrontNearObstacle(false);
	this->assert(oz.getAvailableFrontCenter() == true,
			"setIgnoreAllFront(false) => frontCenter available");
}

// ---------- filtre_levelInFront : tests par niveau ----------
// Contexte : threshold_LR=200, threshold_Front=600, threshold_veryclosed=300
// Note : le filtre applique un "patch balise" de +/-50mm sur x et y.

void test::ObstacleZoneTest::testFiltreFront_Level1_RightClose()
{
	ObstacleZone oz;
	// Adversaire devant, tres pres, a droite : ydist=100, xdist=200
	// Apres patch : y=150, x=250
	// Level 1 : y<=300 && x>=200 && x<=600
	int level = oz.filtre_levelInFront(
			200, 600, 300,
			0.0f, 200.0f, 100.0f, 0.0f);
	this->assert(level == 1, "adv devant-droit proche => level 1");
}

void test::ObstacleZoneTest::testFiltreFront_Level2_LeftClose()
{
	ObstacleZone oz;
	// Adversaire devant, tres pres, a gauche : xdist=-200
	// Apres patch : x=-250, y=150
	// Level 2 : y<=300 && x<=-200 && x>=-600
	int level = oz.filtre_levelInFront(
			200, 600, 300,
			0.0f, -200.0f, 100.0f, 0.0f);
	this->assert(level == 2, "adv devant-gauche proche => level 2");
}

void test::ObstacleZoneTest::testFiltreFront_Level3_MidZone()
{
	ObstacleZone oz;
	// Adversaire devant, zone moyenne (entre very closed et front), centre
	// ydist=400 => apres patch y=450 (entre 300 et 600)
	int level = oz.filtre_levelInFront(
			200, 600, 300,
			0.0f, 0.0f, 400.0f, 0.0f);
	this->assert(level == 3, "adv zone moyenne centree => level 3");
}

void test::ObstacleZoneTest::testFiltreFront_Level4_DeadFront()
{
	ObstacleZone oz;
	// Adversaire pile devant, tres proche : ydist=100, xdist=0
	// Apres patch : y=150, x=0
	// Level 4 : y<=300 && x dans [-200, 200]
	int level = oz.filtre_levelInFront(
			200, 600, 300,
			0.0f, 0.0f, 100.0f, 0.0f);
	this->assert(level == 4, "adv pile devant tres proche => level 4");
}

void test::ObstacleZoneTest::testFiltreFront_Outside_Behind()
{
	ObstacleZone oz;
	// Adversaire derriere : ydist<0 => filtre front retourne 0
	int level = oz.filtre_levelInFront(
			200, 600, 300,
			0.0f, 0.0f, -200.0f, 0.0f);
	this->assert(level == 0, "adv derriere => filtre front retourne 0");
}

// ---------- filtre_levelInBack ----------

void test::ObstacleZoneTest::testFiltreBack_Level4_DeadBehind()
{
	ObstacleZone oz;
	// Adversaire pile derriere, tres proche : ydist=-100, xdist=0
	// Apres patch : y=-150, x=0
	// Level -4 : y>=-300 && x dans [-200, 200]
	int level = oz.filtre_levelInBack(
			200, 600, 300,
			0.0f, 0.0f, -100.0f, 0.0f);
	this->assert(level == -4, "adv pile derriere tres proche => level -4");
}

void test::ObstacleZoneTest::testFiltreBack_Outside_InFront()
{
	ObstacleZone oz;
	// Adversaire devant : ydist>0 => filtre back retourne 0
	int level = oz.filtre_levelInBack(
			200, 600, 300,
			0.0f, 0.0f, 200.0f, 0.0f);
	this->assert(level == 0, "adv devant => filtre back retourne 0");
}

// ---------- Cas limites ----------

void test::ObstacleZoneTest::testFiltreFront_OverlapLevel1and4()
{
	ObstacleZone oz;
	// A la frontiere threshold_LR, level 1 et level 4 sont tous les deux vrais.
	// L'ordre dans le code : level 1 teste en premier, level 1 gagne.
	// xdist=150 => apres patch x=200 == threshold_LR_mm
	int level = oz.filtre_levelInFront(
			200, 600, 300,
			0.0f, 150.0f, 100.0f, 0.0f);
	this->assert(level == 1,
			"frontiere x=threshold_LR : level 1 (prioritaire) gagne");
}

void test::ObstacleZoneTest::testFiltreFront_ZeroPosition()
{
	ObstacleZone oz;
	// xdist=0, ydist=0 : patch balise ne s'applique pas (test strict sur > et <)
	// Donc ydist=0 => la condition ydist>0 est fausse => retourne 0
	int level = oz.filtre_levelInFront(
			200, 600, 300,
			0.0f, 0.0f, 0.0f, 0.0f);
	this->assert(level == 0,
			"adv en (0,0) : y>0 faux => retourne 0");
}
