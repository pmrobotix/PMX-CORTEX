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

#include <cmath>

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

	testChaineBalise_AdvDevant();
	testChaineBalise_AdvDroite();
	testChaineBalise_AdvGauche();
	testChaineBalise_AdvArriere();
	testChaineBalise_AdvDroitePur();
	testChaineBalise_AdvGauchePur();
	testChaineBalise_AdvArriereDroit();
	testChaineBalise_AdvArriereGauche();

	testFiltreFront_TooLateralRight();
	testFiltreFront_TooLateralLeft();
	testFiltreFront_TooDeep();
	testFiltreBack_TooDeep();
	testFiltreBack_DeadBehind_LimitLR();
	testFiltreBoth_AdvAtX0_Lateral();
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
// Convention canonique repere robot : x=avant (>0), y=lateral gauche>0 / droite<0.
// Contexte : threshold_LR=200, threshold_Front=600, threshold_veryclosed=300
// Note : le filtre applique un "patch balise" de +/-50mm sur x et y.

void test::ObstacleZoneTest::testFiltreFront_Level1_RightClose()
{
	ObstacleZone oz;
	// Adversaire devant, tres pres, a droite : x=100 (avant), y=-200 (droite)
	// Apres patch : x=150, y=-250
	// Level 1 : x<=300 && y<=-200 && y>=-600
	int level = oz.filtre_levelInFront(
			200, 600, 300,
			0.0f, 100.0f, -200.0f, 0.0f);
	this->assert(level == 1, "adv devant-droit proche => level 1");
}

void test::ObstacleZoneTest::testFiltreFront_Level2_LeftClose()
{
	ObstacleZone oz;
	// Adversaire devant, tres pres, a gauche : x=100 (avant), y=200 (gauche)
	// Apres patch : x=150, y=250
	// Level 2 : x<=300 && y>=200 && y<=600
	int level = oz.filtre_levelInFront(
			200, 600, 300,
			0.0f, 100.0f, 200.0f, 0.0f);
	this->assert(level == 2, "adv devant-gauche proche => level 2");
}

void test::ObstacleZoneTest::testFiltreFront_Level3_MidZone()
{
	ObstacleZone oz;
	// Adversaire devant, zone moyenne (entre very closed et front), centre
	// x=400 => apres patch x=450 (entre 300 et 600), y=0 (centre)
	int level = oz.filtre_levelInFront(
			200, 600, 300,
			0.0f, 400.0f, 0.0f, 0.0f);
	this->assert(level == 3, "adv zone moyenne centree => level 3");
}

void test::ObstacleZoneTest::testFiltreFront_Level4_DeadFront()
{
	ObstacleZone oz;
	// Adversaire pile devant, tres proche : x=100 (avant), y=0 (centre)
	// Apres patch : x=150, y=0
	// Level 4 : x<=300 && y dans [-200, 200]
	int level = oz.filtre_levelInFront(
			200, 600, 300,
			0.0f, 100.0f, 0.0f, 0.0f);
	this->assert(level == 4, "adv pile devant tres proche => level 4");
}

void test::ObstacleZoneTest::testFiltreFront_Outside_Behind()
{
	ObstacleZone oz;
	// Adversaire derriere : x<0 => filtre front retourne 0
	int level = oz.filtre_levelInFront(
			200, 600, 300,
			0.0f, -200.0f, 0.0f, 0.0f);
	this->assert(level == 0, "adv derriere => filtre front retourne 0");
}

// ---------- filtre_levelInBack ----------

void test::ObstacleZoneTest::testFiltreBack_Level4_DeadBehind()
{
	ObstacleZone oz;
	// Adversaire pile derriere, tres proche : x=-100 (derriere), y=0
	// Apres patch : x=-150, y=0
	// Level -4 : x>=-300 && y dans [-200, 200]
	int level = oz.filtre_levelInBack(
			200, 600, 300,
			0.0f, -100.0f, 0.0f, 0.0f);
	this->assert(level == -4, "adv pile derriere tres proche => level -4");
}

void test::ObstacleZoneTest::testFiltreBack_Outside_InFront()
{
	ObstacleZone oz;
	// Adversaire devant : x>0 => filtre back retourne 0
	int level = oz.filtre_levelInBack(
			200, 600, 300,
			0.0f, 200.0f, 0.0f, 0.0f);
	this->assert(level == 0, "adv devant => filtre back retourne 0");
}

// ---------- Cas limites ----------

void test::ObstacleZoneTest::testFiltreFront_OverlapLevel1and4()
{
	ObstacleZone oz;
	// A la frontiere threshold_LR, level 1 et level 4 sont tous les deux vrais.
	// L'ordre dans le code : level 1 teste en premier, level 1 gagne.
	// y=-150 => apres patch y=-200 == -threshold_LR_mm (limite droite)
	int level = oz.filtre_levelInFront(
			200, 600, 300,
			0.0f, 100.0f, -150.0f, 0.0f);
	this->assert(level == 1,
			"frontiere y=-threshold_LR : level 1 (prioritaire) gagne");
}

void test::ObstacleZoneTest::testFiltreFront_ZeroPosition()
{
	ObstacleZone oz;
	// x=0, y=0 : patch balise ne s'applique pas (test strict sur > et <)
	// Donc x=0 => la condition x>0 est fausse => retourne 0
	int level = oz.filtre_levelInFront(
			200, 600, 300,
			0.0f, 0.0f, 0.0f, 0.0f);
	this->assert(level == 0,
			"adv en (0,0) : x>0 faux => retourne 0");
}

// ---------- Chaine complete balise -> filtre ----------
// Simule la convention de la balise Teensy :
//   x = d * cos(theta_deg),  y = d * sin(theta_deg)
//   theta_deg : 0 = devant, +deg = gauche (sens trigo CCW)
// Puis appelle directement le filtre (qui interprete x=avant, y=gauche>0).
// Valide la coherence balise -> filtre, sans transformation intermediaire.
//
// Thresholds : LR=200, Front=600, veryClosed=300.

void test::ObstacleZoneTest::testChaineBalise_AdvDevant()
{
	// Balise : adv pile devant a 200mm (theta_deg=0, d=200)
	// -> x = 200, y = 0
	// Apres patch : x=250, y=0
	// Level 4 : x<=300 && y dans [-200, 200]
	ObstacleZone oz;
	const float d = 200.0f;
	const float theta_deg = 0.0f;
	const float theta_rad = theta_deg * (float)M_PI / 180.0f;
	const float x = d * std::cos(theta_rad);
	const float y = d * std::sin(theta_rad);
	int level = oz.filtre_levelInFront(200, 600, 300, d, x, y, theta_deg);
	this->assert(level == 4,
			"chaine balise : adv pile devant 200mm => level 4 (dead front)");
}

void test::ObstacleZoneTest::testChaineBalise_AdvDroite()
{
	// Balise : adv devant-droit a 400mm, angle = -60 deg (= droite)
	// -> x = 400 * cos(-60) = 200, y = 400 * sin(-60) = -346
	// Apres patch : x=250, y=-396
	// Level 1 : x<=300 && y<=-200 && y>=-600 OK
	ObstacleZone oz;
	const float d = 400.0f;
	const float theta_deg = -60.0f;
	const float theta_rad = theta_deg * (float)M_PI / 180.0f;
	const float x = d * std::cos(theta_rad);
	const float y = d * std::sin(theta_rad);
	int level = oz.filtre_levelInFront(200, 600, 300, d, x, y, theta_deg);
	this->assert(level == 1,
			"chaine balise : adv devant-droit 400mm/-60deg => level 1 (droite)");
}

void test::ObstacleZoneTest::testChaineBalise_AdvGauche()
{
	// Balise : adv devant-gauche a 400mm, angle = +60 deg (= gauche)
	// -> x = 400 * cos(60) = 200, y = 400 * sin(60) = +346
	// Apres patch : x=250, y=+396
	// Level 2 : x<=300 && y>=200 && y<=600 OK
	ObstacleZone oz;
	const float d = 400.0f;
	const float theta_deg = +60.0f;
	const float theta_rad = theta_deg * (float)M_PI / 180.0f;
	const float x = d * std::cos(theta_rad);
	const float y = d * std::sin(theta_rad);
	int level = oz.filtre_levelInFront(200, 600, 300, d, x, y, theta_deg);
	this->assert(level == 2,
			"chaine balise : adv devant-gauche 400mm/+60deg => level 2 (gauche)");
}

void test::ObstacleZoneTest::testChaineBalise_AdvArriere()
{
	// Balise : adv pile derriere a 200mm, angle = 180 deg
	// -> x = 200 * cos(180) = -200, y = 200 * sin(180) ~ 0
	// Apres patch : x=-250, y=0
	// filtre front : x<0 => retourne 0 (pas devant)
	// filtre back  : x<0 OK, x>=-300 OK, y dans [-200,200] => level -4
	ObstacleZone oz;
	const float d = 200.0f;
	const float theta_deg = 180.0f;
	const float theta_rad = theta_deg * (float)M_PI / 180.0f;
	const float x = d * std::cos(theta_rad);
	const float y = d * std::sin(theta_rad);
	int levelF = oz.filtre_levelInFront(200, 600, 300, d, x, y, theta_deg);
	int levelB = oz.filtre_levelInBack (200, 600, 300, d, x, y, theta_deg);
	this->assert(levelF == 0,
			"chaine balise : adv arriere => filtre front retourne 0");
	this->assert(levelB == -4,
			"chaine balise : adv arriere 200mm => filtre back level -4");
}

void test::ObstacleZoneTest::testChaineBalise_AdvDroitePur()
{
	// Balise : adv pile a droite (lateral pur), angle = -90 deg, d=400
	// -> x = 400 * cos(-90) = 0, y = 400 * sin(-90) = -400
	// Apres patch : x=0, y=-450
	// filtre front : x>0 faux => retourne 0
	// filtre back  : x<0 faux => retourne 0
	// (l'adv n'est ni devant ni derriere, on ne reagit pas - c'est voulu)
	ObstacleZone oz;
	const float d = 400.0f;
	const float theta_deg = -90.0f;
	const float theta_rad = theta_deg * (float)M_PI / 180.0f;
	const float x = d * std::cos(theta_rad);
	const float y = d * std::sin(theta_rad);
	int levelF = oz.filtre_levelInFront(200, 600, 300, d, x, y, theta_deg);
	int levelB = oz.filtre_levelInBack (200, 600, 300, d, x, y, theta_deg);
	this->assert(levelF == 0, "chaine balise : adv droite pur (-90deg) => front=0");
	this->assert(levelB == 0, "chaine balise : adv droite pur (-90deg) => back=0");
}

void test::ObstacleZoneTest::testChaineBalise_AdvGauchePur()
{
	// Balise : adv pile a gauche (lateral pur), angle = +90 deg, d=400
	// -> x=0, y=+400. Apres patch x=0, y=+450 -> ni front ni back.
	ObstacleZone oz;
	const float d = 400.0f;
	const float theta_deg = +90.0f;
	const float theta_rad = theta_deg * (float)M_PI / 180.0f;
	const float x = d * std::cos(theta_rad);
	const float y = d * std::sin(theta_rad);
	int levelF = oz.filtre_levelInFront(200, 600, 300, d, x, y, theta_deg);
	int levelB = oz.filtre_levelInBack (200, 600, 300, d, x, y, theta_deg);
	this->assert(levelF == 0, "chaine balise : adv gauche pur (+90deg) => front=0");
	this->assert(levelB == 0, "chaine balise : adv gauche pur (+90deg) => back=0");
}

void test::ObstacleZoneTest::testChaineBalise_AdvArriereDroit()
{
	// Balise : adv arriere-droit, angle = -135 deg, d=400
	// -> x = 400*cos(-135) = -283, y = 400*sin(-135) = -283
	// Apres patch : x=-333, y=-333
	// filtre back  : x<0 OK, x>=-300 faux (-333 < -300) => pas level -1/-2/-4
	//                level -3 : -600<=x<-300 OK (-333), -600<=y<=600 OK => -3
	ObstacleZone oz;
	const float d = 400.0f;
	const float theta_deg = -135.0f;
	const float theta_rad = theta_deg * (float)M_PI / 180.0f;
	const float x = d * std::cos(theta_rad);
	const float y = d * std::sin(theta_rad);
	int levelB = oz.filtre_levelInBack (200, 600, 300, d, x, y, theta_deg);
	this->assert(levelB == -3,
			"chaine balise : adv arriere-droit 400mm/-135deg => level -3 (zone moyenne)");
}

void test::ObstacleZoneTest::testChaineBalise_AdvArriereGauche()
{
	// Symetrique : angle = +135 deg, d=400
	// -> x = -283, y = +283. Apres patch x=-333, y=+333
	// level -3 : x<-300 OK, |y|<=600 OK => -3
	ObstacleZone oz;
	const float d = 400.0f;
	const float theta_deg = +135.0f;
	const float theta_rad = theta_deg * (float)M_PI / 180.0f;
	const float x = d * std::cos(theta_rad);
	const float y = d * std::sin(theta_rad);
	int levelB = oz.filtre_levelInBack (200, 600, 300, d, x, y, theta_deg);
	this->assert(levelB == -3,
			"chaine balise : adv arriere-gauche 400mm/+135deg => level -3");
}

// ---------- Frontieres et zones exclues (anti-mix front/back) ----------

void test::ObstacleZoneTest::testFiltreFront_TooLateralRight()
{
	// Adv devant proche mais TRES a droite (au-dela de threshold_Front lateral)
	// x=100 (avant), y=-700 (droite, hors zone)
	// Apres patch : x=150, y=-750
	// Level 1 : y>=-600 faux (-750<-600) => pas level 1
	// Level 4 : y>=-200 faux => pas level 4
	// => return 0 (adv visible mais hors zone d'interet)
	ObstacleZone oz;
	int level = oz.filtre_levelInFront(200, 600, 300,
			0.0f, 100.0f, -700.0f, 0.0f);
	this->assert(level == 0,
			"adv devant mais trop a droite (|y|>600) => return 0");
}

void test::ObstacleZoneTest::testFiltreFront_TooLateralLeft()
{
	// Symetrique gauche : x=100, y=+700
	ObstacleZone oz;
	int level = oz.filtre_levelInFront(200, 600, 300,
			0.0f, 100.0f, +700.0f, 0.0f);
	this->assert(level == 0,
			"adv devant mais trop a gauche (|y|>600) => return 0");
}

void test::ObstacleZoneTest::testFiltreFront_TooDeep()
{
	// Adv tres loin devant : x=700, y=0 (au-dela de threshold_Front en avant)
	// Apres patch : x=750, y=0
	// Level 3 : x<=600 faux => pas level 3
	// Level 1/2/4 : x<=300 faux => pas
	// => return 0
	ObstacleZone oz;
	int level = oz.filtre_levelInFront(200, 600, 300,
			0.0f, 700.0f, 0.0f, 0.0f);
	this->assert(level == 0,
			"adv trop loin devant (x>600 apres patch) => return 0");
}

void test::ObstacleZoneTest::testFiltreBack_TooDeep()
{
	// Symetrique back : x=-700, y=0 trop loin derriere
	ObstacleZone oz;
	int level = oz.filtre_levelInBack(200, 600, 300,
			0.0f, -700.0f, 0.0f, 0.0f);
	this->assert(level == 0,
			"adv trop loin derriere (x<-600 apres patch) => return 0");
}

void test::ObstacleZoneTest::testFiltreBack_DeadBehind_LimitLR()
{
	// Frontiere LR : adv pile derriere, |y| = threshold_LR exact
	// x=-100, y=-150 -> patch x=-150, y=-200 == -threshold_LR
	// Level -1 : y<=-200 OK, y>=-600 OK => level -1 (gagne en premier)
	ObstacleZone oz;
	int level = oz.filtre_levelInBack(200, 600, 300,
			0.0f, -100.0f, -150.0f, 0.0f);
	this->assert(level == -1,
			"frontiere back LR (y=-200 apres patch) => level -1");
}

void test::ObstacleZoneTest::testFiltreBoth_AdvAtX0_Lateral()
{
	// Adv exactement sur l'axe lateral du robot (x=0).
	// Critique : NI front (x>0 faux), NI back (x<0 faux). Aucun mix possible.
	// Patch n'agit pas si x==0 (test strict >0 / <0).
	ObstacleZone oz;
	int levelF = oz.filtre_levelInFront(200, 600, 300, 0.0f, 0.0f, 400.0f, 0.0f);
	int levelB = oz.filtre_levelInBack (200, 600, 300, 0.0f, 0.0f, 400.0f, 0.0f);
	this->assert(levelF == 0, "adv x=0 (axe lateral) => front retourne 0");
	this->assert(levelB == 0, "adv x=0 (axe lateral) => back  retourne 0");
}
