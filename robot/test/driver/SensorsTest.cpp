/*!
 * \file
 * \brief Tests unitaires de Sensors (simulation uniquement).
 *
 * Tests de la couche metier : configuration, flags available/ignore,
 * filtres de detection multi-niveaux. Le driver est le stub simu.
 */

#ifdef SIMU

#include "SensorsTest.hpp"

#include "../../src/common/Robot.hpp"
#include "../../src/common/asserv/Asserv.hpp"
#include "../../src/common/action/Actions.hpp"
#include "../../src/common/action/Sensors.hpp"
#include "../../src/common/geometry/TableGeometry.hpp"
#include "../../src/common/interface/ASensorsDriver.hpp"
#include "../../src/common/log/SvgWriter.hpp"

// Stub Asserv minimal
class TestAsservSensors : public Asserv {
public:
	TestAsservSensors(std::string botId, Robot* robot) : Asserv(botId, robot) {}
};

// SvgWriter stub (methodes pure virtual)
class TestSvgWriterSensors : public SvgWriter {
public:
	TestSvgWriterSensors() : SvgWriter("test-sensors") { done_ = true; }
	void writePosition_Bot(float, float, float, int) override {}
	void writePosition_BotPos(float, float, float) override {}
	void writePosition_AdvPos(float, float, float, float, int) override {}
	void writeZone(const char*, float, float, float, float, float, float, float) override {}
	void writeIaPath(const char*, const char*, float, float) override {}
	void pathPolyline(std::string) override {}
};

static Robot* testRobot_ = nullptr;
static TestAsservSensors* testAsserv_ = nullptr;
static TestSvgWriterSensors* testSvg_ = nullptr;
static Actions* testActions_ = nullptr;
static Sensors* testSensors_ = nullptr;

void test::SensorsTest::initRobot()
{
	if (testRobot_ == nullptr)
	{
		testRobot_ = new Robot();
		testRobot_->setTableGeometry(new TableGeometry(3000, 2000, 90, testRobot_->sharedPosition()));
		testAsserv_ = new TestAsservSensors("OPOS6UL_Robot", testRobot_);
		testSvg_ = new TestSvgWriterSensors();
		testRobot_->setAsserv(testAsserv_);
		testRobot_->setSVG(testSvg_);
		testActions_ = new Actions();
		testRobot_->setActions(testActions_);
		testSensors_ = new Sensors(*testActions_, testRobot_);
	}
}

void test::SensorsTest::suite()
{
	initRobot();

	testConstructor();
	testIsConnected();

	testAvailableFlagsFront();
	testAvailableFlagsBack();
	testSetIgnoreAll();

	testClearPositionsAdv();
	testSyncInvalidName();

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

// ---------- Cycle de vie ----------

void test::SensorsTest::testConstructor()
{
	this->assert(testSensors_ != nullptr, "Sensors instancie avec Robot + Actions");
}

void test::SensorsTest::testIsConnected()
{
	this->assert(testSensors_->is_connected(), "is_connected() true en simu");
}

// ---------- Configuration : flags available ----------

void test::SensorsTest::testAvailableFlagsFront()
{
	// enable=true, ignore=false => available true
	testSensors_->addConfigFront(false, true, false);
	testSensors_->setIgnoreFrontNearObstacle(false, false, false);
	this->assert(testSensors_->getAvailableFrontCenter() == true,
			"enableFrontCenter=true, ignore=false => available true");

	// enable=true, ignore=true => available false
	testSensors_->setIgnoreFrontNearObstacle(false, true, false);
	this->assert(testSensors_->getAvailableFrontCenter() == false,
			"enable=true, ignore=true => available false");

	// enable=false => available false
	testSensors_->addConfigFront(false, false, false);
	testSensors_->setIgnoreFrontNearObstacle(false, false, false);
	this->assert(testSensors_->getAvailableFrontCenter() == false,
			"enable=false => available false");
}

void test::SensorsTest::testAvailableFlagsBack()
{
	testSensors_->addConfigBack(false, true, false);
	testSensors_->setIgnoreBackNearObstacle(false, false, false);
	this->assert(testSensors_->getAvailableBackCenter() == true,
			"enableBackCenter=true, ignore=false => available true");

	testSensors_->setIgnoreBackNearObstacle(false, true, false);
	this->assert(testSensors_->getAvailableBackCenter() == false,
			"enable=true, ignore=true => available false");
}

void test::SensorsTest::testSetIgnoreAll()
{
	testSensors_->addConfigFront(true, true, true);
	testSensors_->setIgnoreAllFrontNearObstacle(true);
	this->assert(testSensors_->getAvailableFrontCenter() == false,
			"setIgnoreAllFront(true) => frontCenter unavailable");

	testSensors_->setIgnoreAllFrontNearObstacle(false);
	this->assert(testSensors_->getAvailableFrontCenter() == true,
			"setIgnoreAllFront(false) => frontCenter available");
}

// ---------- Delegation vers driver ----------

void test::SensorsTest::testClearPositionsAdv()
{
	testSensors_->clearPositionsAdv();
	ASensorsDriver::bot_positions positions = testSensors_->setPositionsAdvByBeacon();
	this->assert(positions.empty(),
			"apres clearPositionsAdv(), setPositionsAdvByBeacon() vide");
}

void test::SensorsTest::testSyncInvalidName()
{
	int r = testSensors_->sync("nom_invalide");
	this->assert(r == -1, "sync avec nom invalide retourne -1");
}

// ---------- filtre_levelInFront : tests par niveau ----------
// Contexte : les thresholds sont ajoutes par l'appelant. On fixe :
//   threshold_LR = 200, threshold_Front = 600, threshold_veryclosed_front = 300
// Note : le filtre applique un "patch balise" de +/-50mm sur x et y.

void test::SensorsTest::testFiltreFront_Level1_RightClose()
{
	// Adversaire devant, tres pres, a droite : ydist=100, xdist=200
	// Apres patch : y=150, x=250
	// Level 1 : y<=300 && x>=200 && x<=600 => xdist apres patch = 250 >= 200 OK
	int level = testSensors_->filtre_levelInFront(
			200, 600, 300,  // thresh LR, Front, veryClosed
			0.0f, 200.0f, 100.0f, 0.0f);
	this->assert(level == 1, "adv devant-droit proche => level 1");
}

void test::SensorsTest::testFiltreFront_Level2_LeftClose()
{
	// Adversaire devant, tres pres, a gauche : xdist=-200
	// Apres patch : x=-250, y=150
	// Level 2 : y<=300 && x<=-200 && x>=-600 OK
	int level = testSensors_->filtre_levelInFront(
			200, 600, 300,
			0.0f, -200.0f, 100.0f, 0.0f);
	this->assert(level == 2, "adv devant-gauche proche => level 2");
}

void test::SensorsTest::testFiltreFront_Level3_MidZone()
{
	// Adversaire devant, zone moyenne (entre very closed et front), centre
	// ydist=400 => apres patch y=450 (entre 300 et 600)
	// xdist=0 => apres patch x=0
	int level = testSensors_->filtre_levelInFront(
			200, 600, 300,
			0.0f, 0.0f, 400.0f, 0.0f);
	this->assert(level == 3, "adv zone moyenne centree => level 3");
}

void test::SensorsTest::testFiltreFront_Level4_DeadFront()
{
	// Adversaire pile devant, tres proche : ydist=100, xdist=0
	// Apres patch : y=150, x=0
	// Level 4 : y<=300 && x dans [-200, 200]
	int level = testSensors_->filtre_levelInFront(
			200, 600, 300,
			0.0f, 0.0f, 100.0f, 0.0f);
	this->assert(level == 4, "adv pile devant tres proche => level 4");
}

void test::SensorsTest::testFiltreFront_Outside_Behind()
{
	// Adversaire derriere : ydist<0 => filtre front retourne 0
	int level = testSensors_->filtre_levelInFront(
			200, 600, 300,
			0.0f, 0.0f, -200.0f, 0.0f);
	this->assert(level == 0, "adv derriere => filtre front retourne 0");
}

// ---------- filtre_levelInBack ----------

void test::SensorsTest::testFiltreBack_Level4_DeadBehind()
{
	// Adversaire pile derriere, tres proche : ydist=-100, xdist=0
	// Apres patch : y=-150, x=0
	// Level -4 : y>=-300 && x dans [-200, 200]
	int level = testSensors_->filtre_levelInBack(
			200, 600, 300,
			0.0f, 0.0f, -100.0f, 0.0f);
	this->assert(level == -4, "adv pile derriere tres proche => level -4");
}

void test::SensorsTest::testFiltreBack_Outside_InFront()
{
	// Adversaire devant : ydist>0 => filtre back retourne 0
	int level = testSensors_->filtre_levelInBack(
			200, 600, 300,
			0.0f, 0.0f, 200.0f, 0.0f);
	this->assert(level == 0, "adv devant => filtre back retourne 0");
}

// ---------- Cas limites ----------

void test::SensorsTest::testFiltreFront_OverlapLevel1and4()
{
	// A la frontiere threshold_LR, level 1 et level 4 sont tous les deux vrais.
	// L'ordre dans le code : level 1 teste en premier, level 1 gagne.
	// ydist=100 => apres patch y=150
	// xdist=150 => apres patch x=200 == threshold_LR_mm
	// Level 1 : x>=200 && x<=600 OK
	// Level 4 : x>=-200 && x<=200 OK aussi
	int level = testSensors_->filtre_levelInFront(
			200, 600, 300,
			0.0f, 150.0f, 100.0f, 0.0f);
	this->assert(level == 1,
			"frontiere x=threshold_LR : level 1 (prioritaire) gagne");
}

void test::SensorsTest::testFiltreFront_ZeroPosition()
{
	// xdist=0, ydist=0 : patch balise ne s'applique pas (test strict sur > et <)
	// Donc ydist=0 => la condition ydist>0 est fausse => retourne 0
	// Ceci documente le comportement actuel.
	int level = testSensors_->filtre_levelInFront(
			200, 600, 300,
			0.0f, 0.0f, 0.0f, 0.0f);
	this->assert(level == 0,
			"adv en (0,0) : y>0 faux => retourne 0");
}

#endif // SIMU
