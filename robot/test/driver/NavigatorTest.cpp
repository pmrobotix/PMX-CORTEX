/*!
 * \file
 * \brief Tests unitaires de Navigator (simulation uniquement).
 *
 * Tests theoriques : constructeur, enum, policy, pas de mouvement reel.
 * Les tests de mouvement sont dans les tests fonctionnels
 * (O_AsservLineRotateTest, O_AsservWaypointTest).
 */

#ifdef SIMU

#include "NavigatorTest.hpp"

#include <cmath>
#include <vector>

#include "../../src/common/Robot.hpp"
#include "../../src/common/asserv/Asserv.hpp"
#include "../../src/common/navigator/Navigator.hpp"
#include "../../src/common/navigator/RetryPolicy.hpp"
#include "../../src/common/interface/AAsservDriver.hpp"
#include "../../src/common/log/SvgWriter.hpp"

// Asserv concrete minimale pour les tests
class TestAsserv : public Asserv {
public:
    TestAsserv(std::string botId, Robot* robot) : Asserv(botId, robot) {}
};

// SvgWriter stub pour les tests (methodes pure virtual)
class TestSvgWriter : public SvgWriter {
public:
    TestSvgWriter() : SvgWriter("test") { done_ = true; }
    void writePosition_Bot(float, float, float, int) override {}
    void writePosition_BotPos(float, float, float) override {}
    void writePosition_AdvPos(float, float, float, float, int) override {}
    void writeZone(const char*, float, float, float, float, float, float, float) override {}
    void writeIaPath(const char*, const char*, float, float) override {}
    void pathPolyline(std::string) override {}
};

static Robot* testRobot_ = nullptr;
static TestAsserv* testAsserv_ = nullptr;
static TestSvgWriter* testSvg_ = nullptr;

void test::NavigatorTest::initRobot()
{
    if (testRobot_ == nullptr)
    {
        testRobot_ = new Robot();
        testAsserv_ = new TestAsserv("OPOS6UL_Robot", testRobot_);
        testSvg_ = new TestSvgWriter();
        testRobot_->setAsserv(testAsserv_);
        testRobot_->setSVG(testSvg_);
    }
}

void test::NavigatorTest::suite()
{
    initRobot();
    testConstructor();
    testConstructorWithoutIAbyPath();
    testManualPathEmpty();
    testRetryPolicyDefault();
    testPathModeEnum();
}

void test::NavigatorTest::testConstructor()
{
    Navigator nav(testRobot_, nullptr);
    this->assert(true, "Navigator(robot, nullptr) constructeur OK");
}

void test::NavigatorTest::testConstructorWithoutIAbyPath()
{
    Navigator nav(testRobot_);
    this->assert(true, "Navigator(robot) constructeur sans IAbyPath OK");
}

void test::NavigatorTest::testManualPathEmpty()
{
    Navigator nav(testRobot_);
    std::vector<Waypoint> empty;
    TRAJ_STATE ts = nav.manualPath(empty);
    this->assert(ts == TRAJ_FINISHED, "manualPath vide retourne TRAJ_FINISHED");
}

void test::NavigatorTest::testRetryPolicyDefault()
{
    // line/goTo/rotateDeg ont noRetry par defaut
    // On verifie que noRetry a les bonnes valeurs
    RetryPolicy p = RetryPolicy::noRetry();
    this->assert(p.maxObstacleRetries == 1, "noRetry = 1 essai obstacle");
    this->assert(p.maxCollisionRetries == 1, "noRetry = 1 essai collision");
    this->assert(p.waitTempoUs == 0, "noRetry = pas d'attente");

    // manualPath/pathTo ont standard par defaut
    RetryPolicy s = RetryPolicy::standard();
    this->assert(s.maxObstacleRetries == 2, "standard = 2 essais obstacle");
    this->assert(s.maxCollisionRetries == 2, "standard = 2 essais collision");
    this->assert(s.waitTempoUs == 2000000, "standard = 2s attente");
}

void test::NavigatorTest::testPathModeEnum()
{
    // Verifie que les valeurs de l'enum sont distinctes
    this->assert(STOP != CHAIN, "STOP != CHAIN");
    this->assert(CHAIN != CHAIN_NONSTOP, "CHAIN != CHAIN_NONSTOP");
    this->assert(STOP != CHAIN_NONSTOP, "STOP != CHAIN_NONSTOP");

    // Verifie que Waypoint a des valeurs par defaut
    Waypoint wp = {100, 200};
    this->assert(wp.x == 100, "Waypoint x");
    this->assert(wp.y == 200, "Waypoint y");
    this->assert(wp.reverse == false, "Waypoint reverse par defaut = false");

    Waypoint wpRev = {100, 200, true};
    this->assert(wpRev.reverse == true, "Waypoint reverse = true");
}

#endif // SIMU
