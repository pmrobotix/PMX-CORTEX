/*!
 * \file
 * \brief Tests manuels longs de l'ActionManagerPosixTimer.
 */

#include "timer/ActionManagerPosixTimer.hpp"
#include "ActionManagerPosixTimerManualTest.hpp"

#include <chrono>
#include <sstream>
#include <string>
#include <thread>

#include "action/IAction.hpp"
#include "timer/ITimerListener.hpp"
#include "timer/ITimerPosixListener.hpp"
#include "utils/Chronometer.hpp"
#include "log/Logger.hpp"
#include "log/LoggerFactory.hpp"

using namespace utils;

// --- Mocks ---

class MockAction: public IAction {
private:

    static inline const logs::Logger& logger() {
        static const logs::Logger &instance = logs::LoggerFactory::logger("test::MockAction");
        return instance;
    }

    long int lasttime_;
    utils::Chronometer chronoA_;
    long nbmax_;
    long nb_;

public:

    MockAction(int num, long nbmax) :
            chronoA_("MockAction")
    {
        std::ostringstream oss;
        oss << "MockAction" << num;
        name_ = oss.str();

        lasttime_ = 0;
        chronoA_.start();
        nb_ = 0;
        nbmax_ = nbmax;
    }

    virtual ~MockAction() {
    }

    bool execute() {
        long int timechrono = chronoA_.getElapsedTimeInMicroSec();
        logger().info() << "executing " << name_ << " nb=" << nb_ << " ... t=" << timechrono - lasttime_ << "us" << logs::end;
        lasttime_ = timechrono;
        nb_++;
        if (nb_ < nbmax_) {
            return true;
        }
        else {
            logger().info() << "===> last executing " << name_ << " nb=" << nb_ << logs::end;
            nb_ = 0;
            lasttime_ = 0;
            chronoA_.stop();
            return false;
        }
    }
};

class MockPosixTimer: public ITimerPosixListener {
private:

    static inline const logs::Logger& logger() {
        static const logs::Logger &instance = logs::LoggerFactory::logger("test::MockPosixTimer");
        return instance;
    }

    long last_t_;

public:

    MockPosixTimer(std::string label, int time_ms)
    {
        this->init(label, time_ms * 1000);
        last_t_ = 0;
    }

    virtual ~MockPosixTimer() {
    }

    void onTimer(utils::Chronometer chrono) {
        long t = chrono.getElapsedTimeInMicroSec();
        logger().info() << "onTimer executing " << name_ << "... t=" << t << " us  : " << t - last_t_ << logs::end;
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        last_t_ = t;
    }

    void onTimerEnd(utils::Chronometer chrono) {
        logger().info() << "onTimerEnd executing " << name_ << "... t=" << chrono.getElapsedTimeInMicroSec() << " us" << logs::end;
    }
};

class MockTimer: public ITimerListener {
private:

    static inline const logs::Logger& logger() {
        static const logs::Logger &instance = logs::LoggerFactory::logger("test::MockTimer");
        return instance;
    }

public:

    MockTimer(std::string label, int timeSpan_ms)
    {
        timeSpan_ms_ = timeSpan_ms;
        name_ = label;
    }

    virtual ~MockTimer() {
    }

    void onTimer(utils::Chronometer chrono) {
        logger().info() << "onTimer executing " << name_ << "... t=" << chrono.getElapsedTimeInMicroSec() << " us" << logs::end;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        logger().info() << "onTimer executing finished " << name_ << "... t=" << chrono.getElapsedTimeInMicroSec() << " us" << logs::end;
    }

    void onTimerEnd(utils::Chronometer chrono) {
        logger().info() << "onTimerEnd executing " << name_ << "... t=" << chrono.getElapsedTimeInMicroSec() << " us" << logs::end;
    }
};

// --- Suite ---

void test::ActionManagerPosixTimerManualTest::suite() {
    testExecutePosixBig();
}

// --- Tests manuels (verification visuelle des logs) ---

void test::ActionManagerPosixTimerManualTest::testExecute() {
    logger().info() << "testExecute()..." << logs::end;
    ActionManagerPosixTimer manager;

    MockAction *action1 = new MockAction(1, 10);
    MockAction *action2 = new MockAction(2, 20);
    MockTimer *timer1 = new MockTimer("timer1", 1000);
    MockTimer *timer2 = new MockTimer("timer2", 500);

    manager.addTimer(timer1);
    manager.addTimer(timer2);
    manager.addAction(action1);

    manager.start("ActionsAndTimers", 1);

    logger().debug() << "wait 8sec" << logs::end;
    std::this_thread::sleep_for(std::chrono::seconds(8));
    manager.stopTimer("timer1");
    manager.addAction(action2);
    manager.addAction(action1);

    manager.debugTimers();
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    manager.pause(true);
    logger().debug() << "pause 4sec" << logs::end;
    std::this_thread::sleep_for(std::chrono::seconds(4));
    manager.pause(false);
    manager.addAction(action1);
    manager.addTimer(timer1);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    manager.stopTimer("timer2");
    manager.debugTimers();

    manager.stop();
    logger().info() << "testExecute() END" << logs::end;
}

void test::ActionManagerPosixTimerManualTest::testExecutePosixBig() {
    logger().info() << "testExecutePosixBig()..." << logs::end;

    ActionManagerPosixTimer manager;

    MockAction *action1 = new MockAction(1, 10);
    MockAction *action2 = new MockAction(2, 20);
    MockAction *action3 = new MockAction(3, 10);
    MockPosixTimer *ptimer1 = new MockPosixTimer("ptimer1", 250);
    MockPosixTimer *ptimer2 = new MockPosixTimer("ptimer2", 500);

    manager.debugPTimers();
    manager.debugActions();

    manager.start("actionsAndTimers", 2);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    manager.stop();
    std::this_thread::sleep_for(std::chrono::seconds(2));

    manager.addTimer(ptimer1);
    manager.addTimer(ptimer2);
    manager.addAction(action1);
    manager.addAction(action3);

    std::this_thread::sleep_for(std::chrono::seconds(1));
    manager.start("actionsAndTimersv2", 20);

    std::this_thread::sleep_for(std::chrono::seconds(4));
    logger().info() << "add actions 1+2..." << logs::end;
    manager.addAction(action1);
    manager.addAction(action2);
    manager.addTimer(ptimer1);

    std::this_thread::sleep_for(std::chrono::seconds(2));
    manager.debugPTimers();
    manager.debugActions();

    manager.stop();
    logger().info() << "testExecutePosixBig() END" << logs::end;
}

void test::ActionManagerPosixTimerManualTest::testExecutePosix() {
    logger().info() << "testExecutePosix()..." << logs::end;
    ActionManagerPosixTimer manager;

    MockAction *action1 = new MockAction(1, 1000);
    MockAction *action2 = new MockAction(2, 20);
    MockAction *action3 = new MockAction(3, 30);
    MockPosixTimer *ptimer1 = new MockPosixTimer("ptimer1", 1000);
    MockPosixTimer *ptimer2 = new MockPosixTimer("ptimer2", 200);

    manager.addTimer(ptimer1);
    manager.addTimer(ptimer2);
    manager.addAction(action1);
    manager.addAction(action3);

    manager.debugPTimers();
    manager.debugActions();

    manager.start("actionsAndTimers", 0);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    manager.debugPTimers();
    manager.debugActions();

    logger().debug() << "wait 5sec" << logs::end;
    std::this_thread::sleep_for(std::chrono::seconds(5));

    manager.stopPTimer("ptimer1");
    manager.addAction(action2);
    manager.addAction(action1);

    manager.debugPTimers();
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    manager.pause(true);
    logger().debug() << "pause 4sec" << logs::end;
    std::this_thread::sleep_for(std::chrono::seconds(4));
    manager.pause(false);

    manager.addTimer(ptimer1);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    manager.stopPTimer("ptimer2");
    manager.stopPTimer("ptimer1");
    manager.debugPTimers();

    manager.addTimer(ptimer1);
    manager.addTimer(ptimer1);
    manager.debugPTimers();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    manager.stopAllPTimers();
    manager.debugPTimers();

    manager.stop();
    logger().info() << "testExecutePosix() END" << logs::end;
}
