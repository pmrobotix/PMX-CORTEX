/*!
 * \file
 * \brief Tests unitaires de la classe ActionManagerPosixTimer (legacy, deprecated).
 *
 * Tests rapides avec assertions. Les tests longs de verification
 * visuelle sont dans test/manual/ActionManagerPosixTimerManualTest.
 */

#ifndef TEST_ACTIONMANAGERPOSIXTIMERTEST_HPP
#define TEST_ACTIONMANAGERPOSIXTIMERTEST_HPP

#include "../suite/UnitTest.hpp"

namespace test {

class ActionManagerPosixTimerTest: public UnitTest {
private:

	static inline const logs::Logger & logger() {
		static const logs::Logger & instance = logs::LoggerFactory::logger("test::ActionManagerPosixTimerTest");
		return instance;
	}

public:

	ActionManagerPosixTimerTest() :
			UnitTest("ActionManagerPosixTimerTest")
	{
	}

	virtual ~ActionManagerPosixTimerTest() {
	}

	virtual void suite();

	void testInitialState();
	void testAddRemoveActions();
	void testAddRemoveTimers();
	void testAddRemovePosixTimers();
	void testActionExecution();
	void testStopRestart();
	void testPauseResume();
	void testActionFinishesAndRemoved();
	void testMultipleActionsOrdering();
	void testStopTimerByName();
	void testStopPTimerByName();
	void testStopAllPTimers();
	void testConcurrentAddAction();
};
}

#endif
