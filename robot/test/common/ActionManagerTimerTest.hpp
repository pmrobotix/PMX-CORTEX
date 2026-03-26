/*!
 * \file
 * \brief Tests unitaires de la classe ActionManagerTimer.
 *
 * Tests rapides avec assertions. Les tests longs de verification
 * visuelle sont dans test/manual/ActionManagerTimerManualTest.
 */

#ifndef TEST_ACTIONMANAGERTIMERTEST_HPP
#define TEST_ACTIONMANAGERTIMERTEST_HPP

#include "../suite/UnitTest.hpp"

namespace test {

class ActionManagerTimerTest: public UnitTest {
private:

	static inline const logs::Logger & logger() {
		static const logs::Logger & instance = logs::LoggerFactory::logger("test::ActionManagerTimerTest");
		return instance;
	}

public:

	ActionManagerTimerTest() :
			UnitTest("ActionManagerTimerTest")
	{
	}

	virtual ~ActionManagerTimerTest() {
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
