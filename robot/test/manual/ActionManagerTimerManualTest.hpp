/*!
 * \file
 * \brief Tests manuels longs de l'ActionManagerTimer.
 *
 * Verification visuelle des timings dans les logs.
 * Pas d'assertions — l'operateur verifie le comportement attendu.
 */

#ifndef TEST_ACTIONMANAGERTIMERMANUALTEST_HPP
#define TEST_ACTIONMANAGERTIMERMANUALTEST_HPP

#include "../suite/UnitTest.hpp"

namespace test {

class ActionManagerTimerManualTest: public UnitTest {
private:

	static inline const logs::Logger & logger() {
		static const logs::Logger & instance = logs::LoggerFactory::logger("test::ActionManagerTimerManualTest");
		return instance;
	}

public:

	ActionManagerTimerManualTest() :
			UnitTest("ActionManagerTimerManualTest - verification visuelle")
	{
	}

	virtual ~ActionManagerTimerManualTest() {
	}

	virtual void suite();

	void testExecute();
	void testExecutePosix();
	void testExecutePosixBig();
};
}

#endif
