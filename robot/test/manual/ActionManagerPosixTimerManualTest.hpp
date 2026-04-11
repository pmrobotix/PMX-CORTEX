/*!
 * \file
 * \brief Tests manuels longs de l'ActionManagerPosixTimer (legacy, deprecated).
 *
 * Verification visuelle des timings dans les logs.
 * Pas d'assertions — l'operateur verifie le comportement attendu.
 */

#ifndef TEST_ACTIONMANAGERPOSIXTIMERMANUALTEST_HPP
#define TEST_ACTIONMANAGERPOSIXTIMERMANUALTEST_HPP

#include "../suite/UnitTest.hpp"

namespace test {

class ActionManagerPosixTimerManualTest: public UnitTest {
private:

	static inline const logs::Logger & logger() {
		static const logs::Logger & instance = logs::LoggerFactory::logger("test::ActionManagerPosixTimerManualTest");
		return instance;
	}

public:

	ActionManagerPosixTimerManualTest() :
			UnitTest("ActionManagerPosixTimerManualTest - verification visuelle")
	{
	}

	virtual ~ActionManagerPosixTimerManualTest() {
	}

	virtual void suite();

	void testExecute();
	void testExecutePosix();
	void testExecutePosixBig();
};
}

#endif
