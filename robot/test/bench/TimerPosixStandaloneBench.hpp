/*!
 * \file
 * \brief Bench du timer POSIX standalone (sans ActionManagerTimer).
 *
 * Verification visuelle du fonctionnement bas niveau des signaux POSIX.
 */

#ifndef TEST_TIMERPOSIXSTANDALONEBENCH_HPP
#define TEST_TIMERPOSIXSTANDALONEBENCH_HPP

#include "../suite/UnitTest.hpp"

namespace test {

class TimerPosixStandaloneBench: public UnitTest {
private:

	static inline const logs::Logger & logger() {
		static const logs::Logger & instance = logs::LoggerFactory::logger("test::TimerPosixStandaloneBench");
		return instance;
	}

public:

	TimerPosixStandaloneBench() :
			UnitTest("TimerPosixStandaloneBench - timer POSIX standalone")
	{
	}

	virtual ~TimerPosixStandaloneBench() {
	}

	virtual void suite();

	void test();
};
}

#endif
