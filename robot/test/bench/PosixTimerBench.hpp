/*!
 * \file
 * \brief Benchmark de precision des timers POSIX.
 *
 * Mesure le jitter et la precision des intervalles en microsecondes.
 * A comparer entre x86 (SIMU) et ARM (OPOS6UL).
 */

#ifndef TEST_POSIXTIMERBENCH_HPP
#define TEST_POSIXTIMERBENCH_HPP

#include "../suite/UnitTest.hpp"

namespace test {

class PosixTimerBench: public UnitTest {
private:

	static inline const logs::Logger & logger() {
		static const logs::Logger & instance = logs::LoggerFactory::logger("test::PosixTimerBench");
		return instance;
	}

public:

	PosixTimerBench() :
			UnitTest("PosixTimerBench - precision timers POSIX")
	{
	}

	virtual ~PosixTimerBench() {
	}

	virtual void suite();

	void benchTimer10ms();
	void benchTimer1ms();
	void benchTimer100ms();
};
}

#endif
