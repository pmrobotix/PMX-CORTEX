/*!
 * \file
 * \brief Point d'entree des benchmarks.
 *
 * Ces tests mesurent des metriques de performance (temps, debit, resolution).
 * Pas d'assertions — les resultats sont affiches dans les logs.
 */

#include "../../src/common/thread/Thread.hpp"
#include "../suite/UnitTestSuite.hpp"
#include <sys/mman.h>
#include <cerrno>
#include <cstring>
#include <iostream>

#include "PosixTimerBench.hpp"
#include "TimerPosixStandaloneBench.hpp"

int main(int, char**)
{
	if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
		std::cerr << "mlockall FAILED: " << strerror(errno) << std::endl;
	}
	utils::set_realtime_priority(3, "bench");

	UnitTestSuite suite;

	suite.addTest(new test::PosixTimerBench());
	suite.addTest(new test::TimerPosixStandaloneBench());

	suite.run();

	return 0;
}
