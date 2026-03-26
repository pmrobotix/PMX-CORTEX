/*!
 * \file
 * \brief Point d'entree des benchmarks.
 *
 * Ces tests mesurent des metriques de performance (temps, debit, resolution).
 * Pas d'assertions — les resultats sont affiches dans les logs.
 */

#include "../../src/common/thread/Thread.hpp"
#include "../suite/UnitTestSuite.hpp"

#include "PosixTimerBench.hpp"

int main(int, char**)
{
	utils::set_realtime_priority(3, "bench");

	UnitTestSuite suite;

	suite.addTest(new test::PosixTimerBench());

	suite.run();

	return 0;
}
