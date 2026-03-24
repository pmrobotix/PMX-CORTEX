/*!
 * \file
 * \brief Point d'entree des benchmarks.
 *
 * Ces tests mesurent des metriques de performance (temps, debit, resolution).
 * Pas d'assertions — les resultats sont affiches dans les logs.
 */

#include "../../src/common/thread/Thread.hpp"
#include "../suite/UnitTestSuite.hpp"

int main(int, char**)
{
	utils::set_realtime_priority(3, "bench");

	UnitTestSuite suite;

	// Ajouter les benchmarks ici :
	// suite.addTest(new test::ChronometerBench());
	// suite.addTest(new test::ReadWriteBench());

	suite.run();

	return 0;
}
