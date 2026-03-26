/*!
 * \file
 * \brief Test manuel du driver d'asservissement.
 *
 * Verification visuelle des commandes moteur et encodeurs.
 */

#ifndef TEST_ASSERVDRIVERMANUALTEST_HPP
#define TEST_ASSERVDRIVERMANUALTEST_HPP

#include "interface/ARobotPositionShared.hpp"
#include "interface/AAsservDriver.hpp"

#include "../suite/UnitTest.hpp"

namespace test {

class AsservDriverManualTest: public UnitTest
{
private:

	static inline const logs::Logger& logger()
	{
		static const logs::Logger &instance = logs::LoggerFactory::logger("AsservDriverManualTest");
		return instance;
	}

	ARobotPositionShared * aRobotPositionShared_;
	AAsservDriver *asservdriver_;

public:

	AsservDriverManualTest();

	virtual ~AsservDriverManualTest()
	{
	}

	virtual void suite();

	void testSet();
};
}

#endif
