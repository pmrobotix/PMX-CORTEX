/*!
 * \file
 * \brief Test manuel de la tirette et des switchs arriere.
 *
 * Lit la tirette en boucle pour verification physique par un operateur.
 * A executer sur le robot en atelier.
 */

#ifndef TEST_SWITCHMANUALTEST_HPP
#define TEST_SWITCHMANUALTEST_HPP

#include "../suite/UnitTest.hpp"
#include "interface/ASwitchDriver.hpp"
#include "log/LoggerFactory.hpp"

namespace test {

/*!
 * \brief Test manuel de la tirette et des switchs.
 */
class SwitchManualTest : public UnitTest
{
private:

	static inline const logs::Logger & logger()
	{
		static const logs::Logger & instance = logs::LoggerFactory::logger("SwitchManualTest");
		return instance;
	}

public:

	ASwitchDriver* switchdriver;

	SwitchManualTest() : UnitTest("SwitchManualTest")
	{
		switchdriver = ASwitchDriver::create("SwitchManualTest");
	}

	virtual ~SwitchManualTest()
	{
	}

	virtual void suite();

	/*!
	 * \brief Lit la tirette en boucle (verification physique par l'operateur).
	 */
	void testTirette();

	/*!
	 * \brief Lit les switchs arriere en boucle.
	 */
	void testBackSwitches();
};

}

#endif
