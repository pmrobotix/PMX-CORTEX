/*!
 * \file
 * \brief Test unitaire du driver de switchs via l'interface ASwitchDriver.
 *
 * Verifie le comportement fonctionnel : connexion, valeurs retournees.
 * Le test manuel de la tirette est dans test/manual/SwitchManualTest.
 */

#ifndef TEST_SWITCHDRIVERTEST_HPP
#define TEST_SWITCHDRIVERTEST_HPP

#include "../suite/UnitTest.hpp"
#include "interface/ASwitchDriver.hpp"
#include "log/LoggerFactory.hpp"

namespace test {

/*!
 * \brief Test unitaire fonctionnel du driver de switchs.
 */
class SwitchDriverTest : public UnitTest
{
private:

	static inline const logs::Logger & logger()
	{
		static const logs::Logger & instance = logs::LoggerFactory::logger("SwitchDriverTest");
		return instance;
	}

public:

	ASwitchDriver* switchdriver;

	SwitchDriverTest() : UnitTest("SwitchDriverTest")
	{
		switchdriver = ASwitchDriver::create("SwitchDriverTest");
	}

	virtual ~SwitchDriverTest()
	{
	}

	virtual void suite();

	/*!
	 * \brief Verifie que create() retourne une instance connectee.
	 */
	void testConnected();

	/*!
	 * \brief Verifie les valeurs par defaut des switchs.
	 */
	void testDefaultValues();
};

}

#endif
