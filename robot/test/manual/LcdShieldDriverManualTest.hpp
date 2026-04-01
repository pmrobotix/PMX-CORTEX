/*!
 * \file
 * \brief Définition de la classe LcdShieldDriverManualTest.
 */

#ifndef OPOS6UL_LCDSHIELDDRIVERMANUALTEST_HPP
#define OPOS6UL_LCDSHIELDDRIVERMANUALTEST_HPP

#include <stddef.h>
#include <string>

#include "interface/ALcdShieldDriver.hpp"
#include "log/LoggerFactory.hpp"
#include "../suite/UnitTest.hpp"

namespace test
{

/*!
 * \brief Teste la classe \ref LcdShieldDriver.
 */
class LcdShieldDriverManualTest: public UnitTest
{
private:

	/*!
	 * \brief Retourne le \ref Logger associé à la classe \ref LcdShieldDriverManualTest(OPO).
	 */
	static inline const logs::Logger & logger()
	{
		static const logs::Logger & instance = logs::LoggerFactory::logger(
				"LcdShieldDriverManualTest.OPO");
		return instance;
	}

public:

	ALcdShieldDriver* lcdshielddriver;

	/*!
	 * \brief Constructeur de la classe.
	 */
	LcdShieldDriverManualTest() :
			UnitTest("LcdShieldDriverManualTest")
	{
		lcdshielddriver = ALcdShieldDriver::create("LcdShieldDriverManualTest");
	}

	/*!
	 * \brief Destructeur de la classe.
	 */
	virtual ~LcdShieldDriverManualTest()
	{
	}

	virtual void suite();

	void test();

};
}

#endif
