/*!
 * \file
 * \brief Définition de la classe ColorDriverManualTest.
 */

#ifndef OPOS6UL_COLORDRIVERMANUALTEST_HPP
#define OPOS6UL_COLORDRIVERMANUALTEST_HPP


#include "interface/AColorDriver.hpp"
#include "log/LoggerFactory.hpp"
#include "../suite/UnitTest.hpp"

namespace test
{

/*!
 * \brief Teste la classe \ref ColorDriverManualTest.
 */
class ColorDriverManualTest: public UnitTest
{
private:

	/*!
	 * \brief Retourne le \ref Logger associé à la classe \ref ColorDriverManualTest(OPO).
	 */
	static inline const logs::Logger & logger()
	{
		static const logs::Logger & instance = logs::LoggerFactory::logger(
				"ColorDriverManualTest.OPO");
		return instance;
	}

public:

	AColorDriver* colordriver;

	/*!
	 * \brief Constructeur de la classe.
	 */
	ColorDriverManualTest() :
			UnitTest("ColorDriverManualTest")
	{
		colordriver = AColorDriver::create("ColorDriverManualTest");
	}

	/*!
	 * \brief Destructeur de la classe.
	 */
	virtual ~ColorDriverManualTest()
	{
	}

	virtual void suite();

	void firstTest();

};
}

#endif
