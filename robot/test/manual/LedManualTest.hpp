/*!
 * \file
 * \brief Test manuel visuel du driver de LEDs.
 *
 * Fait clignoter les LEDs une par une puis en patterns alternants.
 * Verification visuelle par un humain : "les LEDs clignotent correctement".
 */

#ifndef TEST_LEDMANUALTEST_HPP
#define TEST_LEDMANUALTEST_HPP

#include "../suite/UnitTest.hpp"
#include "interface/ALedDriver.hpp"
#include "log/LoggerFactory.hpp"

namespace test {

/*!
 * \brief Test manuel visuel des LEDs.
 *
 * Ce test ne verifie rien programmatiquement — il produit un effet
 * visuel que l'operateur doit observer sur le robot.
 */
class LedManualTest : public UnitTest
{
private:

	static inline const logs::Logger & logger()
	{
		static const logs::Logger & instance = logs::LoggerFactory::logger("LedManualTest");
		return instance;
	}

public:

	ALedDriver* leddriver;

	LedManualTest() : UnitTest("LedManualTest")
	{
		leddriver = ALedDriver::create("LedManualTest", 8);
	}

	virtual ~LedManualTest()
	{
	}

	virtual void suite();

	/*!
	 * \brief Allume/eteint chaque LED une par une (verification visuelle).
	 */
	void testBlinkEach();

	/*!
	 * \brief Alterne les patterns 0xAA et 0x55 (verification visuelle).
	 */
	void testAlternatePattern();
};

}

#endif
