/*!
 * \file
 * \brief Test unitaire du driver de LEDs.
 *
 * Verifie le comportement fonctionnel via l'interface ALedDriver.
 * En SIMU, on cast vers LedDriver pour verifier l'etat interne (gpio[]).
 */

#ifndef TEST_LEDDRIVERTEST_HPP
#define TEST_LEDDRIVERTEST_HPP

#include "../suite/UnitTest.hpp"
#include "interface/ALedDriver.hpp"
#include "log/LoggerFactory.hpp"

namespace test {

/*!
 * \brief Test unitaire fonctionnel du driver de LEDs.
 */
class LedDriverTest : public UnitTest
{
private:

	static inline const logs::Logger & logger()
	{
		static const logs::Logger & instance = logs::LoggerFactory::logger("LedDriverTest");
		return instance;
	}

public:

	ALedDriver* leddriver;

	LedDriverTest() : UnitTest("LedDriverTest")
	{
		leddriver = ALedDriver::create("LedDriverTest", 8);
	}

	virtual ~LedDriverTest()
	{
	}

	virtual void suite();

	/*!
	 * \brief Verifie que create() retourne une instance valide.
	 */
	void testCreate();

	/*!
	 * \brief Verifie que setBit met la bonne couleur au bon index.
	 */
	void testSetBit();

	/*!
	 * \brief Verifie que setBytes applique le masque correctement.
	 */
	void testSetBytes();

	/*!
	 * \brief Verifie que setBytes(0x00, OFF) remet tout a zero.
	 */
	void testReset();
};

}

#endif
