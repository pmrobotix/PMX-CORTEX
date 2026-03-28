/*!
 * \file
 * \brief Test manuel de l'ecran TFT 7" (framebuffer) et du tactile resistif.
 *
 * Ecran : Santek ST0700-Adapt (800x480, 18 bits, tactile resistif 4 fils)
 * Framebuffer : /dev/fb0 (mxsfb-drm)
 * Touchscreen : /dev/input/event0 (TSC i.MX6ULL)
 */

#ifndef TEST_SCREENMANUALTEST_HPP
#define TEST_SCREENMANUALTEST_HPP

#include "../suite/UnitTest.hpp"
#include "log/LoggerFactory.hpp"

namespace test {

class ScreenManualTest : public UnitTest
{
private:

	static inline const logs::Logger & logger()
	{
		static const logs::Logger & instance = logs::LoggerFactory::logger("ScreenManualTest");
		return instance;
	}

public:

	ScreenManualTest() : UnitTest("ScreenManualTest")
	{
	}

	virtual ~ScreenManualTest()
	{
	}

	virtual void suite();

	/*!
	 * \brief Test framebuffer : affiche des bandes de couleur.
	 */
	void testFramebuffer();

	/*!
	 * \brief Test tactile : lit les events et dessine des croix.
	 */
	void testTouchscreen();
};

}

#endif
