/*!
 * \file
 * \brief Test manuel de l'ecran TFT 7" (framebuffer) et du tactile resistif.
 *
 * Ecran : Santek ST0700-Adapt (800x480, 18 bits, tactile resistif 4 fils)
 * Framebuffer : /dev/fb0 (mxsfb-drm)
 * Touchscreen : /dev/input/event0 (TSC i.MX6ULL)
 */

#ifndef TEST_SCREENFRAMEBUFFERMANUALTEST_HPP
#define TEST_SCREENFRAMEBUFFERMANUALTEST_HPP

#include "../suite/UnitTest.hpp"
#include "log/LoggerFactory.hpp"

namespace test {

class ScreenFramebufferManualTest : public UnitTest
{
private:

	static inline const logs::Logger & logger()
	{
		static const logs::Logger & instance = logs::LoggerFactory::logger("ScreenFramebufferManualTest");
		return instance;
	}

public:

	ScreenFramebufferManualTest() : UnitTest("ScreenFramebufferManualTest")
	{
	}

	virtual ~ScreenFramebufferManualTest()
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

	/*!
	 * \brief Verification de la calibration : affiche 5 croix et mesure l'erreur.
	 */
	void testVerifyCalibration();

	/*!
	 * \brief Calibration tactile 3 points : affiche des croix cibles et
	 *        mesure les coordonnees brutes pour calculer les coefficients.
	 */
	void testCalibration();
};

}

#endif
