/*!
 * \file
 * \brief Définition de la classe SensorDriverManualTest.
 */

#ifndef OPOS6UL_SENSORDRIVERMANUALTEST_HPP
#define OPOS6UL_SENSORDRIVERMANUALTEST_HPP


#include "interface/ARobotPositionShared.hpp"
#include "interface/ASensorsDriver.hpp"
#include "log/LoggerFactory.hpp"
#include "../suite/UnitTest.hpp"

namespace test
{

/*!
 * \brief Teste la classe \ref SensorDriverManualTest.
 */
class SensorDriverManualTest: public UnitTest
{
private:

	/*!
	 * \brief Retourne le \ref Logger associé à la classe \ref SensorDriverManualTest(OPO).
	 */
	static inline const logs::Logger & logger()
	{
		static const logs::Logger & instance = logs::LoggerFactory::logger(
				"SensorDriverManualTest.OPO");
		return instance;
	}

public:

	ASensorsDriver* sensordriver;
	ARobotPositionShared * aRobotPositionShared_;

	/*!
	 * \brief Constructeur de la classe.
	 */
	SensorDriverManualTest() :
			UnitTest("SensorDriverManualTest")
	{
	    aRobotPositionShared_ = ARobotPositionShared::create();
	    sensordriver = ASensorsDriver::create("SensorDriverManualTest", aRobotPositionShared_);
	}

	/*!
	 * \brief Destructeur de la classe.
	 */
	virtual ~SensorDriverManualTest()
	{
	}

	virtual void suite();

	void firstTest();

};
}

#endif
