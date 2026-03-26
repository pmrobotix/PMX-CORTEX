/*!
 * \file
 * \brief Test manuel du driver de boutons.
 *
 * Test interactif : attend les appuis clavier via ConsoleKeyInput
 * puis via le driver de boutons (IPC).
 */

#ifndef TEST_BUTTONDRIVERMANUALTEST_HPP
#define TEST_BUTTONDRIVERMANUALTEST_HPP

#include "interface/AButtonDriver.hpp"

#include "../suite/UnitTest.hpp"

namespace test {

class ButtonDriverManualTest: public UnitTest
{
private:

	static inline const logs::Logger & logger()
	{
		static const logs::Logger & instance = logs::LoggerFactory::logger("ButtonDriverManualTest");
		return instance;
	}

public:

	AButtonDriver* buttondriver_;

	ButtonDriverManualTest() :
			UnitTest("ButtonDriverManualTest")
	{
		buttondriver_ = AButtonDriver::create();
	}

	virtual ~ButtonDriverManualTest()
	{
	}

	virtual void suite();

	void testConsoleKeyInput();
	void testDriver();
};
}

#endif
