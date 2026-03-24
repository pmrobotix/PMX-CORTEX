/*!
 * \file
 * \brief Implementation du test manuel de la tirette et des switchs.
 */

#include "SwitchManualTest.hpp"

#include <unistd.h>

void test::SwitchManualTest::suite()
{
	testTirette();
	testBackSwitches();
}

void test::SwitchManualTest::testTirette()
{
	logger().info() << "testTirette : lecture 10 fois (verification physique)" << logs::end;

	for (int i = 0; i < 10; i++) {
		int s = switchdriver->tirettePressed();
		logger().info() << "tirette = " << s << logs::end;
		usleep(400000);
	}
}

void test::SwitchManualTest::testBackSwitches()
{
	logger().info() << "testBackSwitches : lecture 5 fois (verification physique)" << logs::end;

	for (int i = 0; i < 5; i++) {
		int left = switchdriver->backLeftPressed();
		int right = switchdriver->backRightPressed();
		logger().info() << "backLeft=" << left << " backRight=" << right << logs::end;
		usleep(400000);
	}
}
