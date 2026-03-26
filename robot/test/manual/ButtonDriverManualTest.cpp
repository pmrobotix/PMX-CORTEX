/*!
 * \file
 * \brief Test manuel du driver de boutons.
 */

#include "ButtonDriverManualTest.hpp"

#include <unistd.h>
#include <cstdio>

#include "utils/ConsoleKeyInput.hpp"
#include "log/Logger.hpp"

void test::ButtonDriverManualTest::suite()
{
	this->testConsoleKeyInput();
	this->testDriver();
}

void test::ButtonDriverManualTest::testConsoleKeyInput()
{
	logger().info() << "testConsoleKeyInput : appuyez sur les fleches, Enter pour terminer..." << logs::end;
	char cInput;
	do {
		cInput = ConsoleKeyInput::mygetch();
		switch (cInput) {
		case 65:
			printf("Up arrow key!\n");
			break;
		case 66:
			printf("Down arrow key!\n");
			break;
		case 67:
			printf("Right arrow key!\n");
			break;
		case 68:
			printf("Left arrow key!\n");
			break;
		case 10:
			printf("Enter key!\n");
			break;
		case 127:
			printf("BACK key!\n");
			break;
		}
		usleep(1000);
	} while (cInput != 10);

	logger().info() << "testConsoleKeyInput OK" << logs::end;
}

void test::ButtonDriverManualTest::testDriver()
{
	logger().info() << "testDriver : appuyez sur UP/BACK via IPC, Enter pour terminer..." << logs::end;

	while (!buttondriver_->pressed(BUTTON_ENTER_KEY)) {
		if (buttondriver_->pressed(BUTTON_UP_KEY)) {
			logger().info() << "UP" << logs::end;
		}
		if (buttondriver_->pressed(BUTTON_BACK_KEY)) {
			logger().info() << "BACK" << logs::end;
		}
	}

	logger().info() << "testDriver OK" << logs::end;
}
