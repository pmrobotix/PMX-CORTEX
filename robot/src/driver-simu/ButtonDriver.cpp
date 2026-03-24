/*!
 * \file
 * \brief Implementation SIMU du driver de boutons.
 */

#include "ButtonDriver.hpp"

#include <unistd.h>

#include "log/Logger.hpp"

AButtonDriver * AButtonDriver::create()
{
	return new ButtonDriver();
}

ButtonDriver::ButtonDriver()
{
	lindex = 0;
	back_ = false;
	enter_ = false;
	up_ = false;
	down_ = false;
	left_ = false;
	right_ = false;

	stop_ = false;
	thread_created_ = 0;
}

ButtonDriver::~ButtonDriver()
{
	stop_ = true;
	if (tbutton_.joinable())
		tbutton_.join();
}

bool ButtonDriver::pressed(ButtonTouch button)
{
	if (thread_created_ == 0) {
		ButtonDriverWrapper *w_ = new ButtonDriverWrapper(this);
		tbutton_ = w_->buttonThread("ButtonDriver", 0);
		thread_created_ = 1;
	}

	switch (button) {
	case BUTTON_ENTER_KEY:
		if (enter_) {
			logger().debug() << "Enter key!" << logs::end;
			return true;
		}
		break;
	case BUTTON_BACK_KEY:
		if (back_) {
			logger().debug() << "BACK key!" << logs::end;
			return true;
		}
		break;
	case BUTTON_UP_KEY:
		if (up_) {
			logger().debug() << "UP arrow key!" << logs::end;
			return true;
		}
		break;
	case BUTTON_DOWN_KEY:
		if (down_) {
			logger().debug() << "DOWN arrow key!" << logs::end;
			return true;
		}
		break;
	case BUTTON_LEFT_KEY:
		if (left_) {
			logger().debug() << "LEFT arrow key!" << logs::end;
			return true;
		}
		break;
	case BUTTON_RIGHT_KEY:
		if (right_) {
			logger().debug() << "RIGHT arrow key!" << logs::end;
			return true;
		}
		break;
	default:
		return false;
	}

	usleep(10000); // anti-rebond

	return false;
}
