/*!
 * \file
 * \brief Implementation de MenuShieldLCD (nouveau modele a 3 phases).
 */

#include "MenuShieldLCD.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "log/Logger.hpp"
#include "action/LcdShield.hpp"
#include "action/ButtonBar.hpp"
#include "Robot.hpp"

// SELECT thresholds (secondes)
static constexpr double SELECT_ACTION_MAX  = 0.5;   // clic court < 0.5s = setPos/reset/test
static constexpr double SELECT_EXIT_SEC    = 2.0;   // hold >= 2s = exit(0)

MenuShieldLCD::MenuShieldLCD(LcdShield& lcd, ButtonBar& btns) :
		lcd_(lcd), btns_(btns), holdChrono_("MenuShieldLCD.hold")
{
	alive_ = lcd_.is_connected();
	logger().info() << "MenuShieldLCD ctor alive=" << alive_ << logs::end;
}

const char* MenuShieldLCD::strategyShort(const char* longName)
{
	if (!longName) return "S?";
	if (std::strcmp(longName, "tabletest") == 0) return "S1";
	if (std::strcmp(longName, "strat2") == 0)    return "S2";
	if (std::strcmp(longName, "strat3") == 0)    return "S3";
	if (std::strcmp(longName, "all") == 0)       return "S-";
	return "S?";
}

void MenuShieldLCD::buildLine0(const Robot& robot, char out[17]) const
{
	// Format: "BLUE     S1 D:40" ou "YELLW* S1 D:40" (16 chars + terminator)
	const char* colorStr = (robot.getMyColor() == PMXYELLOW) ? "YELLW" : "BLUE";
	char lockChar = (robot.phase() >= PHASE_ARMED) ? '*' : ' ';

	std::snprintf(out, 17, "%-6s%c%s   D:%d",
			colorStr, lockChar,
			strategyShort(robot.strategy().c_str()),
			static_cast<int>(robot.advDiameter()));
	out[16] = '\0';
}

void MenuShieldLCD::buildLine1(const Robot& robot, char out[17]) const
{
	// Ligne contextuelle : rubrique + valeur en cours d'edition
	const char* fieldName = "?";
	char value[12] = "?";

	switch (editField_) {
		case FIELD_COLOR:
			fieldName = "COLOR";
			std::snprintf(value, sizeof(value), "%s",
					robot.getMyColor() == PMXYELLOW ? "YELLOW" : "BLUE");
			break;
		case FIELD_DIAM:
			fieldName = "DIAM ";
			std::snprintf(value, sizeof(value), "%d cm",
					static_cast<int>(robot.advDiameter()));
			break;
		case FIELD_STRAT:
			fieldName = "STRAT";
			std::snprintf(value, sizeof(value), "%s",
					strategyShort(robot.strategy().c_str()));
			break;
		case FIELD_TEST:
			fieldName = "TEST ";
			std::snprintf(value, sizeof(value), "T%d", static_cast<int>(testSelected_));
			break;
		default: break;
	}
	std::snprintf(out, 17, ">%s:%-9s", fieldName, value);
	out[16] = '\0';
}

void MenuShieldLCD::nextField(const Robot& robot)
{
	// Cycle COLOR -> DIAM -> STRAT -> TEST. En ARMED on saute COLOR.
	do {
		editField_ = static_cast<EditField>((editField_ + 1) % FIELD_COUNT);
	} while (editField_ == FIELD_COLOR && robot.phase() >= PHASE_ARMED);
}

void MenuShieldLCD::prevField(const Robot& robot)
{
	do {
		editField_ = static_cast<EditField>((editField_ + FIELD_COUNT - 1) % FIELD_COUNT);
	} while (editField_ == FIELD_COLOR && robot.phase() >= PHASE_ARMED);
}

void MenuShieldLCD::incrementCurrentField(Robot& robot)
{
	switch (editField_) {
		case FIELD_COLOR: {
			bool ok = robot.setMyColorChecked(PMXBLUE);
			logger().info() << "[CLICK] UP on COLOR -> BLUE ok=" << ok
					<< " (phase=" << (int)robot.phase() << ")" << logs::end;
			break;
		}
		case FIELD_DIAM: {
			int d = robot.advDiameter() + 5;
			if (d > 250) d = 5;
			bool ok = robot.setAdvDiameter(static_cast<uint8_t>(d));
			logger().info() << "[CLICK] UP on DIAM -> " << d << " ok=" << ok << logs::end;
			break;
		}
		case FIELD_STRAT: {
			const std::string s = robot.strategy();
			std::string next;
			if (s == "tabletest")  next = "strat2";
			else if (s == "strat2") next = "strat3";
			else                    next = "tabletest";
			bool ok = robot.setStrategyChecked(next);
			logger().info() << "[CLICK] UP on STRAT -> " << next << " ok=" << ok << logs::end;
			break;
		}
		case FIELD_TEST:
			if (testSelected_ < 5) testSelected_++;
			else testSelected_ = 1;
			logger().info() << "[CLICK] UP on TEST -> T" << (int)testSelected_ << logs::end;
			break;
		default: break;
	}
}

void MenuShieldLCD::decrementCurrentField(Robot& robot)
{
	switch (editField_) {
		case FIELD_COLOR: {
			bool ok = robot.setMyColorChecked(PMXYELLOW);
			logger().info() << "[CLICK] DOWN on COLOR -> YELLOW ok=" << ok
					<< " (phase=" << (int)robot.phase() << ")" << logs::end;
			break;
		}
		case FIELD_DIAM: {
			int d = robot.advDiameter() - 5;
			if (d < 5) d = 250;
			bool ok = robot.setAdvDiameter(static_cast<uint8_t>(d));
			logger().info() << "[CLICK] DOWN on DIAM -> " << d << " ok=" << ok << logs::end;
			break;
		}
		case FIELD_STRAT: {
			const std::string s = robot.strategy();
			std::string next;
			if (s == "tabletest")  next = "strat3";
			else if (s == "strat2") next = "tabletest";
			else                    next = "strat2";
			bool ok = robot.setStrategyChecked(next);
			logger().info() << "[CLICK] DOWN on STRAT -> " << next << " ok=" << ok << logs::end;
			break;
		}
		case FIELD_TEST:
			if (testSelected_ > 1) testSelected_--;
			else testSelected_ = 5;
			logger().info() << "[CLICK] DOWN on TEST -> T" << (int)testSelected_ << logs::end;
			break;
		default: break;
	}
}

void MenuShieldLCD::handleClick(Robot& robot, ButtonTouch b)
{
	switch (b) {
		case BUTTON_LEFT_KEY:
			prevField(robot);
			logger().info() << "[CLICK] LEFT -> editField=" << (int)editField_ << logs::end;
			break;
		case BUTTON_RIGHT_KEY:
			nextField(robot);
			logger().info() << "[CLICK] RIGHT -> editField=" << (int)editField_ << logs::end;
			break;
		case BUTTON_UP_KEY:    incrementCurrentField(robot); break;
		case BUTTON_DOWN_KEY:  decrementCurrentField(robot); break;
		case BUTTON_ENTER_KEY:
			break;
		case BUTTON_BACK_KEY:
			// SELECT clic court = setPos/reset/triggerTest
			// SELECT hold 3s+ = exit (gere par checkHoldBack)
			if (editField_ == FIELD_TEST) {
				robot.triggerTestMode(testSelected_);
				logger().info() << "[CLICK] SELECT on TEST -> triggerTestMode T"
						<< (int)testSelected_ << logs::end;
			} else if (robot.phase() == PHASE_CONFIG) {
				logger().info() << "[CLICK] SELECT -> requestSetPos" << logs::end;
				robot.requestSetPos();
			} else if (robot.phase() == PHASE_ARMED) {
				logger().info() << "[CLICK] SELECT -> requestReset" << logs::end;
				robot.requestReset();
			}
			break;
		default: break;
	}
}

void MenuShieldLCD::pollInputs(Robot& robot)
{
	if (!alive_) return;

	// Si editField_ pointe sur COLOR mais qu'on est en ARMED, deplacer.
	if (editField_ == FIELD_COLOR && robot.phase() >= PHASE_ARMED) {
		editField_ = FIELD_DIAM;
	}

	ButtonTouch b = btns_.checkOneOfAllPressed();

	if (b != BUTTON_NONE) {
		if (lastButton_ != b) {
			holdChrono_.start();
			lastButton_ = b;
			countdownMsg_[0] = '\0';
		}
		// SELECT hold : exit apres 2s
		if (b == BUTTON_BACK_KEY) {
			double elapsed = holdChrono_.getElapsedTimeInSec();
			double remaining = SELECT_EXIT_SEC - elapsed;
			if (remaining <= 0) {
				logger().info() << "[HOLD] SELECT 2s -> exit(0)" << logs::end;
				lcd_.clear();
				lcd_.home();
				lcd_.print("EXIT !");
				std::exit(0);
			}
			if (elapsed >= SELECT_ACTION_MAX) {
				// Zone countdown : affiche le decompte exit
				std::snprintf(countdownMsg_, 17, "EXIT: %3.1fs      ", remaining);
				countdownMsg_[16] = '\0';
			}
		}
	} else {
		// Relachement du bouton
		if (lastButton_ != BUTTON_NONE) {
			if (lastButton_ == BUTTON_BACK_KEY) {
				double elapsed = holdChrono_.getElapsedTimeInSec();
				if (elapsed < SELECT_ACTION_MAX) {
					// Clic court : setPos/reset/test
					handleClick(robot, lastButton_);
				}
				// 0.5s - 2s : zone morte, rien
			} else {
				// Autres boutons : action immediate au relachement
				handleClick(robot, lastButton_);
			}
		}
		lastButton_ = BUTTON_NONE;
		countdownMsg_[0] = '\0';
	}
}

void MenuShieldLCD::refreshDisplay(const Robot& robot)
{
	if (!alive_) return;

	char l0[17];
	buildLine0(robot, l0);

	char l1[17];
	if (countdownMsg_[0] != '\0') {
		std::strncpy(l1, countdownMsg_, 17);
		l1[16] = '\0';
	} else {
		buildLine1(robot, l1);
	}

	if (std::strcmp(l0, lastLine0_) != 0) {
		lcd_.setCursor(0, 0);
		lcd_.print(l0);
		std::strncpy(lastLine0_, l0, 17);
	}
	if (std::strcmp(l1, lastLine1_) != 0) {
		lcd_.setCursor(0, 1);
		lcd_.print(l1);
		std::strncpy(lastLine1_, l1, 17);
	}
}
