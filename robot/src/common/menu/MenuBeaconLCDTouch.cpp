/*!
 * \file
 * \brief Implementation de MenuBeaconLCDTouch (nouveau modele a 3 phases).
 *
 * Voir robot/md/O_STATE_NEW_INIT.md section 6.
 *
 * Ce plugin :
 *   - pollInputs : lit Settings via I2C, detecte les clics touch via seq_touch,
 *     applique les modifs dans Robot. Consomme actionReq (1=SETPOS/RESET).
 *   - refreshDisplay : pousse Robot -> Settings vers Teensy (matchColor,
 *     strategy, advDiameter, ledLuminosity, matchState).
 */

#include "MenuBeaconLCDTouch.hpp"

#include "log/Logger.hpp"
#include "action/Sensors.hpp"
#include "Robot.hpp"

MenuBeaconLCDTouch::MenuBeaconLCDTouch(Sensors& sensors) : sensors_(sensors)
{
	logger().info() << "MenuBeaconLCDTouch ctor" << logs::end;
}

void MenuBeaconLCDTouch::pollInputs(Robot& robot)
{
	MatchSettingsData current;
	if (!sensors_.readMatchSettings(current)) {
		if (alive_) {
			logger().warn() << "[POLL] readMatchSettings FAILED (I2C err), alive_=false" << logs::end;
		}
		alive_ = false;
		return;
	}
	if (!alive_) {
		logger().info() << "[POLL] beacon back alive" << logs::end;
	}
	alive_ = true;

	// Premier read : initialise shadow et seq_touch. On ADOPTE les valeurs
	// Teensy dans Robot pour gerer le cas ou l'operateur a configure le touch
	// AVANT le boot OPOS6UL. En PHASE_CONFIG, tous les setters acceptent.
	// Si Teensy a des defaults (personne n'a touche), les valeurs sont identiques
	// aux defaults Robot -> pas d'effet visible.
	if (!shadowInit_) {
		shadow_ = current;
		shadow_.actionReq = 0;  // actionReq d'avant le boot OPOS6UL est stale, on l'ignore
		lastSeqTouchSeen_ = current.seq_touch;
		shadowInit_ = true;
		logger().info() << "[POLL] first read shadow init: seq_touch=" << (int)current.seq_touch
				<< " mColor=" << (int)current.matchColor
				<< " strat=" << (int)current.strategy
				<< " diam=" << (int)current.advDiameter
				<< " led=" << (int)current.ledLuminosity
				<< " state=" << (int)current.matchState
				<< " actionReq=" << (int)current.actionReq
				<< logs::end;

		// Adoption des valeurs Teensy dans Robot (ordre de boot indifferent)
		RobotColor c = (current.matchColor == 1) ? PMXYELLOW : PMXBLUE;
		robot.setMyColorChecked(c);
		switch (current.strategy) {
			case 1: robot.setStrategyChecked("PMX1"); break;
			case 2: robot.setStrategyChecked("PMX2"); break;
			case 3: robot.setStrategyChecked("PMX3"); break;
			default: break;
		}
		if (current.advDiameter >= 5 && current.advDiameter <= 250)
			robot.setAdvDiameter(current.advDiameter);
		if (current.ledLuminosity >= 0 && current.ledLuminosity <= 100)
			robot.setLedLuminosity(static_cast<uint8_t>(current.ledLuminosity));

		// Adoption des 8 zones de prise (index 0..5 par zone, defaut 0=BBYY).
		robot.setPickupP1(current.pickup_P1);
		robot.setPickupP2(current.pickup_P2);
		robot.setPickupP3(current.pickup_P3);
		robot.setPickupP4(current.pickup_P4);
		robot.setPickupP11(current.pickup_P11);
		robot.setPickupP12(current.pickup_P12);
		robot.setPickupP13(current.pickup_P13);
		robot.setPickupP14(current.pickup_P14);

		// Si actionReq etait a 1 au boot (stale), le consommer cote Teensy
		if (current.actionReq != 0) {
			sensors_.writeActionReq(0);
			logger().info() << "[POLL] cleared stale actionReq=" << (int)current.actionReq << logs::end;
		}

		logger().info() << "[POLL] adopted Teensy values into Robot: color="
				<< (c == PMXYELLOW ? "YELLOW" : "BLUE")
				<< " strat=" << (int)current.strategy
				<< " diam=" << (int)current.advDiameter
				<< " led=" << (int)current.ledLuminosity
				<< " pickup=[" << (int)current.pickup_P1 << "," << (int)current.pickup_P2
				<< "," << (int)current.pickup_P3  << "," << (int)current.pickup_P4
				<< "," << (int)current.pickup_P11 << "," << (int)current.pickup_P12
				<< "," << (int)current.pickup_P13 << "," << (int)current.pickup_P14 << "]"
				<< logs::end;
		return;
	}

	// --- Detection seq_touch (fiable) ---
	bool seqChanged = false;
	const bool seqActive = (current.seq_touch != 0) || (lastSeqTouchSeen_ != 0);
	if (seqActive) {
		if (current.seq_touch < lastSeqTouchSeen_) {
			logger().warn() << "[POLL] seq_touch regression ("
					<< (int)lastSeqTouchSeen_ << " -> " << (int)current.seq_touch
					<< ") : Teensy reboot detected" << logs::end;
			lastSeqTouchSeen_ = current.seq_touch;
			return;
		}
		if (current.seq_touch == lastSeqTouchSeen_) {
			return;  // aucune modif touch depuis dernier poll
		}
		seqChanged = true;
		logger().info() << "[POLL] seq_touch " << (int)lastSeqTouchSeen_
				<< " -> " << (int)current.seq_touch
				<< " : NEW TOUCH CLICK detected" << logs::end;
		lastSeqTouchSeen_ = current.seq_touch;
	}

	// --- Adoption des champs modifies (fallback delta vs shadow si seq inactif) ---

	// matchColor : 0=BLUE, 1=YELLOW
	if (current.matchColor != shadow_.matchColor) {
		RobotColor newColor = (current.matchColor == 1) ? PMXYELLOW : PMXBLUE;
		bool ok = robot.setMyColorChecked(newColor);
		logger().info() << "[POLL] delta matchColor " << (int)shadow_.matchColor
				<< " -> " << (int)current.matchColor
				<< " : setMyColorChecked(" << (newColor == PMXYELLOW ? "YELLOW" : "BLUE")
				<< ") ok=" << ok
				<< " (phase=" << (int)robot.phase() << ")" << logs::end;
		shadow_.matchColor = current.matchColor;
	}

	// strategy : 1/2/3 -> "PMX1"/"PMX2"/"PMX3"
	if (current.strategy != shadow_.strategy) {
		const char* name = nullptr;
		switch (current.strategy) {
			case 1: name = "PMX1"; break;
			case 2: name = "PMX2"; break;
			case 3: name = "PMX3"; break;
			default: break;
		}
		if (name) {
			bool ok = robot.setStrategyChecked(name);
			logger().info() << "[POLL] delta strategy " << (int)shadow_.strategy
					<< " -> " << (int)current.strategy
					<< " : setStrategy(" << name << ") ok=" << ok << logs::end;
		} else {
			logger().info() << "[POLL] delta strategy " << (int)shadow_.strategy
					<< " -> " << (int)current.strategy
					<< " : IGNORED (invalid value)" << logs::end;
		}
		shadow_.strategy = current.strategy;
	}

	// advDiameter
	if (current.advDiameter != shadow_.advDiameter) {
		bool ok = robot.setAdvDiameter(current.advDiameter);
		logger().info() << "[POLL] delta advDiameter " << (int)shadow_.advDiameter
				<< " -> " << (int)current.advDiameter
				<< " : setAdvDiameter ok=" << ok << logs::end;
		shadow_.advDiameter = current.advDiameter;
	}

	// ledLuminosity
	if (current.ledLuminosity != shadow_.ledLuminosity) {
		bool ok = robot.setLedLuminosity(static_cast<uint8_t>(current.ledLuminosity));
		logger().info() << "[POLL] delta ledLuminosity " << (int)shadow_.ledLuminosity
				<< " -> " << (int)current.ledLuminosity
				<< " : setLedLuminosity ok=" << ok << logs::end;
		shadow_.ledLuminosity = current.ledLuminosity;
	}

	// Zones de prise : 8 bytes pickup_Pn (index 0..5), simple delta vs shadow.
	// Pas d'invalidation a > 5 cote Robot (les setters bornent deja).
	#define POLL_DELTA_PICKUP(field, setter) \
		if (current.field != shadow_.field) { \
			bool ok = robot.setter(current.field); \
			logger().info() << "[POLL] delta " #field " " << (int)shadow_.field \
					<< " -> " << (int)current.field << " ok=" << ok << logs::end; \
			shadow_.field = current.field; \
		}
	POLL_DELTA_PICKUP(pickup_P1,  setPickupP1)
	POLL_DELTA_PICKUP(pickup_P2,  setPickupP2)
	POLL_DELTA_PICKUP(pickup_P3,  setPickupP3)
	POLL_DELTA_PICKUP(pickup_P4,  setPickupP4)
	POLL_DELTA_PICKUP(pickup_P11, setPickupP11)
	POLL_DELTA_PICKUP(pickup_P12, setPickupP12)
	POLL_DELTA_PICKUP(pickup_P13, setPickupP13)
	POLL_DELTA_PICKUP(pickup_P14, setPickupP14)
	#undef POLL_DELTA_PICKUP

	// testMode : one-shot, declenche si != 0
	if (current.testMode != shadow_.testMode) {
		if (current.testMode != 0) {
			robot.triggerTestMode(current.testMode);
			logger().info() << "[POLL] delta testMode " << (int)shadow_.testMode
					<< " -> " << (int)current.testMode
					<< " : triggerTestMode" << logs::end;
		}
		shadow_.testMode = current.testMode;
	}

	// actionReq : bouton SETPOS / RESET (sens selon phase)
	// Protection contre les lectures I2C corrompues : la Teensy fait toujours
	// actionReq=1 ET seq_touch++ ensemble. Si actionReq=1 mais seq_touch n'avait
	// pas change lors de la detection (prevSeqTouch), c'est un bit flip I2C.
	if (current.actionReq != 0 && shadow_.actionReq == 0) {
		if (!seqChanged) {
			logger().warn() << "[POLL] actionReq=1 but seq_touch unchanged ("
					<< (int)current.seq_touch << ") -> I2C glitch, IGNORED" << logs::end;
			shadow_.actionReq = current.actionReq;
			return;
		}
		if (robot.phase() == PHASE_CONFIG) {
			logger().info() << "[POLL] actionReq=1 in CONFIG -> requestSetPos" << logs::end;
			robot.requestSetPos();
		} else if (robot.phase() == PHASE_ARMED) {
			logger().info() << "[POLL] actionReq=1 in ARMED -> requestReset" << logs::end;
			robot.requestReset();
		} else {
			logger().warn() << "[POLL] actionReq=1 but phase=" << (int)robot.phase()
					<< " : IGNORED" << logs::end;
		}
		// Handshake : remettre a 0 cote Teensy
		bool ok = sensors_.writeActionReq(0);
		logger().info() << "[POLL] writeActionReq(0) ok=" << ok << logs::end;
		if (ok) {
			shadow_.actionReq = 0;
		} else {
			shadow_.actionReq = current.actionReq;
		}
	} else {
		shadow_.actionReq = current.actionReq;
	}
}

void MenuBeaconLCDTouch::refreshDisplay(const Robot& robot)
{
	if (!alive_) return;

	// matchColor : Robot -> byte (0=BLUE, 1=YELLOW)
	uint8_t colorByte = robot.isMatchColor() ? 1 : 0;
	if (colorByte != shadow_.matchColor) {
		bool ok = sensors_.writeMatchColor(colorByte);
		logger().info() << "[PUSH] matchColor " << (int)shadow_.matchColor
				<< " -> " << (int)colorByte << " ok=" << ok << logs::end;
		if (ok) {
			shadow_.matchColor = colorByte;
		} else {
			alive_ = false;
			return;
		}
	}

	// strategy : "PMX1"/"PMX2"/"PMX3" -> 1/2/3
	uint8_t stratByte = 0;
	const std::string s = robot.strategy();
	if (s == "PMX1") stratByte = 1;
	else if (s == "PMX2") stratByte = 2;
	else if (s == "PMX3") stratByte = 3;
	if (stratByte != shadow_.strategy) {
		bool ok = sensors_.writeStrategy(stratByte);
		logger().info() << "[PUSH] strategy " << (int)shadow_.strategy
				<< " -> " << (int)stratByte << " (" << s << ") ok=" << ok << logs::end;
		if (ok) {
			shadow_.strategy = stratByte;
		} else {
			alive_ = false;
			return;
		}
	}

	// advDiameter
	if (robot.advDiameter() != shadow_.advDiameter) {
		bool ok = sensors_.writeAdvDiameter(robot.advDiameter());
		logger().info() << "[PUSH] advDiameter " << (int)shadow_.advDiameter
				<< " -> " << (int)robot.advDiameter() << " ok=" << ok << logs::end;
		if (ok) {
			shadow_.advDiameter = robot.advDiameter();
		} else {
			alive_ = false;
			return;
		}
	}

	// ledLuminosity
	if (static_cast<int8_t>(robot.ledLuminosity()) != shadow_.ledLuminosity) {
		logger().info() << "[PUSH] ledLuminosity " << (int)shadow_.ledLuminosity
				<< " -> " << (int)robot.ledLuminosity() << logs::end;
		sensors_.writeLedLuminosity(robot.ledLuminosity());
		shadow_.ledLuminosity = static_cast<int8_t>(robot.ledLuminosity());
	}

	// matchState = phase directement (0=CONFIG, 1=ARMED, 2=MATCH, 3=END)
	uint8_t stateByte = static_cast<uint8_t>(robot.phase());
	if (stateByte != shadow_.matchState) {
		bool ok = sensors_.writeMatchState(stateByte);
		logger().info() << "[PUSH] matchState " << (int)shadow_.matchState
				<< " -> " << (int)stateByte << " ok=" << ok << logs::end;
		if (ok) {
			shadow_.matchState = stateByte;
		} else {
			alive_ = false;
			return;
		}
	}
}
