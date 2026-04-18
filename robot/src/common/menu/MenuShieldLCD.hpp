/*!
 * \file
 * \brief Source de menu basee sur le shield LCD 2x16 + boutons.
 *
 * Voir robot/md/O_STATE_NEW_INIT.md section 5 pour la specification complete.
 *
 * Layout 2x16 toujours visible :
 *   Ligne 0:  Y* S2  D:40cm
 *   Ligne 1:  >COLOR: YELLOW
 *
 * Cycle des rubriques :
 *   PHASE_CONFIG :  COLOR -> DIAM -> STRAT -> TEST -> (COLOR)
 *   PHASE_ARMED  :  DIAM -> STRAT -> TEST -> (DIAM)   (COLOR retiree du cycle)
 *
 * Boutons :
 *   LEFT   click   = rubrique precedente
 *   RIGHT  click   = rubrique suivante
 *   UP     click   = valeur "haut" (COLOR:BLUE, DIAM:+5, STRAT:next, TEST:T+1)
 *   DOWN   click   = valeur "bas"  (COLOR:YELLOW, DIAM:-5, STRAT:prev, TEST:T-1)
 *   SELECT click   = contextuel :
 *                     - curseur sur TEST : declenche le test selectionne
 *                     - sinon : setPos (CONFIG) ou reset (ARMED)
 *
 *   Note : le shield n'a que 5 boutons physiques (UP/DOWN/LEFT/RIGHT/SELECT).
 *   SELECT = BUTTON_BACK_KEY dans le driver. BUTTON_ENTER_KEY n'existe pas.
 */

#ifndef COMMON_MENU_MENUSHIELDLCD_HPP_
#define COMMON_MENU_MENUSHIELDLCD_HPP_

#include <cstdint>

#include "IMenuSource.hpp"
#include "interface/AButtonDriver.hpp"
#include "log/LoggerFactory.hpp"
#include "utils/Chronometer.hpp"

class LcdShield;
class ButtonBar;
class Robot;

class MenuShieldLCD : public IMenuSource
{
public:
	/// Champs editables via le curseur. L'ordre definit le cycle LEFT/RIGHT.
	/// COLOR est present en CONFIG seulement, masque en ARMED.
	enum EditField {
		FIELD_COLOR = 0,   ///< Present en CONFIG, skippe en ARMED.
		FIELD_DIAM,
		FIELD_STRAT,
		FIELD_TEST,
		FIELD_COUNT        ///< Sentinelle.
	};

private:
	static inline const logs::Logger& logger()
	{
		static const logs::Logger &instance = logs::LoggerFactory::logger("MenuShieldLCD");
		return instance;
	}

	LcdShield& lcd_;
	ButtonBar& btns_;
	bool alive_ = true;

	ButtonTouch lastButton_ = BUTTON_NONE;
	utils::Chronometer holdChrono_;       ///< Chrono pour hold SELECT (exit).

	EditField editField_ = FIELD_COLOR;  ///< Curseur, defaut sur COLOR pour selection rapide au boot.
	uint8_t testSelected_ = 1;           ///< T1..T5 dans la page TEST.

	// Cache pour eviter les flicker LCD.
	char lastLine0_[17] = {0};
	char lastLine1_[17] = {0};
	char countdownMsg_[17] = {0};        ///< Message countdown exit pendant hold SELECT.

public:
	MenuShieldLCD(LcdShield& lcd, ButtonBar& btns);
	~MenuShieldLCD() override = default;

	void pollInputs(Robot& robot) override;
	void refreshDisplay(const Robot& robot) override;
	bool isAlive() const override { return alive_; }
	const char* name() const override { return "MenuShieldLCD"; }

private:
	void buildLine0(const Robot& robot, char out[17]) const;
	void buildLine1(const Robot& robot, char out[17]) const;

	static const char* strategyShort(const char* longName);

	// Navigation curseur selon la phase (saute COLOR en ARMED).
	void nextField(const Robot& robot);
	void prevField(const Robot& robot);

	// Edition valeur du champ courant.
	void incrementCurrentField(Robot& robot);
	void decrementCurrentField(Robot& robot);

	void handleClick(Robot& robot, ButtonTouch b);
};

#endif
