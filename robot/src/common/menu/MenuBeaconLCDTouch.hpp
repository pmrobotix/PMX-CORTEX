/*!
 * \file
 * \brief Source de menu basee sur le LCD tactile de la balise (Teensy, I2C 0x2D).
 *
 * Voir robot/md/O_STATE_NEW_INIT.md section 6 pour la specification.
 *
 * Deux responsabilites :
 *   - pollInputs : lit Settings Teensy via Sensors::readMatchSettings(). Compare
 *     avec le shadow pour detecter les modifications du touch (delta detection
 *     avant l'etape 7 de migration qui introduira seq_touch). Applique les
 *     changements dans Robot via les setters.
 *   - refreshDisplay : push l'etat Robot vers Settings Teensy via les writeXxx
 *     de Sensors. Met a jour le shadow en consequence.
 *
 * Tant que l'etape 7 migration Teensy (ajout seq_touch cote TofSensors.h) n'est
 * pas faite, la detection de reboot est basee sur la comparaison des champs :
 * si tous les champs lus correspondent a ce qu'on a ecrit la derniere fois,
 * pas de delta. Si un champ diverge, c'est soit un clic touch, soit un reboot
 * Teensy (defaults). Dans les 2 cas on adopte la valeur, quitte a perdre un
 * parametre si la Teensy a reboote en plein match (tres rare).
 */

#ifndef COMMON_MENU_MENUBEACONLCDTOUCH_HPP_
#define COMMON_MENU_MENUBEACONLCDTOUCH_HPP_

#include <cstdint>

#include "IMenuSource.hpp"
#include "interface/ASensorsDriver.hpp"
#include "log/LoggerFactory.hpp"

class Sensors;
class Robot;

class MenuBeaconLCDTouch : public IMenuSource
{
private:
	static inline const logs::Logger& logger()
	{
		static const logs::Logger &instance = logs::LoggerFactory::logger("MenuBeaconLCDTouch");
		return instance;
	}

	Sensors& sensors_;              ///< Pour readMatchSettings / writeXxx
	MatchSettingsData shadow_{};    ///< Dernier snapshot connu (ecrit par nous ou lu)
	bool shadowInit_ = false;       ///< shadow_ n'est utilisable qu'apres le 1er read OK
	/// Mis a jour sur chaque readMatchSettings. Init a true (optimiste) sinon
	/// le controller ne nous appelle jamais et on ne peut pas se reveiller.
	bool alive_ = true;
	uint8_t lastSeqTouchSeen_ = 0;  ///< Utilise si seq_touch actif (etape 7 migration)

public:
	explicit MenuBeaconLCDTouch(Sensors& sensors);
	~MenuBeaconLCDTouch() override = default;

	void pollInputs(Robot& robot) override;
	void refreshDisplay(const Robot& robot) override;
	bool isAlive() const override { return alive_; }
	const char* name() const override { return "MenuBeaconLCDTouch"; }
};

#endif
