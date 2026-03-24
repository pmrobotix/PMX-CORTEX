/*!
 * \file
 * \brief Interface abstraite du driver de LEDs.
 *
 * Gere une barre de LEDs multicolores pour l'affichage
 * d'etat du robot (couleur d'equipe, debug, animations).
 */

#ifndef ALEDDRIVER_HPP_
#define ALEDDRIVER_HPP_

#include <sys/types.h>
#include <string>

/*!
 * \brief Couleurs disponibles pour les LEDs.
 */
enum LedColor
{
	LED_BLACK,   ///< LED eteinte.
	LED_GREEN,   ///< Vert.
	LED_RED,     ///< Rouge.
	LED_ORANGE,  ///< Orange.
	LED_AMBER,   ///< Ambre.
	LED_YELLOW,  ///< Jaune.
	LED_OFF      ///< Sentinelle de fin d'enumeration.
};

/*!
 * \brief Interface abstraite du driver de LEDs.
 *
 * Fournit le controle d'une barre de LEDs (setBit pour une LED
 * individuelle, setBytes pour toute la barre). L'implementation ARM
 * pilote les LEDs hardware, la SIMU est un stub.
 */
class ALedDriver
{

public:

	/*!
	 * \brief Cree l'instance concrete selon la plateforme.
	 * \param botName Nom du robot (utilise par le simulateur).
	 * \param nb Nombre de LEDs dans la barre.
	 * \return Pointeur vers l'instance creee.
	 */
	static ALedDriver * create(std::string botName, int nb);

	/*!
	 * \brief Allume une LED a une position donnee.
	 * \param index Position de la LED dans la barre (0 a nb-1).
	 * \param color Couleur a appliquer.
	 */
	virtual void setBit(int index, LedColor color) = 0;

	/*!
	 * \brief Definit l'etat de toute la barre par masque binaire.
	 *
	 * Chaque bit a 1 dans \p hex allume la LED correspondante
	 * avec la couleur donnee. Les bits a 0 eteignent la LED.
	 *
	 * \param hex Masque binaire des positions.
	 * \param color Couleur des LEDs allumees.
	 */
	virtual void setBytes(uint hex, LedColor color) = 0;

	/*!
	 * \brief Lit la couleur d'une LED a une position donnee.
	 *
	 * Utile pour les tests unitaires. En ARM, retourne -1
	 * car il n'y a pas de lecture hardware sur les GPIO.
	 *
	 * \param index Position de la LED dans la barre (0 a nb-1).
	 * \return Couleur de la LED (LedColor) ou -1 si non disponible.
	 */
	virtual int getBit(int index) = 0;

	virtual ~ALedDriver()
	{
	}

protected:

	ALedDriver()
	{
	}

};

#endif
