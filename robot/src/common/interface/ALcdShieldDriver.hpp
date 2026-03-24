/*!
 * \file
 * \brief Interface abstraite du driver d'ecran LCD I2C.
 *
 * Gere un ecran LCD (typiquement 2x16 ou 4x20 caracteres)
 * connecte en I2C, utilise pour l'affichage de debug sur le robot.
 */

#ifndef ALCDSHIELDDRIVER_HPP_
#define ALCDSHIELDDRIVER_HPP_

#include <stddef.h>
#include <cstdint>
#include <string>

/*!
 * \brief Interface abstraite du driver d'ecran LCD.
 *
 * Fournit les commandes de base pour piloter un ecran LCD
 * (clear, cursor, backlight, print). L'implementation ARM
 * communique en I2C, la SIMU est un stub.
 */
class ALcdShieldDriver
{

public:

	/*!
	 * \brief Cree l'instance concrete selon la plateforme.
	 * \param botName Nom du robot (utilise par le simulateur).
	 * \return Pointeur vers l'instance creee.
	 */
	static ALcdShieldDriver * create(std::string botName);

	virtual ~ALcdShieldDriver()
	{
	}

	/*!
	 * \brief Verifie si l'ecran LCD est connecte.
	 * \return true si la communication I2C est etablie.
	 */
	virtual bool is_connected() = 0;

	/*!
	 * \brief Efface tout le contenu de l'ecran.
	 */
	virtual void clear() = 0;

	/*!
	 * \brief Remet le curseur en position (0, 0).
	 */
	virtual void home() = 0;

	/*!
	 * \brief Allume le retro-eclairage.
	 */
	virtual void setBacklightOn() = 0;

	/*!
	 * \brief Eteint le retro-eclairage.
	 */
	virtual void setBacklightOff() = 0;

	/*!
	 * \brief Positionne le curseur.
	 * \param col Colonne (0-based).
	 * \param row Ligne (0-based).
	 */
	virtual void setCursor(uint8_t col, uint8_t row) = 0;

	/*!
	 * \brief Ecrit un octet brut sur l'ecran.
	 * \param value Octet a ecrire.
	 * \return Nombre d'octets ecrits.
	 */
	virtual size_t write(uint8_t value) = 0;

	/*!
	 * \brief Affiche une chaine a une position donnee.
	 * \param str Chaine a afficher.
	 * \param row Ligne (1-based).
	 * \param col Colonne (1-based, defaut 1).
	 */
	virtual void print_content_string(std::string str, int row, int col = 1) = 0;

	/*!
	 * \brief Affiche un entier a une position donnee.
	 * \param value Valeur a afficher.
	 * \param row Ligne (1-based).
	 * \param col Colonne (1-based, defaut 1).
	 */
	virtual void print_content_integer(int value, int row, int col = 1) = 0;

protected:

	ALcdShieldDriver()
	{
	}

};

#endif
