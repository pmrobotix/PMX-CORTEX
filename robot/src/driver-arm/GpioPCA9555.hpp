/*!
 * \file
 * \brief Driver I2C pour le GPIO expander PCA9555.
 *
 * Gere un module PCA9555 (DFR0013) connecte en I2C :
 * Port0 en sortie (relais, GPIO), Port1 en entree (switchs, tirette).
 */

#ifndef OPOS6UL_GPIO_PCA9555_HPP
#define OPOS6UL_GPIO_PCA9555_HPP

#include <as_devices/cpp/as_i2c.hpp>

#include "log/LoggerFactory.hpp"

/// Adresse I2C du PCA9555 (DFR0013, jumpers A0=0, A1=0, A2=1).
#define GPIOBOARD_PCA9555		 0x24

// Registres PCA9555
#define IN_P0 			0x00 ///< Lecture entrees port0.
#define IN_P1 			0x01 ///< Lecture entrees port1.
#define OUT_P0 			0x02 ///< Ecriture sorties port0.
#define OUT_P1 			0x03 ///< Ecriture sorties port1.
#define INV_P0 			0x04 ///< Inversion polarite port0.
#define INV_P1 			0x05 ///< Inversion polarite port1.
#define CONFIG_P0 		0x06 ///< Configuration direction port0 (0=output, 1=input).
#define CONFIG_P1 		0x07 ///< Configuration direction port1 (0=output, 1=input).

#define PAUSE 200000 ///< Delai d'initialisation en microsecondes.

/*!
 * \brief Driver singleton pour le GPIO expander PCA9555 via I2C.
 *
 * Port0 = sorties (setOnP0/setOffP0), Port1 = entrees (getValueP1).
 * Utilise pour la tirette et les switchs arriere sur l'OPOS6UL.
 */
class GpioPCA9555
{
private:

	static const logs::Logger & logger()
	{
		static const logs::Logger & instance = logs::LoggerFactory::logger("GpioPCA9555.OPO");
		return instance;
	}

	AsI2c i2cGB_;           ///< Bus I2C.
	bool connected_;        ///< true si le PCA9555 est detecte.
	int port0Value_;        ///< Valeur courante du port0 (sorties).
	int port1Value_;        ///< Valeur courante du port1 (entrees).

	/*!
	 * \brief Constructeur prive (singleton).
	 */
	GpioPCA9555();

	/*!
	 * \brief Ecrit un octet dans un registre I2C.
	 * \param command Adresse du registre.
	 * \param value Valeur a ecrire.
	 * \return 0 si succes, <0 si erreur.
	 */
	long write_i2c(unsigned char command, unsigned char value);

	/*!
	 * \brief Lit un octet depuis un registre I2C.
	 * \param command Adresse du registre.
	 * \return Valeur lue ou <0 si erreur.
	 */
	long read_i2c(unsigned char command);

public:

	/*!
	 * \brief Retourne l'instance unique (singleton).
	 * \return Reference vers l'instance.
	 */
	static GpioPCA9555 & instance()
	{
		static GpioPCA9555 instance;
		return instance;
	}

	virtual inline ~GpioPCA9555()
	{
	}

	/*!
	 * \brief Initialise la communication I2C et configure les ports.
	 * \return true si le PCA9555 est detecte et configure.
	 */
	bool begin();

	/*!
	 * \brief Verifie si le PCA9555 est connecte.
	 * \return true si connecte.
	 */
	bool isConnected()
	{
		return connected_;
	}

	/*!
	 * \brief Lit la valeur d'un pin du port1 (entrees).
	 * \param pin Numero du pin (0-7).
	 * \return 0 ou 1, -1 si non connecte.
	 */
	int getValueP1(int pin);

	/*!
	 * \brief Ecrit une valeur sur un pin du port0 (sorties).
	 * \param port Registre de sortie (OUT_P0).
	 * \param pin Numero du pin (0-7).
	 * \param value 0 ou 1.
	 */
	void setValueP0(int port, int pin, int value);

	/*!
	 * \brief Active un pin du port0.
	 * \param pin Numero du pin (0-7).
	 */
	void setOnP0(int pin);

	/*!
	 * \brief Desactive un pin du port0.
	 * \param pin Numero du pin (0-7).
	 */
	void setOffP0(int pin);

	/*!
	 * \brief Configure les registres de direction des ports.
	 * \return true si succes.
	 */
	bool setup();
};

#endif
