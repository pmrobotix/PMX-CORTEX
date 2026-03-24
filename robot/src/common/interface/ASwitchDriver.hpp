/*!
 * \file
 * \brief Interface abstraite du driver de switchs et tirette.
 *
 * Gere la tirette de depart de match, les capteurs de contact
 * arriere et les sorties GPIO generiques.
 */

#ifndef ASWITCHDRIVER_HPP_
#define ASWITCHDRIVER_HPP_

#include <string>

/*!
 * \brief Interface abstraite du driver de switchs.
 *
 * Fournit la lecture de la tirette de depart, des switchs arriere
 * et le controle de sorties GPIO. L'implementation ARM lit les GPIO
 * hardware, la SIMU simule les etats.
 */
class ASwitchDriver
{

public:

	/*!
	 * \brief Cree l'instance concrete selon la plateforme.
	 * \param botName Nom du robot.
	 * \return Pointeur vers l'instance creee.
	 */
	static ASwitchDriver * create(std::string botName);

	/*!
	 * \brief Verifie si le module est connecte.
	 * \return true si la communication est etablie.
	 */
	virtual bool is_connected() = 0;

	/*!
	 * \brief Lit l'etat de la tirette de depart.
	 * \return 1 si la tirette est en place, 0 si retiree.
	 */
	virtual int tirettePressed() = 0;

	/*!
	 * \brief Lit le switch arriere gauche.
	 * \return 1 si appuye, 0 sinon.
	 */
	virtual int backLeftPressed() = 0;

	/*!
	 * \brief Lit le switch arriere droit.
	 * \return 1 si appuye, 0 sinon.
	 */
	virtual int backRightPressed() = 0;

	/*!
	 * \brief Active ou desactive une sortie GPIO.
	 * \param gpio Numero du GPIO.
	 * \param activate true pour activer, false pour desactiver.
	 */
	virtual void setGPIO(int gpio, bool activate) = 0;

	virtual ~ASwitchDriver()
	{
	}

protected:

	ASwitchDriver()
	{
	}

};

#endif
