/*!
 * \file
 * \brief Interface abstraite du driver de son.
 *
 * Gere la sortie audio du robot : beep, tonalites, lecture de fichiers
 * audio et synthese vocale.
 */

#ifndef ASOUNDDRIVER_HPP_
#define ASOUNDDRIVER_HPP_

#include <string>
#include <vector>

/*!
 * \brief Interface abstraite du driver de son.
 *
 * Fournit les commandes audio (beep, tone, play, speak).
 * Chaque commande peut etre synchrone (bloquante) ou asynchrone.
 */
class ASoundDriver
{

public:

	/*!
	 * \brief Cree l'instance concrete selon la plateforme.
	 * \return Pointeur vers l'instance creee.
	 */
	static ASoundDriver * create();

	/*!
	 * \brief Emet un beep avec les arguments donnes.
	 * \param args Arguments du beep (format dependant de l'implementation).
	 * \param bSynchronous true pour bloquer jusqu'a la fin du son.
	 */
	virtual void beep(const std::string &args, bool bSynchronous = true) = 0;

	/*!
	 * \brief Joue une tonalite a une frequence donnee.
	 * \param frequency Frequence en Hz.
	 * \param ms Duree en millisecondes.
	 * \param bSynchronous true pour bloquer jusqu'a la fin.
	 */
	virtual void tone(unsigned frequency, unsigned ms, bool bSynchronous = true) = 0;

	/*!
	 * \brief Joue une sequence de tonalites.
	 * \param sequence Vecteur de [frequence, duree, ...] pour chaque note.
	 * \param bSynchronous true pour bloquer jusqu'a la fin.
	 */
	virtual void tone(const std::vector<std::vector<float> > &sequence, bool bSynchronous = true) = 0;

	/*!
	 * \brief Joue un fichier audio.
	 * \param soundfile Chemin vers le fichier audio.
	 * \param bSynchronous true pour bloquer jusqu'a la fin.
	 */
	virtual void play(const std::string &soundfile, bool bSynchronous = true) = 0;

	/*!
	 * \brief Synthese vocale (text-to-speech).
	 * \param text Texte a prononcer.
	 * \param bSynchronous true pour bloquer jusqu'a la fin.
	 */
	virtual void speak(const std::string &text, bool bSynchronous = true) = 0;

	virtual ~ASoundDriver()
	{
	}

protected:

	ASoundDriver()
	{
	}

};

#endif
