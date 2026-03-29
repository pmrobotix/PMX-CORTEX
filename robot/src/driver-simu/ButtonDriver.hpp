/*!
 * \file
 * \brief Implementation SIMU du driver de boutons.
 *
 * Simule les boutons physiques via des messages IPC System V.
 * Un processus externe envoie les appuis clavier dans une file
 * de messages, et un thread interne les lit en continu.
 */

#ifndef SIMU_BUTTONDRIVER_HPP_
#define SIMU_BUTTONDRIVER_HPP_

#include <sys/ipc.h>
#include <sys/msg.h>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <thread>

#include "interface/AButtonDriver.hpp"
#include "log/LoggerFactory.hpp"

/*!
 * \brief Structure de message IPC pour la file de messages.
 */
struct msgform
{
	long mtype;
	char mtext[512];
};

/*!
 * \brief Implementation simulation du driver de boutons.
 *
 * Utilise une file de messages System V (IPC) pour recevoir
 * les appuis clavier depuis un processus externe (console separee).
 */
class ButtonDriverSimu: public AButtonDriver
{
private:

	static inline const logs::Logger & logger()
	{
		static const logs::Logger & instance = logs::LoggerFactory::logger("ButtonDriver.SIMU");
		return instance;
	}

	int lindex;
	std::thread tbutton_;  ///< Thread de lecture IPC.

public:

	bool back_;    ///< Etat du bouton retour.
	bool enter_;   ///< Etat du bouton entree.
	bool up_;      ///< Etat du bouton haut.
	bool down_;    ///< Etat du bouton bas.
	bool left_;    ///< Etat du bouton gauche.
	bool right_;   ///< Etat du bouton droite.

	bool stop_;             ///< Flag d'arret du thread.
	int thread_created_;    ///< 1 si le thread IPC est demarre.

	/*!
	 * \brief Constructeur. Initialise les flags des boutons.
	 */
	ButtonDriverSimu();

	/*!
	 * \brief Destructeur. Arrete le thread IPC et attend sa fin.
	 */
	~ButtonDriverSimu();

	bool pressed(ButtonTouch button) override;
};

/*!
 * \brief Wrapper pour le thread de lecture IPC des boutons.
 *
 * Lit les messages IPC ("enter", "up", "down", etc.) et met
 * a jour les flags du ButtonDriver en continu.
 */
class ButtonDriverWrapper
{
public:
	ButtonDriverWrapper(ButtonDriverSimu * buttondriver)
	{
		buttondriver_ = buttondriver;
	}
	~ButtonDriverWrapper()
	{
	}

	ButtonDriverSimu * buttondriver_;

	/*!
	 * \brief Boucle de lecture des messages IPC.
	 * \param arg1 Nom du thread.
	 */
	void checkButton(const char *arg1, int)
	{
		int res;
		int frequete;
		int CLEF_REQUETES = 0x00012345;
		struct msgform msg;

		frequete = msgget(CLEF_REQUETES, 0700 | IPC_CREAT);
		if (frequete == -1) {
			perror("checkButton() msgget");
			return;
		}

		while (!buttondriver_->stop_) {

			buttondriver_->enter_ = false;
			buttondriver_->back_ = false;
			buttondriver_->up_ = false;
			buttondriver_->down_ = false;
			buttondriver_->left_ = false;
			buttondriver_->right_ = false;

			res = msgrcv(frequete, &msg, 512, 0, 0);
			if (res == -1) {
				perror("checkButton() msgrcv");
				return;
			} else {
				std::string str(msg.mtext);
				if (str == "enter")
					buttondriver_->enter_ = true;
				if (str == "back")
					buttondriver_->back_ = true;
				if (str == "up")
					buttondriver_->up_ = true;
				if (str == "down")
					buttondriver_->down_ = true;
				if (str == "right")
					buttondriver_->right_ = true;
				if (str == "left")
					buttondriver_->left_ = true;
			}
			usleep(200000);
		}
	}

	/*!
	 * \brief Demarre le thread de lecture IPC.
	 * \param arg1 Nom du thread.
	 * \param args Non utilise.
	 * \return Le thread cree.
	 */
	std::thread buttonThread(const char *arg1, int args)
	{
		return std::thread([=]
		{   this->checkButton(arg1, args);});
	}
};

#endif
