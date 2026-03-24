/*!
 * \file
 * \brief Classe utilitaire pour la saisie clavier en mode console.
 */

#ifndef COMMON_CONSOLEKEYINPUT_HPP_
#define COMMON_CONSOLEKEYINPUT_HPP_

#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>

/*!
 * \brief Classe utilitaire pour la gestion du clavier et du curseur console.
 */
class ConsoleKeyInput
{
public:

	/*!
	 * \brief Lit un caractère depuis le terminal sans echo ni attente de retour chariot.
	 * \return Le caractère lu.
	 */
	static char mygetch()
	{
		/*#include <unistd.h>   //_getch*/
		/*#include <termios.h>  //_getch*/
		char buf = 0;
		struct termios old ;//= {0}
		fflush(stdout);
		if (tcgetattr(0, &old) < 0)
			perror("tcsetattr()");
		old.c_lflag &= ~ICANON;
		old.c_lflag &= ~ECHO;
		old.c_cc[VMIN] = 1;
		old.c_cc[VTIME] = 0;
		if (tcsetattr(0, TCSANOW, &old) < 0)
			perror("tcsetattr ICANON");
		if (read(0, &buf, 1) < 0)
			perror("read()");
		old.c_lflag |= ICANON;
		old.c_lflag |= ECHO;
		if (tcsetattr(0, TCSADRAIN, &old) < 0)
			perror("tcsetattr ~ICANON");
		//printf("%d\n", buf);
		return buf;
	}

	/*!
	 * \brief Efface l'écran du terminal.
	 */
	static void clearScreen()
	{
		std::cout << "\033[2J\033[1;1H";
	}

	/*!
	 * \brief Déplace le curseur à une position donnée (ligne, colonne). Le coin supérieur gauche est (1,1).
	 * \param row Numéro de ligne.
	 * \param col Numéro de colonne.
	 */
	static void setPrintPos(int row, int col)
	{
		printf("\033[%d;%dH", row, col);
	}

	/*!
	 * \brief Positionne le curseur dans un flux de sortie via des sauts de ligne et espaces.
	 * \param _os Flux de sortie.
	 * \param _x Nombre d'espaces horizontaux.
	 * \param _y Nombre de sauts de ligne verticaux.
	 */
	static void setPos(std::ostream& _os, const std::streamsize& _x, const std::streamsize& _y)
	{
		char tmp = _os.fill();

		if (_y > 0)
		{
			_os.fill('\n');
			_os.width(_y);
			_os << '\n';
		}
		if (_x > 0)
		{
			_os.fill(' ');
			_os.width(_x);
			_os << ' ';
		}
		_os.flush();
		_os.fill(tmp);
	}

	/*!
	 * \brief Convertit un entier positif en chaîne de caractères (base 10).
	 * \param a Entier à convertir (doit être > 0).
	 * \return La représentation en chaîne.
	 */
	static std::string itoa(int a)
	{
		std::string ss = "";   //create empty string
		while (a)
		{
			int x = a % 10;
			a /= 10;
			char i = '0';
			i = i + x;
			ss = i + ss;      //append new character at the front of the string!
			std::this_thread::yield();
		}
		return ss;
	}

};

#endif /* COMMON_CONSOLEKEYINPUT_HPP_ */
