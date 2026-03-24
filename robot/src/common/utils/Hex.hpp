/*!
 * \file
 * \brief Utilitaires pour l'affichage binaire et hexadécimal de valeurs.
 *
 * Exemples d'utilisation :
 * \code
 * // Affichage binaire 8 digits : cout << std::bitset<8>(number) << endl;
 * // Affichage hexa 4 digits : cout << setfill('0') << setw(4) << std::hex << val << endl;
 * // Affichage binaire via bin() :
 * //   cout << "bin="; bin(value, cout); cout << endl;
 * \endcode
 */

#ifndef COMMON_HEX_HPP_
#define COMMON_HEX_HPP_

#include <iostream>

/*!
 * \brief Retourne le bit de poids fort d'un type entier.
 * \param t Référence sur la variable (modifiée).
 * \return Le bit de poids fort.
 */
template<typename T>
inline T highbit(T& t)
{
	return t = (((T) (-1)) >> 1) + 1;
}

/*!
 * \brief Affiche la représentation binaire d'une valeur sur un flux de sortie.
 * \param value Valeur à afficher.
 * \param o Flux de sortie (ex: std::cout).
 * \return Le flux de sortie.
 */
template<typename T>
std::ostream& bin(T& value, std::ostream &o)
{
	for (T bit = highbit(bit); bit; bit >>= 1)
	{
		o << ((value & bit) ? '1' : '0');
	}
	return o;
}

#endif
