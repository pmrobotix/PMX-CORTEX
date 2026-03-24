/*!
 * \file
 * \brief Utilitaires pour la manipulation de chemins de fichiers.
 */

#ifndef COMMON_UTILS_FILE_HPP_
#define COMMON_UTILS_FILE_HPP_

#include <string>

/*!
 * \brief Extrait l'extension d'un chemin de fichier.
 * \param pathName Chemin du fichier (ex: "image.png").
 * \return L'extension sans le point (ex: "png").
 */
std::string getExt(std::string pathName)
{
	// Finds the last persiod character of the string
	int period = pathName.find_last_of(".");
	// I use  + 1 because I don't really need the to include the period
	std::string ext = pathName.substr(period + 1);
	return ext;
}



#endif /* COMMON_UTILS_FILE_HPP_ */
