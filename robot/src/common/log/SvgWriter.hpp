/*!
 * \file
 * \brief Définition de la classe SvgWriter.
 */

#ifndef LOG_SVGWRITER_HPP_
#define	LOG_SVGWRITER_HPP_

#include <string>

#include "../utils/PointerList.hpp"
#include "../thread/Mutex.hpp"
#include <simple_svg_1.0.0.hpp>

namespace logs
{
class Logger;
} /* namespace logs */

/*!
 * \brief Wrapper pour la génération de fichier svg via le système de log.
 */
class SvgWriter: public utils::Mutex {
private:

	/*
	 * Id of the associated robot.
	 */
	std::string id_;

	utils::PointerList<std::string> symbol_list_;

protected:
	bool done_;
	bool beginDone_;
	const logs::Logger *fLogger;

	svg::Document *docSensor_;
	const logs::Logger *fLoggerSensors;

public:

	/*!
	 * \brief Constructeur de la classe.
	 *
	 * Ce constructeur est privé pour empécher la création d'une instance
	 * de la classe.
	 */
	SvgWriter(std::string id);

	/*!
	 * \brief Destructeur de la classe.
	 */
	virtual ~SvgWriter();

	/*!
	 * \brief Retourne le logger principal du SVG.
	 * \return Référence sur le logger SVG.
	 */
	inline const logs::Logger& logger()
	{
		return *fLogger;
	}

	/*!
	 * \brief Retourne le logger dédié aux capteurs SVG.
	 * \return Référence sur le logger capteurs.
	 */
	inline const logs::Logger& loggerSvgSensor()
	{
		return *fLoggerSensors;
	}

	/*!
	 * \brief Ajoute un symbole SVG dans la section defs.
	 * \param symbol Contenu SVG du symbole à ajouter.
	 */
	void addDefsSymbol(std::string symbol);

	/*!
	 * \brief Écrit l'en-tête SVG (grille, defs, symboles).
	 */
	void beginHeader();

	/*!
	 * \brief Ferme le document SVG (balises de fin).
	 */
	void endHeader();

	/*!
	 * \brief Écrit un texte à une position donnée dans le SVG.
	 * \param x_mm Position X en mm.
	 * \param y_mm Position Y en mm.
	 * \param text Texte à afficher.
	 */
	void writeText(float x_mm, float y_mm, std::string text);

	/*!
	 * \brief Écrit un texte avec couleur et taille personnalisées.
	 * \param x_mm Position X en mm.
	 * \param y_mm Position Y en mm.
	 * \param text Texte à afficher.
	 * \param color Couleur du texte (ex: "red", "#FF0000").
	 * \param fontsize Taille de la police (ex: "12").
	 */
	void writeTextCustom(float x_mm, float y_mm, std::string text, std::string color, std::string fontsize);

	/*!
	 * \brief Dessine la position du robot sur le SVG.
	 * \param x_mm Position X en mm.
	 * \param y_mm Position Y en mm.
	 * \param a_rad Angle en radians.
	 * \param color Code couleur (0 par défaut).
	 */
	virtual void writePosition_Bot(float x_mm, float y_mm, float a_rad, int color = 0) = 0;

	/*!
	 * \brief Dessine un marqueur de position du robot.
	 * \param x_mm Position X en mm.
	 * \param y_mm Position Y en mm.
	 * \param a_rad Angle en radians.
	 */
	virtual void writePosition_BotPos(float x_mm, float y_mm, float a_rad) = 0;

	/*!
	 * \brief Dessine la position d'un adversaire détecté.
	 * \param x_adv_mm Position X de l'adversaire en mm.
	 * \param y_adv_mm Position Y de l'adversaire en mm.
	 * \param x_pos_mm Position X du robot en mm.
	 * \param y_pos_mm Position Y du robot en mm.
	 * \param color Code couleur (0 par défaut).
	 */
	virtual void writePosition_AdvPos(float x_adv_mm, float y_adv_mm, float x_pos_mm, float y_pos_mm,
			int color = 0) = 0;

	/*!
	 * \brief Dessine une zone d'action sur le terrain.
	 * \param name Nom de la zone.
	 * \param minX_mm Coin X minimum en mm.
	 * \param minY_mm Coin Y minimum en mm.
	 * \param width_mm Largeur en mm.
	 * \param height_mm Hauteur en mm.
	 * \param startX_mm Position X de départ en mm.
	 * \param startY_mm Position Y de départ en mm.
	 * \param startAngle_rad Angle de départ en radians.
	 */
	virtual void writeZone(const char *name, float minX_mm, float minY_mm, float width_mm, float height_mm,
			float startX_mm, float startY_mm, float startAngle_rad) = 0;

	/*!
	 * \brief Dessine un chemin IA entre deux zones.
	 * \param zone1Name Nom de la zone de départ.
	 * \param zone2Name Nom de la zone d'arrivée.
	 * \param x_mm Position X du point intermédiaire en mm.
	 * \param y_mm Position Y du point intermédiaire en mm.
	 */
	virtual void writeIaPath(const char *zone1Name, const char *zone2Name, float x_mm, float y_mm) = 0;

	/*!
	 * \brief Dessine une polyline SVG à partir d'une liste de points.
	 * \param points Liste de points au format SVG (ex: "0,0 100,50 200,0").
	 */
	virtual void pathPolyline(std::string points) = 0;

};

#endif
