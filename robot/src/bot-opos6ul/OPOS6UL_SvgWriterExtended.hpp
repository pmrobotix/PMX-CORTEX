/*!
 * \file OPOS6UL_SvgWriterExtended.hpp
 * \brief Extension de l'écriture SVG pour la carte OPOS6UL.
 */

#ifndef OPOS6UL_SVGWRITEREXTENDED_HPP_
#define OPOS6UL_SVGWRITEREXTENDED_HPP_

#include <string>

#include "log/Logger.hpp"
#include "log/SvgWriter.hpp"

/*!
 * \brief Génération de fichiers SVG pour la visualisation du robot et du terrain.
 *
 * Permet d'écrire les positions du robot, de l'adversaire, les zones et les chemins IA.
 */
class OPOS6UL_SvgWriterExtended: public SvgWriter
{
private:
    logs::Logger::LoggerBuffer *fLogBuffer;
    logs::Logger::LoggerBuffer *fLogBufferSensors;
public:

    OPOS6UL_SvgWriterExtended(std::string botId);

    ~OPOS6UL_SvgWriterExtended()
    {
    }

    /*!
     * \brief Écrit la position de l'adversaire et du robot dans le SVG.
     * \param x_adv_mm Position X de l'adversaire en mm.
     * \param y_adv_mm Position Y de l'adversaire en mm.
     * \param x_pos_mm Position X du robot en mm.
     * \param y_pos_mm Position Y du robot en mm.
     * \param color Couleur du tracé (0 par défaut).
     */
    void writePosition_AdvPos(float x_adv_mm, float y_adv_mm, float x_pos_mm, float y_pos_mm, int color = 0);

    /*!
     * \brief Écrit la position et l'orientation du robot dans le SVG.
     * \param x_mm Position X en mm.
     * \param y_mm Position Y en mm.
     * \param angle_rad Angle d'orientation en radians.
     * \param color Couleur du tracé (0 par défaut).
     */
    void writePosition_Bot(float x_mm, float y_mm, float angle_rad, int color = 0);

    /*!
     * \brief Écrit la position du robot avec orientation (version simplifiée).
     * \param x_mm Position X en mm.
     * \param y_mm Position Y en mm.
     * \param angle_rad Angle d'orientation en radians.
     */
    void writePosition_BotPos(float x_mm, float y_mm, float angle_rad);

    /*!
     * \brief Écrit une zone d'action sur le terrain dans le SVG.
     * \param name Nom de la zone.
     * \param minX Coordonnée X minimale de la zone.
     * \param minY Coordonnée Y minimale de la zone.
     * \param width Largeur de la zone.
     * \param height Hauteur de la zone.
     * \param startX Position X de départ dans la zone.
     * \param startY Position Y de départ dans la zone.
     * \param startAngle_rad Angle de départ en radians.
     */
    void writeZone(const char* name, float minX, float minY, float width, float height, float startX, float startY,
            float startAngle_rad);

    /*!
     * \brief Écrit un chemin IA entre deux zones dans le SVG.
     * \param zone1Name Nom de la zone de départ.
     * \param zone2Name Nom de la zone d'arrivée.
     * \param x_mm Position X intermédiaire en mm.
     * \param y_mm Position Y intermédiaire en mm.
     */
    void writeIaPath(const char* zone1Name, const char* zone2Name, float x_mm, float y_mm);

    /*!
     * \brief Écrit un chemin polyline (suite de points) dans le SVG.
     * \param points Chaîne de coordonnées au format SVG polyline.
     */
    void pathPolyline(std::string points);
};

#endif
