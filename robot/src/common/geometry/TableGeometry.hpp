/*!
 * \file
 * \brief Geometrie de la table de jeu et filtres d'appartenance au terrain.
 *
 * Centralise les dimensions de la table et les tests d'appartenance
 * au terrain. Remplace les anciennes methodes filtre_IsInsideTable*
 * qui etaient dans Asserv (couplage indesirable).
 */

#ifndef COMMON_GEOMETRY_TABLEGEOMETRY_HPP_
#define COMMON_GEOMETRY_TABLEGEOMETRY_HPP_

#include "log/LoggerFactory.hpp"
#include "interface/ARobotPositionShared.hpp"

/*!
 * \brief Dimensions et tests d'appartenance a la table de jeu.
 *
 * Chaque robot instancie sa propre TableGeometry avec ses dimensions
 * (largeur, hauteur) et sa marge de filtrage. L'acces a la position
 * courante du robot se fait via ARobotPositionShared pour projeter
 * une detection capteur sur la table.
 */
class TableGeometry {
private:

	static inline const logs::Logger& logger()
	{
		static const logs::Logger &instance = logs::LoggerFactory::logger("TableGeometry");
		return instance;
	}

	int table_width_mm_;   ///< Largeur de la table en mm (axe X).
	int table_height_mm_;  ///< Hauteur de la table en mm (axe Y).
	int margin_mm_;        ///< Marge de filtre aux bords (mm).

	ARobotPositionShared *sharedPos_;  ///< Position partagee (non-owning).

public:

	/*!
	 * \brief Constructeur.
	 * \param table_width_mm Largeur de la table en mm.
	 * \param table_height_mm Hauteur de la table en mm.
	 * \param margin_mm Marge de filtre aux bords en mm.
	 * \param sharedPos Pointeur vers la position partagee du robot (non-owning).
	 */
	TableGeometry(int table_width_mm, int table_height_mm, int margin_mm,
			ARobotPositionShared *sharedPos) :
			table_width_mm_(table_width_mm),
			table_height_mm_(table_height_mm),
			margin_mm_(margin_mm),
			sharedPos_(sharedPos)
	{
	}

	~TableGeometry()
	{
	}

	/*!
	 * \brief Verifie si un point (repere table) est dans les limites du terrain.
	 * \param x_mm Position X dans le repere table (mm).
	 * \param y_mm Position Y dans le repere table (mm).
	 * \return true si le point est a l'interieur du terrain (avec marge).
	 */
	bool isPointInsideTable(int x_mm, int y_mm) const
	{
		bool inside = (x_mm > margin_mm_)
				&& (x_mm < table_width_mm_ - margin_mm_)
				&& (y_mm > margin_mm_)
				&& (y_mm < table_height_mm_ - margin_mm_);

		logger().debug() << "isPointInsideTable(" << x_mm << ", " << y_mm
				<< ") => " << inside << logs::end;
		return inside;
	}

	/*!
	 * \brief Verifie si une detection capteur projetee est dans le terrain.
	 *
	 * Projette la detection (distance + position laterale capteur) dans le
	 * repere table en utilisant la position courante du robot, puis teste
	 * l'appartenance au terrain.
	 *
	 * \param dist_detect_mm Distance de detection capteur en mm (peut etre negatif pour arriere).
	 * \param lateral_pos_sensor_mm Position laterale du capteur (-1 gauche, 0 centre, 1 droite).
	 * \return true si la detection pointe dans le terrain.
	 */
	bool isSensorReadingInsideTable(int dist_detect_mm, int lateral_pos_sensor_mm) const
	{
		if (sharedPos_ == nullptr) {
			logger().error() << "isSensorReadingInsideTable: sharedPos_ is null"
					<< logs::end;
			return false;
		}

		ROBOTPOSITION p = sharedPos_->getRobotPosition(0);

		// Projection du capteur dans le repere table.
		// Le capteur est a 140mm du centre robot, decale lateralement.
		float distmm = (float) dist_detect_mm;
		float lateral_offset_mm = (float) lateral_pos_sensor_mm * 140.0f;

		float x = p.x + (lateral_offset_mm * cos(p.theta - M_PI_2))
				+ (distmm * cos(p.theta));
		float y = p.y + (lateral_offset_mm * sin(p.theta - M_PI_2))
				+ (distmm * sin(p.theta));

		return isPointInsideTable((int) x, (int) y);
	}

	inline int widthMm() const { return table_width_mm_; }
	inline int heightMm() const { return table_height_mm_; }
	inline int marginMm() const { return margin_mm_; }
};

#endif
