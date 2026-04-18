/*!
 * \file
 * \brief Orchestrateur des sources d'input/output du menu d'init.
 *
 * Detient un vector<IMenuSource>. Sur chaque tick() appelle pollInputs sur
 * toutes les sources d'abord (phase lecture), puis refreshDisplay sur toutes
 * (phase ecriture). Cet ordre garantit qu'a la fin du tick, tous les affichages
 * refletent le meme etat Robot coherent.
 *
 * Voir robot/md/O_STATE_NEW_INIT.md section 4.1.
 */

#ifndef COMMON_MENU_MENUCONTROLLER_HPP_
#define COMMON_MENU_MENUCONTROLLER_HPP_

#include <memory>
#include <vector>

#include "IMenuSource.hpp"
#include "log/LoggerFactory.hpp"

class Robot;

class MenuController
{
private:
	static inline const logs::Logger& logger()
	{
		static const logs::Logger &instance = logs::LoggerFactory::logger("MenuController");
		return instance;
	}

	std::vector<std::unique_ptr<IMenuSource>> sources_;
	Robot& robot_;

public:
	explicit MenuController(Robot& r) : robot_(r) {}

	/*!
	 * \brief Ajoute une source au controller. Propriete transferee.
	 */
	void add(std::unique_ptr<IMenuSource> s);

	/*!
	 * \brief Appele periodiquement (~100 Hz) par O_State_NewInit.
	 *
	 * Ordre critique :
	 *   1. pollInputs sur TOUTES les sources vivantes
	 *   2. refreshDisplay sur TOUTES les sources vivantes
	 */
	void tick();

	/*!
	 * \brief Au moins une source est vivante.
	 */
	bool anyAlive() const;

	/*!
	 * \brief Nombre de sources vivantes.
	 */
	size_t aliveCount() const;

	/*!
	 * \brief Nombre total de sources ajoutees.
	 */
	size_t size() const { return sources_.size(); }
};

#endif
