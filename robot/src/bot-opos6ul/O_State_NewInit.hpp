/*!
 * \file
 * \brief Declaration de O_State_NewInit (remplacant de O_State_Init).
 *
 * Voir robot/md/O_STATE_NEW_INIT.md section 7.
 *
 * Cet etat gere l'init multi-sources (shield LCD 2x16 + LCD tactile balise)
 * via le MenuController et les phases de match (CONFIG/COMMITTED/PRIMED/MATCH).
 *
 * IMPORTANT : pour l'instant cet etat n'est PAS branche dans
 * OPOS6UL_RobotExtended::begin() — il coexiste avec O_State_Init. La bascule
 * sera un changement minimal (1 ligne dans OPOS6UL_RobotExtended.cpp) quand
 * les tests manuels auront valide le nouveau flow.
 */

#ifndef O_STATE_NEWINIT_HPP_
#define O_STATE_NEWINIT_HPP_

#include <string>

#include "log/LoggerFactory.hpp"
#include "state/AAutomateState.hpp"

class MenuController;

class O_State_NewInit : public AAutomateState
{
private:
	static inline const logs::Logger& logger()
	{
		static const logs::Logger &instance = logs::LoggerFactory::logger("O_State_NewInit");
		return instance;
	}

	void setPos();
	void handleTestModeRequest();

	/*!
	 * \brief Attend que la Nucleo (asserv) soit connectee avant setPos().
	 *        - ctrl != nullptr (mode menu) : tick les sources, accepte un reset
	 *          (BACK/RESET) pour annuler, retourne false.
	 *        - ctrl == nullptr (mode /k)   : attend indefiniment (l'utilisateur
	 *          peut ^C).
	 * \return true si la Nucleo est connectee, false si l'operateur a annule.
	 */
	bool waitForAsserv(MenuController* ctrl);

public:
	O_State_NewInit() = default;
	~O_State_NewInit() = default;

	IAutomateState* execute(Robot& robot) override;

	std::string name() override { return "O_State_NewInit"; }
};

#endif
