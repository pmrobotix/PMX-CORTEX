/*!
 * \file
 * \brief Déclaration de la classe O_State_WaitEndOfMatch.
 */

#ifndef O_STATE_WAITENDOFMATCH_HPP
#define	O_STATE_WAITENDOFMATCH_HPP

#include <string>

#include "state/AAutomateState.hpp"
#include "log/LoggerFactory.hpp"

class Robot;

/*!
 * \brief Cette classe est la tache principale du robot.
 */
class O_State_WaitEndOfMatch: public AAutomateState
{
private:

    static inline const logs::Logger & logger()
    {
        static const logs::Logger & instance = logs::LoggerFactory::logger("O_State_WaitEndOfMatch");
        return instance;
    }

public:

    /*!
     * \brief Constructeur de la classe.
     */
    O_State_WaitEndOfMatch()
    {
    }

    /*!
     * \brief Destructeur de la classe.
     */
    ~ O_State_WaitEndOfMatch()
    {
    }

    /*!
     * \brief Exécute l'état d'attente de fin de match.
     * \param robot Référence vers le robot.
     * \return Pointeur vers le prochain état de l'automate.
     */
    IAutomateState* execute(Robot& robot);

    /*!
     * \brief Retourne le nom de cet état.
     * \return Nom de l'état sous forme de chaîne.
     */
    std::string name()
    {
        return "O_State_WaitEndOfMatch";
    }
};

#endif
