/*!
 * \file
 * \brief Déclaration de la classe O_State_Init.
 */

#ifndef O_STATE_INIT_HPP
#define	O_STATE_INIT_HPP

#include <string>

#include "state/AAutomateState.hpp"
#include "log/LoggerFactory.hpp"

/*!
 * \brief État d'initialisation du robot.
 *
 * Cet état configure la position initiale et prépare le robot
 * avant le début du match.
 */
class O_State_Init: public AAutomateState
{
private:

    static inline const logs::Logger & logger()
    {
        static const logs::Logger & instance = logs::LoggerFactory::logger("O_State_Init");
        return instance;
    }

    void setPos();

public:

    O_State_Init()
    {
    }

    ~O_State_Init()
    {
    }

    /*!
     * \brief Exécute l'état d'initialisation.
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
        return "O_State_Init";
    }
};

#endif
