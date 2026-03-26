/*!
 * \file
 * \brief Définition de la classe ButtonBar.
 */

#ifndef BUTTONBAR_HPP_
#define BUTTONBAR_HPP_

#include "log/LoggerFactory.hpp"
#include "AActionsElement.hpp"

#include "interface/AButtonDriver.hpp"

/*!
 * \brief Gestion de la barre de boutons du robot.
 */
class ButtonBar: public AActionsElement
{
private:

    /*!
     * \brief Retourne le \ref Logger associé à la classe \ref ButtonBar.
     */
    static inline const logs::Logger & logger()
    {
        static const logs::Logger & instance = logs::LoggerFactory::logger("ButtonBar");
        return instance;
    }

    /*!
     * \brief Driver matériel des boutons.
     */
    AButtonDriver* buttondriver_;

public:


    /*!
     * \brief Constructor.
     *
     */
    ButtonBar(Actions & actions);

    /*!
     * \brief Destructor.
     */
    ~ButtonBar();

    /*!
     * \brief Vérifie si un bouton est actuellement appuyé.
     * \param button Le bouton à tester.
     * \return \c true si le bouton est appuyé, \c false sinon.
     */
    bool pressed(ButtonTouch button);

    //bool process(ButtonTouch button);

    /*!
     * \brief Attend qu'un bouton soit appuyé (bloquant).
     * \param button Le bouton attendu.
     * \return \c true si le bouton a été appuyé.
     */
    bool waitPressed(ButtonTouch button);

    /*!
     * \brief Attend qu'un des boutons soit appuyé (bloquant).
     * \return Le bouton qui a été appuyé.
     */
    ButtonTouch waitOneOfAllPressed();

    /*!
     * \brief Vérifie si un des boutons est appuyé (non bloquant).
     * \return Le bouton appuyé, ou une valeur par défaut si aucun.
     */
    ButtonTouch checkOneOfAllPressed();

};

#endif
