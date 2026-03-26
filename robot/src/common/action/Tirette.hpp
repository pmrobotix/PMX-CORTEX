/*! \file
 * \brief Définition de la classe Tirette.
 */

#ifndef TIRETTE_HPP_
#define TIRETTE_HPP_

#include "log/LoggerFactory.hpp"
#include "AActionsElement.hpp"

#include "interface/ASwitchDriver.hpp"

/*!
 * \brief Gère la tirette de démarrage du robot (interrupteur de lancement).
 */
class Tirette: public AActionsElement
{
private:

    /*!
     * \brief Retourne le \ref Logger associé à la classe \ref Tirette.
     */
    static inline const logs::Logger & logger()
    {
        static const logs::Logger & instance = logs::LoggerFactory::logger("Tirette");
        return instance;
    }

    ASwitchDriver* switchdriver_;

public:


    /*!
     * \brief Constructor.
     *
     */
    Tirette(Actions & actions);

    /*!
     * \brief Destructor.
     */
    ~Tirette();

    /*!
     * \brief Vérifie si la tirette est connectée.
     */
    bool is_connected();

    /*!
     * \brief Retourne l'état de la tirette (appuyée ou non).
     */
    int pressed();

    /*!
     * \brief Affiche l'état de la tirette pendant \p nb itérations (debug).
     * \param nb Nombre de lectures à effectuer.
     */
    void monitor(int nb);

    /*!
     * \brief Attend que la tirette soit enfoncée (bloquant).
     * \return \c true si la tirette a été enfoncée.
     */
    bool waitPressed();

    /*!
     * \brief Attend que la tirette soit retirée (bloquant).
     * \return \c true si la tirette a été retirée.
     */
    bool waitUnpressed();

    /*!
     * \brief Configure un GPIO associé à la tirette.
     * \param p Numéro du GPIO.
     * \param activate \c true pour activer, \c false pour désactiver.
     */
    void setGPIO(int p, bool activate);

};

#endif
