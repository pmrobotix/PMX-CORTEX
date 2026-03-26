/*!
 * \file
 * \brief Definition de l'interface ITimerListener.
 */

#ifndef COMMON_ITIMERLISTENER_HPP
#define COMMON_ITIMERLISTENER_HPP

#include <string>

namespace utils {
class Chronometer;
}

/*!
 * \brief Cette interface represente une action executee par un timer lorsqu'il
 * atteint son seuil d'execution.
 */
class ITimerListener
{
public:

    /*!
     * \brief Actions a executer pour le timer.
     */
    virtual void onTimer(utils::Chronometer chrono) = 0;

    /*!
     * \brief Actions de fin a executer pour le timer.
     */
    virtual void onTimerEnd(utils::Chronometer chrono) = 0;

    /*!
     * \brief Getter sur les infos permettant d'identifier le timer.
     */
    virtual std::string name()
    {
        return name_;
    }

    /*!
     * \brief Retourne l'intervalle du timer en millisecondes.
     */
    inline int timeSpan()
    {
        return timeSpan_ms_;
    }

    /*!
     * \brief Retourne le timestamp de la derniere execution (microsecondes).
     */
    inline int getLastTime()
    {
        return lasttime_;
    }

    /*!
     * \brief Met a jour le timestamp de la derniere execution.
     * \param l Timestamp en microsecondes.
     */
    inline void setLastTime(long l)
    {
        lasttime_ = l;
    }

    /*!
     * \brief Indique si le timer a demande son arret.
     * \return true si le timer doit etre arrete.
     */
    inline bool requestToStop()
    {
        return requestToStop_;
    }

    /*!
     * \brief Destructeur de la classe.
     */
    virtual inline ~ ITimerListener()
    {
    }

protected:

    bool requestToStop_;       ///< true si le timer a demande son arret.
    long timeSpan_ms_;         ///< Intervalle du timer en millisecondes.
    long lasttime_;            ///< Timestamp de la derniere execution (microsecondes).
    std::string name_;         ///< Nom identifiant le timer.

    /*!
     * \brief Initialise le timer avec un label et un intervalle.
     * \param label Nom du timer.
     * \param time_us Intervalle en microsecondes (converti en ms).
     */
    void init(std::string label, uint time_us)
    {
        name_ = label;
        timeSpan_ms_ = time_us / 1000.0;
    }

    /*!
     * \brief Constructeur de la classe.
     */
    ITimerListener()
    {

        timeSpan_ms_ = 0;
        lasttime_ = 0;
        name_ = "iTimerListener_default";
        requestToStop_ = false;
    }
};

#endif
