/*!
 * \file
 * \brief Définition de la classe Chronometer.
 */

#ifndef COMMON_CHRONOMETER_HPP_
#define	COMMON_CHRONOMETER_HPP_

#include <sys/time.h>
#include <string>

namespace utils {
/*!
 * \brief Gestion d'un chronomètre.
 *
 * Cette précision de ce chronomètre est de l'ordre de la microseconde.
 * Son fonctionnement est 'basique' :
 * - il est lancé via la méthode start(),
 * - il est arrété via la méthode stop().
 * - Les méthodes getElapsedTimeXXX() retournent le temps associé au compteur.
 *
 * Remarques :
 * Les appels successifs à start() réinitialise le chronomètre.
 * Le constructeur initialise un chronomètre mais ne le lance pas.
 *
 */
class Chronometer
{
public:
    /*!
     * \brief Retourne l'heure système courante.
     * \return Structure timeval avec l'heure courante.
     */
    static timeval getTime();

private:

    int stopped_;               ///< Flag d'arrêt (1 = stoppé).
    int timerPeriod_us_;        ///< Période du timer en microsecondes.
    std::string name_;          ///< Nom du chronomètre (pour les logs).
    timeval startCount_;        ///< Temps de départ.
    timeval endCount_;          ///< Temps d'arrêt.
    timeval endSet_;            ///< Consigne de fin (deprecated).
    unsigned long long periodNb_;       ///< Compteur de périodes.
    unsigned int endSetTime_us;         ///< (deprecated)
    unsigned long long timerStartTime_us_; ///< (deprecated)
    unsigned long long lastTime_;       ///< Dernier temps mesuré (pour waitTimer).

public:

    /*!
     * \brief Constructeur. Le chronomètre n'est pas lancé.
     * \param name Nom du chronomètre (pour identification).
     */
    Chronometer(std::string name);

    /*!
     * \brief Destructeur.
     */
    virtual inline ~Chronometer() {}

    /*!
     * \brief Indique si le chronomètre est en cours.
     * \return \c true si démarré.
     */
    inline bool started()
    {
        return !stopped_;
    }

    /*!
     * \brief Lance ou réinitialise le chronomètre.
     */
    void start();

    /*!
     * \brief Arrête le chronomètre.
     */
    void stop();

    /*!
     * \brief Configure le timer avec une période donnée et lance le chronomètre.
     * \param usec Période en microsecondes.
     */
    void setTimer(unsigned int usec);

    /*!
     * \brief Attend la fin de la période en cours.
     *
     * Dort le temps restant pour compléter une période de timerPeriod_us_.
     * Affiche un message d'erreur si la période est dépassée (overflow).
     *
     * \param delay_us Délai de compensation en microsecondes (défaut 0).
     * \param debug Activer les logs de debug (défaut false).
     * \return Le numéro de la période courante.
     */
    int waitTimer(int delay_us = 0, bool debug = false);

    /*!
     * \brief Alias pour getElapsedTimeInSec().
     * \return Le temps écoulé en secondes.
     */
    float getElapsedTime();

    /*!
     * \brief Retourne le temps écoulé en secondes.
     */
    float getElapsedTimeInSec();

    /*!
     * \brief Retourne le temps écoulé en millisecondes.
     */
    float getElapsedTimeInMilliSec();

    /*!
     * \brief Retourne le temps écoulé en microsecondes.
     */
    unsigned long long getElapsedTimeInMicroSec();
};
}
#endif
