/*!
 * \file
 * \brief Définition de la classe LcdShield.
 */

#ifndef LCDSHIELD_HPP_
#define LCDSHIELD_HPP_

#include <stddef.h>
#include <cstdint>
#include <string>

#include "log/LoggerFactory.hpp"
#include "utils/Print.hpp"
#include "AActionsElement.hpp"

class ALcdShieldDriver;

/*!
 * \brief Gestion de l'écran LCD du robot.
 */
class LcdShield: public AActionsElement, public Print
{
private:

    /*!
     * \brief Retourne le \ref Logger associé à la classe \ref LcdShield.
     */
    static inline const logs::Logger & logger()
    {
        static const logs::Logger & instance = logs::LoggerFactory::logger("LcdShield");
        return instance;
    }

    /*!
     * \brief Driver matériel de l'écran LCD.
     */
    ALcdShieldDriver* lcdshielddriver_;


public:

    /*!
     * \brief ID du robot.
     */
    std::string botId_;

    /*!
     * \brief Constructor.
     *
     * \param nb Number of leds in the LedBar.
     */
    LcdShield(std::string botId, Actions & actions);

    /*!
     * \brief Vérifie si l'écran LCD est connecté.
     * \return \c true si connecté, \c false sinon.
     */
    bool is_connected();

    /*!
     * \brief Initialise l'écran LCD.
     */
    void init();

    /*!
     * \brief Réinitialise l'écran LCD.
     */
    void reset();

    /*!
     * \brief Efface le contenu de l'écran.
     */
    void clear();

    /*!
     * \brief Positionne le curseur en haut à gauche.
     */
    void home();

    /*!
     * \brief Active le rétroéclairage.
     */
    void setBacklightOn();

    /*!
     * \brief Désactive le rétroéclairage.
     */
    void setBacklightOff();

    /*!
     * \brief Positionne le curseur à une position donnée.
     * \param col Colonne (0-based).
     * \param row Ligne (0-based).
     */
    void setCursor(uint8_t col, uint8_t row);

    /*
     void noDisplay();
     void display();
     void noBlink();
     void blink();
     void noCursor();
     void cursor();
     void scrollDisplayLeft();
     void scrollDisplayRight();
     void leftToRight();
     void rightToLeft();
     void autoscroll();
     void noAutoscroll();
     */

    /*!
     * \brief Écrit un octet sur l'écran LCD.
     * \param value Octet à écrire.
     * \return Nombre d'octets écrits.
     */
    virtual size_t write(uint8_t value);

    /*!
     * \brief Affiche une chaîne de caractères à une position donnée.
     * \param str Texte à afficher.
     * \param row Ligne d'affichage.
     * \param col Colonne d'affichage (par défaut 1).
     */
    void display_content_string(std::string str, int row, int col=1);

    /*!
     * \brief Affiche un entier à une position donnée.
     * \param value Valeur entière à afficher.
     * \param row Ligne d'affichage.
     * \param col Colonne d'affichage (par défaut 1).
     */
    void display_content_integer(int value, int row, int col=1);


    /*!
     * \brief Destructor.
     */
    ~LcdShield();

};

#endif
