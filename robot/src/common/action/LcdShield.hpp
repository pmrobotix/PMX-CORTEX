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
#include "AActionsElement.hpp"

class ALcdShieldDriver;

/*!
 * \brief Gestion de l'écran LCD du robot.
 */
class LcdShield: public AActionsElement
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
     * \brief Affiche une chaîne de caractères sur l'écran LCD.
     * \param str Texte à afficher.
     */
    void print(const char* str);

    /*!
     * \brief Affiche une chaîne de caractères sur l'écran LCD.
     * \param str Texte à afficher.
     */
    void print(const std::string& str);

    /*!
     * \brief Affiche un entier sur l'écran LCD.
     * \param value Valeur entière à afficher.
     */
    void print(int value);

    /*!
     * \brief Affiche une chaîne suivie d'un retour à la ligne.
     * \param str Texte à afficher.
     */
    void println(const char* str);


    /*!
     * \brief Destructor.
     */
    ~LcdShield();

};

#endif
