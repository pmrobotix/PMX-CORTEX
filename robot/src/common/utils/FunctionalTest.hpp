/*!
 * \file
 * \brief Class definition for functional tests.
 */

#ifndef COMMON_FUNCTIONALTEST_HPP_
#define	COMMON_FUNCTIONALTEST_HPP_

#include <string>

class Arguments;

/*!
 * \brief Abstract class to implement functional tests using ConsoleManager.
 */
class FunctionalTest
{
private:

    /*!
     * \brief Nom du test.
     */
    std::string name_;
    std::string description_;
    /*!
     * \brief Code mnemonique court (2-3 lettres) pour lancer le test en CLI.
     */
    std::string code_;
    /*!
     * \brief Numero du test dans la liste.
     */
    int num_;

protected:

    /*!
     * \brief Constructeur de la classe.
     * \param name
     *        Nom du test.
     */
    FunctionalTest(const std::string & name, const std::string & desc, const std::string & code = "") :
            name_(name), description_(desc), code_(code), num_(0)
    {
    }

public:

    /*!
     * Destructeur de la classe.
     */
    virtual inline ~ FunctionalTest()
    {
    }

    /*!
     * \return Nom du test.
     */
    inline const std::string & name() const
    {
        return name_;
    }

    /*!
     * \return Description du test.
     */
    inline const std::string & desc() const
    {
        return description_;
    }

    /*!
     * \return Code mnemonique du test.
     */
    inline const std::string & code() const
    {
        return code_;
    }

    /*!
     * \return Numero/position du test.
     */
    inline int position()
    {
        return num_;
    }

    /*!
     * \return Numero/position du test.
     */
    inline void setPos(int num)
    {
        num_ = num;
    }

    /*!
     * Methode executant le test associe.
     */
    virtual void run(int argc, char** argv) = 0;

    virtual void configureConsoleArgs(int, char**) // A surcharger par le test en question pour ajouter ses parametres specifiques
    {
    }

    /*!
     * \brief Retourne les arguments par defaut pour ce test (ex: "500 90 0 /+ 250 250 0").
     * Utilise par ConsoleManager quand le test est lance sans arguments explicites.
     */
    virtual std::string defaultArgs() const
    {
        return "";
    }

    /*!
     * \brief Aide multi-lignes affichee par /h sous chaque test.
     *
     * Convention : 1ere ligne = synopsis des args positionnels et options,
     * lignes suivantes = exemples concrets indentes de 8 espaces.
     * Retourne "" si pas d'aide -> ConsoleManager n'imprime rien.
     */
    virtual std::string usageHelp() const
    {
        return "";
    }
};

#endif
