/*!
 * \file
 * \brief Déclaration de la classe O_AsservLineRotateOldTest (deprecated).
 */

#ifndef O_ASSERVLINEROTATEOLDTEST_HPP
#define O_ASSERVLINEROTATEOLDTEST_HPP

#include "utils/FunctionalTest.hpp"
#include "log/LoggerFactory.hpp"

/*!
 * \brief [DEPRECATED] Ancien test ligne droite + rotation (sans Navigator).
 *
 * Remplacé par O_AsservLineRotateTest qui utilise Navigator.
 */
class O_AsservLineRotateOldTest: public FunctionalTest
{
private:

    static inline const logs::Logger & logger()
    {
        static const logs::Logger & instance = logs::LoggerFactory::logger("O_AsservLineRotateOldTest");
        return instance;
    }

public:

    O_AsservLineRotateOldTest() :
            FunctionalTest("Asserv_LineRotate_OLD", "[DEPRECATED] Ligne droite + rotation (ancien, sans Navigator).")
    {
    }

    virtual ~O_AsservLineRotateOldTest()
    {
    }

    virtual void run(int argc, char** argv);

    virtual void configureConsoleArgs(int argc, char** argv);

};

#endif
