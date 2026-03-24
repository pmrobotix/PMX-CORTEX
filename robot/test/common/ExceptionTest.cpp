/*!
 * \file
 * \brief Implémentation des tests de la classe logs::Exception.
 */

#include "ExceptionTest.hpp"
#include "../../src/common/log/Exception.hpp"

#include <string>

void test::ExceptionTest::suite()
{
    testMessageAccess();
    testMessageModification();
    testWhat();
    testThrowCatch();
}

void test::ExceptionTest::testMessageAccess()
{
    logs::Exception ex("erreur de test");

    this->assert(ex.message() == "erreur de test", "message() doit retourner le message initial");
}

void test::ExceptionTest::testMessageModification()
{
    logs::Exception ex("message original");

    ex.message("message modifie");

    this->assert(ex.message() == "message modifie", "message() doit retourner le message modifie");
}

void test::ExceptionTest::testWhat()
{
    logs::Exception ex("what test");

    std::string result = ex.what();

    this->assert(result == "what test", "what() doit retourner le meme contenu que message()");
}

void test::ExceptionTest::testThrowCatch()
{
    bool caught = false;
    std::string caughtMessage;

    try {
        throw logs::Exception("exception levee");
    } catch (logs::Exception & e) {
        caught = true;
        caughtMessage = e.message();
    }

    this->assert(caught, "L'exception doit etre attrapee");
    this->assert(caughtMessage == "exception levee", "Le message attrape doit correspondre");

    // Test catch via std::exception&
    bool caughtAsStd = false;

    try {
        throw logs::Exception("std exception");
    } catch (std::exception & e) {
        caughtAsStd = true;
    }

    this->assert(caughtAsStd, "L'exception doit etre attrapable via std::exception&");
}
