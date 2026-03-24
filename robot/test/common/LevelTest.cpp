/*!
 * \file
 * \brief Implémentation des tests de la classe logs::Level.
 */

#include "LevelTest.hpp"
#include "../../src/common/log/Level.hpp"

void test::LevelTest::suite()
{
    testNames();
    testGreaterOrEqual();
    testEquality();
    testInequality();
    testOrdering();
}

void test::LevelTest::testNames()
{
    this->assert(logs::Level::DEBUG.name() == "DEBUG", "DEBUG.name() doit etre 'DEBUG'");
    this->assert(logs::Level::INFO.name() == "INFO", "INFO.name() doit etre 'INFO'");
    this->assert(logs::Level::WARN.name() == "WARN", "WARN.name() doit etre 'WARN'");
    this->assert(logs::Level::TELEM.name() == "TELEM", "TELEM.name() doit etre 'TELEM'");
    this->assert(logs::Level::ERROR.name() == "ERROR", "ERROR.name() doit etre 'ERROR'");
    this->assert(logs::Level::ALL.name() == "ALL", "ALL.name() doit etre 'ALL'");
    this->assert(logs::Level::NONE.name() == "NONE", "NONE.name() doit etre 'NONE'");
}

void test::LevelTest::testGreaterOrEqual()
{
    // Chaque niveau est >= a lui-meme
    this->assert(logs::Level::DEBUG >= logs::Level::DEBUG, "DEBUG >= DEBUG");
    this->assert(logs::Level::INFO >= logs::Level::INFO, "INFO >= INFO");
    this->assert(logs::Level::ERROR >= logs::Level::ERROR, "ERROR >= ERROR");

    // Niveaux superieurs >= inferieurs
    this->assert(logs::Level::INFO >= logs::Level::DEBUG, "INFO >= DEBUG");
    this->assert(logs::Level::ERROR >= logs::Level::DEBUG, "ERROR >= DEBUG");
    this->assert(logs::Level::NONE >= logs::Level::ERROR, "NONE >= ERROR");

    // Niveaux inferieurs NOT >= superieurs
    this->assert(!(logs::Level::DEBUG >= logs::Level::INFO), "DEBUG pas >= INFO");
    this->assert(!(logs::Level::ALL >= logs::Level::DEBUG), "ALL pas >= DEBUG");
}

void test::LevelTest::testEquality()
{
    this->assert(logs::Level::DEBUG == logs::Level::DEBUG, "DEBUG == DEBUG");
    this->assert(logs::Level::ERROR == logs::Level::ERROR, "ERROR == ERROR");
    this->assert(!(logs::Level::DEBUG == logs::Level::INFO), "DEBUG != INFO via ==");
}

void test::LevelTest::testInequality()
{
    this->assert(logs::Level::DEBUG != logs::Level::INFO, "DEBUG != INFO");
    this->assert(logs::Level::WARN != logs::Level::ERROR, "WARN != ERROR");
    this->assert(!(logs::Level::DEBUG != logs::Level::DEBUG), "DEBUG pas != DEBUG");
}

void test::LevelTest::testOrdering()
{
    // ALL(0) < DEBUG(5) < INFO(10) < WARN(15) < TELEM(18) < ERROR(20) < NONE(100)
    this->assert(logs::Level::DEBUG >= logs::Level::ALL, "DEBUG >= ALL");
    this->assert(logs::Level::INFO >= logs::Level::DEBUG, "INFO >= DEBUG");
    this->assert(logs::Level::WARN >= logs::Level::INFO, "WARN >= INFO");
    this->assert(logs::Level::TELEM >= logs::Level::WARN, "TELEM >= WARN");
    this->assert(logs::Level::ERROR >= logs::Level::TELEM, "ERROR >= TELEM");
    this->assert(logs::Level::NONE >= logs::Level::ERROR, "NONE >= ERROR");

    // Verification inverse
    this->assert(!(logs::Level::ALL >= logs::Level::DEBUG), "ALL pas >= DEBUG");
    this->assert(!(logs::Level::DEBUG >= logs::Level::INFO), "DEBUG pas >= INFO");
    this->assert(!(logs::Level::INFO >= logs::Level::WARN), "INFO pas >= WARN");
    this->assert(!(logs::Level::WARN >= logs::Level::TELEM), "WARN pas >= TELEM");
    this->assert(!(logs::Level::TELEM >= logs::Level::ERROR), "TELEM pas >= ERROR");
    this->assert(!(logs::Level::ERROR >= logs::Level::NONE), "ERROR pas >= NONE");
}
