/*!
 * \file
 * \brief Implémentation des tests de la classe logs::Logger.
 */

#include "LoggerTest.hpp"
#include "../../src/common/log/Logger.hpp"
#include "../../src/common/log/LoggerFactory.hpp"
#include "../../src/common/log/Level.hpp"
#include "../suite/UnitTestAppender.hpp"

void test::LoggerTest::suite()
{
    testSimpleMessages();
    testStreamMessages();
    testLevelFiltering();
    testLoggerProperties();
    testLoggerCopyFromParent();
}

void test::LoggerTest::testSimpleMessages()
{
    const logs::Logger & logger = logs::LoggerFactory::logger("LoggerTest");

    // Verifier que le logger est actif pour tous les niveaux (configure en DEBUG)
    this->assert(logger.isActive(logs::Level::DEBUG), "Logger doit etre actif en DEBUG");
    this->assert(logger.isActive(logs::Level::INFO), "Logger doit etre actif en INFO");
    this->assert(logger.isActive(logs::Level::WARN), "Logger doit etre actif en WARN");
    this->assert(logger.isActive(logs::Level::ERROR), "Logger doit etre actif en ERROR");

    // Les messages simples ne doivent pas provoquer de crash
    logger.debug("test debug message");
    logger.info("test info message");
    logger.warn("test warn message");
    logger.error("test error message");

    // Le message d'erreur a ete ecrit, pas de crash
    this->assert(true, "Les messages simples sont traces sans erreur");
}

void test::LoggerTest::testStreamMessages()
{
    const logs::Logger & logger = logs::LoggerFactory::logger("LoggerTest");

    // Test de l'API stream avec logs::end - ne doit pas provoquer de crash
    logger.info() << "stream value=" << 42 << logs::end;
    logger.debug() << "stream debug " << 3.14 << logs::end;
    logger.warn() << "stream warn " << "text" << logs::end;

    this->assert(true, "Les messages stream sont traces sans erreur");
}

void test::LoggerTest::testLevelFiltering()
{
    // Creer un logger avec niveau WARN : DEBUG et INFO doivent etre filtres
    logs::Logger * rootLogger = logs::LoggerFactory::instance().rootLogger();
    logs::Logger warnLogger(logs::Level::WARN, "WarnOnly", (logs::Appender&)rootLogger->appender());

    this->assert(!warnLogger.isActive(logs::Level::DEBUG), "DEBUG doit etre inactif pour un logger WARN");
    this->assert(!warnLogger.isActive(logs::Level::INFO), "INFO doit etre inactif pour un logger WARN");
    this->assert(warnLogger.isActive(logs::Level::WARN), "WARN doit etre actif pour un logger WARN");
    this->assert(warnLogger.isActive(logs::Level::ERROR), "ERROR doit etre actif pour un logger WARN");

    // Creer un logger avec niveau NONE : tout doit etre filtre
    logs::Logger noneLogger(logs::Level::NONE, "NoneLogger", (logs::Appender&)rootLogger->appender());

    this->assert(!noneLogger.isActive(logs::Level::ERROR), "ERROR doit etre inactif pour un logger NONE");
}

void test::LoggerTest::testLoggerProperties()
{
    const logs::Logger & logger = logs::LoggerFactory::logger("LoggerTest");

    this->assert(logger.name() == "LoggerTest", "Le nom du logger doit etre 'LoggerTest'");
    this->assert(logger.isActive(logs::Level::DEBUG), "Le logger doit etre actif en DEBUG");
}

void test::LoggerTest::testLoggerCopyFromParent()
{
    const logs::Logger & parent = logs::LoggerFactory::logger("LoggerTest");

    // Constructeur par copie : herite du niveau et de l'appender du parent
    logs::Logger child(parent, "ChildLogger");

    this->assert(child.name() == "ChildLogger", "Le child doit avoir son propre nom");
    this->assert(child.level() == parent.level(), "Le child doit heriter du niveau du parent");
    this->assert(child.isActive(logs::Level::DEBUG), "Le child doit etre actif en DEBUG comme le parent");
}
