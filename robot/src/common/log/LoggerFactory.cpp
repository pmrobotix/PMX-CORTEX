/*!
 * \file
 * \brief Implémentation de la classe LoggerFactory.
 */

#include "LoggerFactory.hpp"

#include <unistd.h>
#include <iostream>
#include <utility>

#include "Level.hpp"

logs::LoggerFactory::LoggerFactory() :
        appenders_(), loggers_(), rootLogger_(), stop_(false), priority_(0)
{
    this->initialize();

    if (rootLogger() == NULL) {
        printf("ERROR Exception logs::LoggerFactory::LoggerFactory() NO default rootLogger() \n Exit!\n");
        sleep(1);
        exit(1);
    }
}

void logs::LoggerFactory::setPriority(int p)
{
    priority_ = p;
}

void logs::LoggerFactory::stopLog()
{
    stop_ = true;
    this->waitForEnd();
}

logs::LoggerFactory::~LoggerFactory()
{
    std::map<std::string, logs::Logger *>::iterator i1 = loggers_.begin();
    for (; i1 != loggers_.end(); i1++) {
        delete i1->second;
        i1->second = NULL;
    }
    std::map<std::string, logs::Appender *>::iterator i2 = appenders_.begin();
    for (; i2 != appenders_.end(); i2++) {
        delete i2->second;
        i2->second = NULL;
    }
    stopLog();
    this->cancel();
}

const logs::Logger &
logs::LoggerFactory::logger(const std::string & name)
{
    LoggerFactory & instance = logs::LoggerFactory::instance();

    std::map<std::string, logs::Logger *>::iterator value = instance.loggers_.find(name);
    if (value == instance.loggers_.end()) {
        Logger * logger = new Logger(*instance.rootLogger(), name);
        instance.add(logger);
        return *logger;
    }
    return *(value->second);
}

logs::Appender *
logs::LoggerFactory::appender(const std::string & name)
{
    std::map<std::string, logs::Appender *>::iterator value = this->appenders_.find(name);
    if (value == this->appenders_.end()) {
        return NULL;
    } else {
        return value->second;
    }
}

void logs::LoggerFactory::add(Logger * logger)
{
    // Le logger sans nom ("") devient le rootLogger et démarre le thread de flush.
    if (logger->name() == "") {
        this->rootLogger_ = logger;
        this->start("LoggerFactory", priority_);
    } else {
        loggers_.insert(std::make_pair(logger->name(), logger));
    }
}

void logs::LoggerFactory::add(const std::string & name, logs::Appender * appender)
{
    appenders_.insert(std::make_pair(name, appender));
}

void logs::LoggerFactory::add(const Level & level, const std::string & loggerName, const std::string & appenderName)
{
    Appender * appender = this->appender(appenderName);
    if (appender == NULL) {
        printf("ERROR Exception logs::LoggerFactory::add() %s, %s\nExit!\n", loggerName.c_str(), appenderName.c_str());
        sleep(1);
        exit(1);
    } else {
        Logger *log = new Logger(level, loggerName, *appender);
        this->add(log);
    }
}

void logs::LoggerFactory::execute()
{
    while (!stop_) {
        std::map<std::string, Appender *>::iterator it = appenders_.begin();
        for (it = appenders_.begin(); it != appenders_.end(); ++it) {
            it->second->flush();
        }
        utils::Thread::sleep_for_millis(300);
        this->yield();
    }
}
