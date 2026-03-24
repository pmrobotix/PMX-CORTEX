/*!
 * \file
 * \brief Implémentation des tests de la classe logs::FileAppender.
 */

#include "FileAppenderTest.hpp"
#include "../../src/common/log/appender/FileAppender.hpp"
#include "../../src/common/log/Logger.hpp"
#include "../../src/common/log/LoggerFactory.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cstdio>

// Utilitaire : lit toutes les lignes d'un fichier
static std::vector<std::string> readLines(const std::string & filename)
{
    std::vector<std::string> lines;
    std::ifstream ifs(filename);
    std::string line;
    while (std::getline(ifs, line)) {
        lines.push_back(line);
    }
    return lines;
}

void test::FileAppenderTest::suite()
{
    testWriteAndFlush();
    testMultipleMessages();
    testEmptyFile();
    testDestructorFlushes();
}

void test::FileAppenderTest::testWriteAndFlush()
{
    const std::string filename = "/tmp/pmx_test_fileappender_1.log";
    std::remove(filename.c_str());

    const logs::Logger & logger = logs::LoggerFactory::logger("FileAppenderTest");

    {
        logs::FileAppender appender(filename);
        appender.writeMessage(logger, logs::Level::INFO, "hello file");
        appender.flush();
    }

    std::vector<std::string> lines = readLines(filename);
    this->assert(lines.size() == 1, "Le fichier doit contenir 1 ligne apres flush");
    this->assert(lines[0] == "hello file", "Le contenu doit etre 'hello file'");

    std::remove(filename.c_str());
}

void test::FileAppenderTest::testMultipleMessages()
{
    const std::string filename = "/tmp/pmx_test_fileappender_2.log";
    std::remove(filename.c_str());

    const logs::Logger & logger = logs::LoggerFactory::logger("FileAppenderTest");

    {
        logs::FileAppender appender(filename);
        appender.writeMessage(logger, logs::Level::INFO, "ligne 1");
        appender.writeMessage(logger, logs::Level::DEBUG, "ligne 2");
        appender.writeMessage(logger, logs::Level::ERROR, "ligne 3");
        appender.flush();
    }

    std::vector<std::string> lines = readLines(filename);
    this->assert(lines.size() == 3, "Le fichier doit contenir 3 lignes");
    this->assert(lines[0] == "ligne 1", "Ligne 1 correcte");
    this->assert(lines[1] == "ligne 2", "Ligne 2 correcte");
    this->assert(lines[2] == "ligne 3", "Ligne 3 correcte");

    std::remove(filename.c_str());
}

void test::FileAppenderTest::testEmptyFile()
{
    const std::string filename = "/tmp/pmx_test_fileappender_3.log";
    std::remove(filename.c_str());

    {
        logs::FileAppender appender(filename);
        appender.flush();
    }

    std::vector<std::string> lines = readLines(filename);
    this->assert(lines.size() == 0, "Le fichier doit etre vide sans message");

    std::remove(filename.c_str());
}

void test::FileAppenderTest::testDestructorFlushes()
{
    const std::string filename = "/tmp/pmx_test_fileappender_4.log";
    std::remove(filename.c_str());

    const logs::Logger & logger = logs::LoggerFactory::logger("FileAppenderTest");

    {
        logs::FileAppender appender(filename);
        appender.writeMessage(logger, logs::Level::INFO, "auto flush");
        // Pas d'appel explicite a flush() : le destructeur doit le faire
    }

    std::vector<std::string> lines = readLines(filename);
    this->assert(lines.size() == 1, "Le destructeur doit flush les messages");
    this->assert(lines[0] == "auto flush", "Le contenu doit etre 'auto flush'");

    std::remove(filename.c_str());
}
