/*!
 * \file
 * \brief Implémentation des tests de la classe logs::SvgAppender.
 */

#include "SvgAppenderTest.hpp"
#include "../../src/common/log/appender/SvgAppender.hpp"
#include "../../src/common/log/Logger.hpp"
#include "../../src/common/log/LoggerFactory.hpp"

#include <fstream>
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

void test::SvgAppenderTest::suite()
{
    testWriteAndFlush();
    testMultipleElements();
    testEmptyFile();
    testDestructorFlushes();
}

void test::SvgAppenderTest::testWriteAndFlush()
{
    const std::string filename = "/tmp/pmx_test_svgappender_1.svg";
    std::remove(filename.c_str());

    const logs::Logger & logger = logs::LoggerFactory::logger("SvgAppenderTest");

    {
        logs::SvgAppender appender(filename);
        appender.writeMessage(logger, logs::Level::INFO, "<circle cx=\"10\" cy=\"10\" r=\"5\" />");
        appender.flush();
    }

    std::vector<std::string> lines = readLines(filename);
    this->assert(lines.size() == 1, "Le fichier doit contenir 1 ligne apres flush");
    this->assert(lines[0] == "<circle cx=\"10\" cy=\"10\" r=\"5\" />",
                 "Le contenu SVG doit etre correct");

    std::remove(filename.c_str());
}

void test::SvgAppenderTest::testMultipleElements()
{
    const std::string filename = "/tmp/pmx_test_svgappender_2.svg";
    std::remove(filename.c_str());

    const logs::Logger & logger = logs::LoggerFactory::logger("SvgAppenderTest");

    {
        logs::SvgAppender appender(filename);
        appender.writeMessage(logger, logs::Level::INFO, "<svg>");
        appender.writeMessage(logger, logs::Level::INFO, "<rect x=\"0\" y=\"0\" width=\"100\" height=\"50\" />");
        appender.writeMessage(logger, logs::Level::INFO, "<line x1=\"0\" y1=\"0\" x2=\"100\" y2=\"50\" />");
        appender.writeMessage(logger, logs::Level::INFO, "</svg>");
        appender.flush();
    }

    std::vector<std::string> lines = readLines(filename);
    this->assert(lines.size() == 4, "Le fichier doit contenir 4 lignes");
    this->assert(lines[0] == "<svg>", "Ligne 1 : ouverture svg");
    this->assert(lines[3] == "</svg>", "Ligne 4 : fermeture svg");

    std::remove(filename.c_str());
}

void test::SvgAppenderTest::testEmptyFile()
{
    const std::string filename = "/tmp/pmx_test_svgappender_3.svg";
    std::remove(filename.c_str());

    {
        logs::SvgAppender appender(filename);
        appender.flush();
    }

    std::vector<std::string> lines = readLines(filename);
    this->assert(lines.size() == 0, "Le fichier doit etre vide sans message");

    std::remove(filename.c_str());
}

void test::SvgAppenderTest::testDestructorFlushes()
{
    const std::string filename = "/tmp/pmx_test_svgappender_4.svg";
    std::remove(filename.c_str());

    const logs::Logger & logger = logs::LoggerFactory::logger("SvgAppenderTest");

    {
        logs::SvgAppender appender(filename);
        appender.writeMessage(logger, logs::Level::INFO, "<text>auto flush</text>");
        // Pas d'appel explicite a flush() : le destructeur doit le faire
    }

    std::vector<std::string> lines = readLines(filename);
    this->assert(lines.size() == 1, "Le destructeur doit flush les messages");
    this->assert(lines[0] == "<text>auto flush</text>", "Le contenu SVG doit etre correct");

    std::remove(filename.c_str());
}
