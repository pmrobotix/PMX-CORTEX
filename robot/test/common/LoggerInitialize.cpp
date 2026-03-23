/*!
 * \file
 * \brief Initialisation du système de Log pour les tests unitaires.
 */

#include "../../src/common/log/Level.hpp"
#include "../../src/common/log/LoggerFactory.hpp"
#include "../suite/UnitTestAppender.hpp"

void logs::LoggerFactory::initialize()
{
    setPriority(1);
    this->add("console", new UnitTestAppender());
    add(logs::Level::DEBUG, "", "console");
}
