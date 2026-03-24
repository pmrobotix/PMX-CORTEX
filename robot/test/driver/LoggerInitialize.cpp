/*!
 * \file
 * \brief Initialisation du systeme de Log pour les tests driver.
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
