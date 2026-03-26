/*!
 * \file
 * \brief Implementation ARM du driver d'actions.
 */

#include "ActionDriver.hpp"

AActionDriver * AActionDriver::create(int nb)
{
    return new ActionDriver(nb);
}

ActionDriver::ActionDriver(int nb)
{
}

ActionDriver::~ActionDriver()
{
}

void AActionDriver::function(int value)
{
    //drivers OPO
}
