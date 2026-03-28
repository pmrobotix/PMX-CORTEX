#include "Actions.hpp"

#include <unistd.h>

#include "log/Logger.hpp"

Actions::Actions()
{
}

void Actions::start()
{
    actionManagerTimer_.start("ActionManagerTimer", 60);

    while (!is_started())
    {
        utils::sleep_for_micros(10000);
        //printf(" wait actionManagerTimer_.start \n");
        std::this_thread::yield();
    }
}

bool Actions::is_started()
{
    if (actionManagerTimer_.state() == utils::STARTED)
        return true;
    else
        return false;
}

void Actions::clearAll()
{
    actionManagerTimer_.clearActions();
    actionManagerTimer_.clearTimers();
}

void Actions::cancel()
{
    actionManagerTimer_.cancel();
}

void Actions::waitAndStopManagers()
{
    if (actionManagerTimer_.state() == utils::STARTED)
        //Attente de a fin de tous les timers
        actionManagerTimer_.stop();
}
