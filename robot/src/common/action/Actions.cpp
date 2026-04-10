#include "Actions.hpp"

#include <unistd.h>

#include "log/Logger.hpp"

Actions::Actions()
{
}

void Actions::start()
{
    actionScheduler_.start("ActionTimerScheduler", 60);

    while (!is_started())
    {
        utils::sleep_for_micros(10000);
        std::this_thread::yield();
    }
}

bool Actions::is_started()
{
    return (actionScheduler_.state() == utils::STARTED);
}

void Actions::clearAll()
{
    actionScheduler_.clearActions();
    actionScheduler_.clearTimers();
}

void Actions::cancel()
{
    actionScheduler_.cancel();
}

void Actions::waitAndStopManagers()
{
    if (actionScheduler_.state() == utils::STARTED)
        actionScheduler_.stop();
}
