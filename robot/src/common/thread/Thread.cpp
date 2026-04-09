/*!
 * \file
 * \brief Implémentation de la classe Thread (wrapper POSIX pthread).
 */

#include "Thread.hpp"
#include <thread>
#include <sched.h>
#include <iostream>
#include <cerrno>
#include <cstring>
#include <ctime>

void*
utils::Thread::entryPoint(void *pthis)
{
    utils::Thread *pt = (utils::Thread*) pthis;
    pt->setState(utils::STARTED);
    pt->execute();
    pt->setState(utils::STOPPED);
    return NULL;
}

utils::Thread::Thread() :
        threadId_(), state_(utils::CREATED), priority_(0)
{
}

void utils::Thread::yield() {
    ::sched_yield();
}

void utils::Thread::sched_yield() {
    ::sched_yield();
}

void utils::Thread::sleep_for_micros(int64_t usec) {
    std::this_thread::sleep_for(std::chrono::microseconds(usec));
}

void utils::Thread::sleep_for_millis(int64_t msec) {
    std::this_thread::sleep_for(std::chrono::milliseconds(msec));
}

void utils::Thread::sleep_for_secs(int64_t sec) {
    std::this_thread::sleep_for(std::chrono::seconds(sec));
}

bool utils::Thread::start(std::string name, int priority) {
    priority_ = priority;
    name_ = name;
    this->setState(utils::STARTING);

    int code = pthread_create(&threadId_, NULL, utils::Thread::entryPoint, (void*) this);
    if (code == 0) {
        if (priority != 0) {
            utils::set_realtime_priority(priority, name, threadId_);
        }
        return true;
    }
    else {
        std::cerr << "Thread::start FAILED name=" << name << " error=";
        switch (code) {
            case EAGAIN: std::cerr << "EAGAIN (resources)"; break;
            case EINVAL: std::cerr << "EINVAL (attr invalid)"; break;
            case EPERM:  std::cerr << "EPERM (permission)"; break;
            default:     std::cerr << code;
        }
        std::cerr << std::endl;
        return false;
    }
}

// Pour configurer la priorité temps-réel en tant qu'utilisateur :
// Editer /etc/security/limits.conf et ajouter :
//   <user> hard rtprio 99
//   <user> soft rtprio 99
// Puis reboot. Vérifier avec : ulimit -r
int utils::set_realtime_priority(int p, std::string name, ThreadId thread) {
    if (p < 0) return -1;

    int maxPrio = sched_get_priority_max(SCHED_FIFO);
    if (p >= maxPrio) p = maxPrio;

    struct sched_param params;
    params.sched_priority = p;

    int ret = pthread_setschedparam(thread, SCHED_FIFO, &params);
    if (ret != 0) {
        std::cerr << "Thread " << name << " set_realtime_priority FAILED: "
                  << std::strerror(errno) << std::endl;
        return -2;
    }

    // Vérification
    int policy = 0;
    ret = pthread_getschedparam(thread, &policy, &params);
    if (ret != 0) {
        std::cerr << "Thread " << name << " get_realtime_priority FAILED" << std::endl;
        return -3;
    }
    if (policy != SCHED_FIFO) {
        std::cerr << "Thread " << name << " scheduling is NOT SCHED_FIFO!" << std::endl;
    }

    return params.sched_priority;
}

void utils::sleep_for_micros(int64_t usec) {
    std::this_thread::sleep_for(std::chrono::microseconds(usec));
}

void utils::sleep_for_millis(int64_t msec) {
    std::this_thread::sleep_for(std::chrono::milliseconds(msec));
}

void utils::sleep_for_secs(int64_t sec) {
    std::this_thread::sleep_for(std::chrono::seconds(sec));
}

