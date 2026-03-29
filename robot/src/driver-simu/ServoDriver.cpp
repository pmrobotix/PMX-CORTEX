//drivers...SIMU

#include "ServoDriver.hpp"

#include "log/Logger.hpp"

using namespace std;

ServoDriverSimu::ServoDriverSimu() :
        connected_(true)
{
    logger().debug() << "ServoDriver()" << logs::end;
}

bool ServoDriverSimu::is_connected()
{
    return connected_;
}

void ServoDriverSimu::setType(int servo, ServoType)
{

}

void ServoDriverSimu::hold(int servo) // 1 à 8
{
    switch (servo) {

    default:
        break;
    }
}

void ServoDriverSimu::setPulsePos(int servo, int pos, int rate)
{
    switch (servo) {

    default:
        break;
    }
}

void ServoDriverSimu::release(int servo)
{
    switch (servo) {

    default:
        break;
    }
}

void ServoDriverSimu::setTorque(int servo, int millisec)
{
    switch (servo) {

    default:
        break;
    }
}

void ServoDriverSimu::turn(int servo, int speed)
{
}

int ServoDriverSimu::getMoving(int servo)
{
    return -1;
}
int ServoDriverSimu::getPulsePos(int servo)
{
    return -1;
}

int ServoDriverSimu::ping(int servo)
{
    return 1;
}

void ServoDriverSimu::setMinPulse(int servo, int pulse)
{
}

void ServoDriverSimu::setMidPulse(int servo, int pulse)
{
}

void ServoDriverSimu::setMaxPulse(int servo, int pulse)
{
}

void ServoDriverSimu::setPolarity(int servo, bool inversed)
{
}

int ServoDriverSimu::getTorque(int servo)
{
    return -1;
}
