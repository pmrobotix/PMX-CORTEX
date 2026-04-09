//drivers...OPO

#include "SensorsDriver.hpp"
#include "HardwareConfig.hpp"
#include "../driver-simu/SensorsDriver.hpp"

#include <string>
#include <vector>

#include "log/Logger.hpp"

class ARobotPositionShared;

using namespace std;

ASensorsDriver* ASensorsDriver::create(std::string, ARobotPositionShared *robotpos)
{
	if (!HardwareConfig::instance().isEnabled("SensorsDriver")) {
		return new SensorsDriverSimu(robotpos);
	}
	return new SensorsDriver(robotpos);
}

SensorsDriver::SensorsDriver(ARobotPositionShared *robotpos) :
		beaconSensors_(0, ADDRESS_BeaconSensors), connected_gp2y0e02b_(false) //, gp2_1_(0, ADDRESS_gp2y0e02b), gp2_2_(1, ADDRESS_gp2y0e02b)
{

	beacon_connected_ = beaconSensors_.begin(settings_);

	regs_ = { };
	settings_ = { }; //TODO à ecrire/initialiser en i2c?
	vadv_.clear();
}

SensorsDriver::~SensorsDriver()
{
}

void SensorsDriver::displayNumber(int number)
{
	beaconSensors_.display(number);
}

bool SensorsDriver::is_connected()
{
	return beacon_connected_;
}

int SensorsDriver::getAnalogPinData() //TODO DEPRECATED
{
	return -1;
}

ASensorsDriver::bot_positions SensorsDriver::getvPositionsAdv()
{
	msync_.lock();
	bot_positions tmp = vadv_;
	msync_.unlock();
	return tmp;
}
int SensorsDriver::sync()
{
	// 1. Lecture I2C SANS lock (peut bloquer si beacon ne repond pas)
	uint8_t flags = beaconSensors_.readFlag();
	if (flags == 0xFF) {
		logger().error() << "sync() readFlag error I2C" << logs::end;
		return -1;
	}
	if (!(flags & 0x01)) {
		// 0x80 = alive, pas de nouvelles donnees
		return 0;
	}

	// 0x81 = nouvelles donnees disponibles, lecture complete
	Registers regs = beaconSensors_.getData();
	if (regs.flags == 0xFF) {
		logger().error() << "sync() getData error I2C" << logs::end;
		return -1;
	}

	// 2. Mise a jour des donnees partagees sous lock (bref)
	msync_.lock();
	regs_ = regs;
	vadv_.clear();

	if (regs.nbDetectedBots >= 1)
	{
		vadv_.push_back(RobotPos((int) regs.nbDetectedBots, regs.x1_mm, regs.y1_mm, regs.a1_deg, regs.d1_mm, regs.t1_us));
	}
	if (regs.nbDetectedBots >= 2)
	{
		vadv_.push_back(RobotPos((int) regs.nbDetectedBots, regs.x2_mm, regs.y2_mm, regs.a2_deg, regs.d2_mm, regs.t2_us));
	}
	if (regs.nbDetectedBots >= 3)
	{
		vadv_.push_back(RobotPos((int) regs.nbDetectedBots, regs.x3_mm, regs.y3_mm, regs.a3_deg, regs.d3_mm, regs.t3_us));
	}
	if (regs.nbDetectedBots >= 4)
	{
		vadv_.push_back(RobotPos((int) regs.nbDetectedBots, regs.x4_mm, regs.y4_mm, regs.a4_deg, regs.d4_mm, regs.t4_us));
	}

	logger().debug() << "beacon seq:" << regs.seq << " t1:" << regs.t1_us << "us" << logs::end;

	msync_.unlock();

	return 1; // nouvelles donnees lues
}

void SensorsDriver::addvPositionsAdv(float x, float y)
{
	vadv_.push_back(RobotPos(1, x, y, 0.0f, 0.0f));
}
void SensorsDriver::clearPositionsAdv()
{
	vadv_.clear();
}

int SensorsDriver::leftSide()
{
	//TODO Sharp Gp2y0e02b non connecte => retourne "pas d'obstacle"
	return 400;
}
int SensorsDriver::rightSide()
{
	//TODO Sharp Gp2y0e02b non connecte => retourne "pas d'obstacle"
	return 400;
}
//
//int SensorsDriver::getFrontDistMmFromObject(int diagonal_dist_mm) //TODO position xy ?
//{
//    int dist_from_wheel_axe_mm = 155;
//    float cos_alpha = 45.0/48.0;
//    int lmm = diagonal_dist_mm * cos_alpha + dist_from_wheel_axe_mm;
//    return lmm;
//}
//
//int SensorsDriver::getBackDistMmFromObject(int diagonal_dist_mm)
//{
//    int dist_from_wheel_axe_mm = 70;
//    float cos_alpha = 42.0/48.0;
//    int lmm = diagonal_dist_mm * cos_alpha + dist_from_wheel_axe_mm;
//    return lmm;
//}

//retourne la distance minimum du dernier sync
int SensorsDriver::frontLeft()
{
	//TODO filtrage minimum a faire ici aussi?

	//int res = -1;
	//msync_.lock();
	//if (regs_.c1_mm == 0) res = -1;
	//if (regs_.c2_mm == 0) res = -1;

	if (regs_.c2_mm == 0) return 500;

	//seuil minimum
	if (regs_.c2_mm > 500) //regs_.c1_mm > 30 &&
	{
//        //int dist = 0;
//        if (regs_.c1_mm < regs_.c2_mm) res = regs_.c1_mm;
//        if (regs_.c1_mm >= regs_.c2_mm) res = regs_.c2_mm;
//    }else
//    {
		//msync_.unlock();
		return 500;
	}

	//msync_.unlock();
	return regs_.c2_mm;
}

int SensorsDriver::frontCenter()
{
	//int dist_from_wheel_axe_mm = 140;
	//return irCenter_.getDistanceMm() + dist_from_wheel_axe_mm;
	return -1;
}
int SensorsDriver::frontRight()
{
//TODO filtrage minimum a faire ici aussi?

//    int res = -1;
//    msync_.lock();
//    if (regs_.c3_mm == 0) res = -1;
//    if (regs_.c4_mm == 0) res = -1;
//    //int dist = 0;
//    if (regs_.c3_mm < regs_.c4_mm) res = regs_.c3_mm;
//    if (regs_.c3_mm >= regs_.c4_mm) res = regs_.c4_mm;
//    msync_.unlock();
//    return res;
//    int res = -1;
//    msync_.lock();

	if (regs_.c4_mm == 0) return 500;

	//seuil minimum
	if (regs_.c4_mm > 500)
	{

		return 500;
	}

	//msync_.unlock();
	return regs_.c4_mm;
}

int SensorsDriver::backLeft()
{
	// c6_mm = AR GAUCHE HAUT (ToF zone arriere gauche)
	if (regs_.c6_mm == 0) return 500;
	if (regs_.c6_mm > 500) return 500;
	return regs_.c6_mm;
}
int SensorsDriver::backCenter()
{
	// Pas de capteur ToF centre arriere dedie, utilise la balise (beacon)
	return -1;
}
int SensorsDriver::backRight()
{
	// c8_mm = AR DROIT HAUT (ToF zone arriere droite)
	if (regs_.c8_mm == 0) return 500;
	if (regs_.c8_mm > 500) return 500;
	return regs_.c8_mm;
}
