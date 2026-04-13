#include "Sensors.hpp"

#include <stdlib.h>

#include <algorithm>
#include <vector>

#include "log/Logger.hpp"
#include "log/SvgWriter.hpp"
#include "thread/Thread.hpp"
#include "asserv/Asserv.hpp"
#include "interface/ARobotPositionShared.hpp"
#include "geometry/TableGeometry.hpp"
#include "Actions.hpp"
//#include "../../Log/simple_svg_1.0.0_cho.hpp"
//#include "simple_svg_1.0.0.hpp"

using namespace std;

Sensors::Sensors(Actions &actions, Robot *robot) :
		AActionsElement(actions), robot_(robot)

{
	sensorsdriver_ = ASensorsDriver::create(robot->getID(), robot->sharedPosition());

	// ObstacleZone est initialise par son constructeur (tout a zero/false)

	x_adv_mm = -1.0;
	y_adv_mm = -1.0;

	opponents_last_positions.clear();

	/*
	 int xdim = 3400;
	 int ydim = 2500;

	 svg::Dimensions dimensions(xdim, ydim);
	 svg::Layout lay(dimensions, svg::Layout::TopLeft);

	 doc_ = new svg::Document("sensors", lay);
	 //svg::Document doc("sensors", lay);

	 *doc_ << svg::elemStart("g") << svg::attribute("transform", "translate(200,2200) scale(1,-1)")
	 << svg::emptyElemEnd(false);

	 // Red image border.
	 svg::Polygon border(svg::Fill(svg::Color::White), svg::Stroke(5, svg::Color::Red));
	 border << svg::Point(xdim, ydim) << svg::Point(dimensions.width, ydim)
	 << svg::Point(dimensions.width, dimensions.height) << svg::Point(xdim, dimensions.height);
	 *doc_ << border;

	 //*doc_ << svg::elemStart("g");
	 */

}
Sensors::~Sensors()
{

	//loggerSvg().info() << doc->endToString() << logs::end;
	delete sensorsdriver_;
}

//void Sensors::toSVG()
//{

//     int xdim = 3400;
//     int ydim = 2500;
//
//     svg::Dimensions dimensions(xdim, ydim);
//     svg::Layout lay(dimensions, svg::Layout::TopLeft);
//
//     svg::Document doc("sensors", lay);

//     doc << svg::elemStart("g") << svg::attribute("transform", "translate(200,2200) scale(1,-1)")
//     << svg::emptyElemEnd(false);
//
//     // Red image border.
//     svg::Polygon border(svg::Fill(svg::Color::White), svg::Stroke(5, svg::Color::Red));
//     border << svg::Point(xdim, ydim) << svg::Point(dimensions.width, ydim)
//     << svg::Point(dimensions.width, dimensions.height) << svg::Point(xdim, dimensions.height);
//     doc << border;

//*doc_ << svg::elemEnd("g");
//loggerSvg().info() << doc_->appendToString() << logs::end;
/*
 doc << svg::elemEnd("g");

 loggerSvg().info() << doc.toString() << logs::end;
 */
//loggerSvg().info() << "INIT " << logs::end;
//}
SensorsThread::SensorsThread(Sensors &sensors, int period_ms) :
		sensors_(sensors),
		period_us_(period_ms * 1000),
		stopRequested_(false)
{
	lastdetect_front_level_ = 0;
	lastdetect_back_level_ = 0;

	nb_ensurefront4 = 0;
	nb_ensureback4 = 0;

	nb_sensor_front_a_zero = 0;
	nb_sensor_back_a_zero = 0;
}

void SensorsThread::execute()
{
	logger().info() << "SensorsThread started, period=" << (period_us_ / 1000) << "ms" << logs::end;
	utils::PeriodicTimer timer(period_us_);

	while (!stopRequested_)
	{
		sensorOnTimer();
		timer.sleep_until_next();
	}

	logger().info() << "SensorsThread stopped" << logs::end;
	setState(utils::STOPPED);
}

void SensorsThread::stopThread()
{
	stopRequested_ = true;
}


ASensorsDriver::bot_positions Sensors::setPositionsAdvByBeacon()
{
	//recupere les données qui ont ete enregistrées par le sync
	opponents_last_positions = sensorsdriver_->getvPositionsAdv();
	return opponents_last_positions;
}

void Sensors::clearPositionsAdv()
{
	sensorsdriver_->clearPositionsAdv();
}

void Sensors::display(int n)
{
	sensorsdriver_->displayNumber(n);
}

void Sensors::writeLedLuminosity(uint8_t lum)
{
	sensorsdriver_->writeLedLuminosity(lum);
}

//is connected and alive
bool Sensors::is_connected()
{
	return sensorsdriver_->is_connected();
}
//
////used in is_connected
//bool Sensors::is_alive()
//{
//    return sensorsdriver_->is_alive();
//}

int Sensors::sync(std::string sensorname)
{
	//synchronise les données sur les sensors drivers
	if (sensorname == "beacon_sync")
	{
		return sensorsdriver_->sync(); //renvoi -1 si erreur
	}

	if (sensorname == "right")
	{
		multipleRightSide(10);
	}

	if (sensorname == "left")
	{
		multipleLeftSide(10);
	}

	if (sensorname == "fL")
	{
		return sensorsdriver_->frontLeft(); //renvoi la distance mini
	}
//    if (sensorname == "fC") {
//        return sensorsdriver_->frontCenter();
//    }
	if (sensorname == "fR")
	{
		return sensorsdriver_->frontRight(); //renvoi la distance mini
	}
	if (sensorname == "bL")
	{
		return sensorsdriver_->backLeft(); //renvoi la distance mini
	}
//    if (sensorname == "bC") {
//        return sensorsdriver_->backCenter();
//    }
	if (sensorname == "bR")
	{
		return sensorsdriver_->backRight(); //renvoi la distance mini
	}

//    if (sensorname == "beaconADV") {
////        ASensorsDriver::bot_positions
////            return sensorsdriver_->getvPositionsAdv();
//        }
	return -1;
}

float Sensors::multipleRightSide(int nb)
{
	int data[nb];
	float moy = 0.0;
	for (int ii = 0; ii < nb; ii++)
	{
		data[ii] = rightSide();
		//logger().debug() << "Right= " << data[ii] << logs::end;
		utils::sleep_for_micros(40000);
	}

	//Now we call the sort function
	std::sort(data, data + nb);

//    for (int ii = 0; ii < nb; ii++) {
//        logger().info() << "rtrie= " << data[ii] << logs::end;
//    }

	moy = (data[3] + data[4] + data[5] + data[6]) / 4;

	return moy;
}

float Sensors::multipleLeftSide(int nb)
{
	int data[nb];

	float moy = 0.0;
	for (int ii = 0; ii < nb; ii++)
	{
		data[ii] = leftSide();
		logger().debug() << "Left= " << data[ii] << logs::end;
		utils::sleep_for_micros(40000);
	}

	//Now we call the sort function
	std::sort(data, data + nb);

	for (int ii = 0; ii < nb; ii++)
	{
		logger().info() << "ltrie= " << data[ii] << logs::end;

	}

	moy = (data[3] + data[4] + data[5] + data[6]) / 4;

	return moy;
}

int Sensors::rightSide()
{
	return sensorsdriver_->rightSide();
}
int Sensors::leftSide()
{
	return sensorsdriver_->leftSide();
}

// sur le repere robot uniquement
// axe y devant le robot
// axe x sur la droite du robot

//filtre is_in_front devant ou coté
//0  0  0
//3  3  3  // level 1 obstacleZone_.frontCenterThreshold()
//2G 4  1D // level 2 obstacleZone_.frontCenterVeryClosedThreshold() et back enterVeryClosedThreshold_
//3  3  3  // level 1 obstacleZone_.backCenterThreshold()
//0  0  0

//TODO NiceTOhave filtre is_in_front and back
//99       9/99    99       // 1 jusqu'à 9 vecteur robot adv nous rentre dedans, vitesse reduite
//3/30     1       2/20     // level 1 obstacleZone_.frontCenterThreshold()
//97/-97G  0       98/-98D      // level 0 obstacleZone_.frontCenterVeryClosedThreshold() => 0 arret complet
//-3/-30   -1      -2/-20     // 2 ou 3 si vecteur entrant, 20 ou 30 si le robot adv s'en va;
//-99      -9/-99  -99       //possibilité de créer new level si on veut avec plus de threshold

//les coordonnées x_adv_mm, y_adv_mm sont sur le repère robot

//retourne 0, sinon le niveau detecté 2 veryClosed, 1 first level
int Sensors::front(bool display)
{
	//logger().error() << "FRONT!!!!" << logs::end;
	//on recupere les distances de detection gauche et droite
	int fL = sync("fL");
	int fR = sync("fR");

	ASensorsDriver::bot_positions vpos;

	int tfMin = 9999;

	logger().debug() << "enable L=" << obstacleZone_.enableFrontLeft() << " C=" << obstacleZone_.enableFrontCenter() << " R=" << obstacleZone_.enableFrontRight()
			<< logs::end;

	int level = 0;

	if (obstacleZone_.enableFrontLeft()) //existance
	{
		bool fL_filter = this->robot()->tableGeometry()->isSensorReadingInsideTable(fL, -1); //negatif = capteur placé à gauche
		//logger().info() << " fL_filter= " << fL_filter << logs::end;
		if (fL_filter)
		{
			{
				if ((!obstacleZone_.ignoreFrontLeft() && (fL < obstacleZone_.frontLeftThreshold())))
				{
					if (display) logger().debug() << "1 frontLeft= " << fL << logs::end;

					if (fL > 60) if (tfMin > fL) tfMin = fL;

					level = 3;
				}
				if ((!obstacleZone_.ignoreFrontLeft() && (fL < obstacleZone_.frontLeftVeryClosedThreshold())))
				{
					if (display) logger().debug() << "2 frontLeft= " << fL << logs::end;
					level = 4;
				}
			}
		}
	}

	if (obstacleZone_.enableFrontRight())
	{
		bool fR_filter = this->robot()->tableGeometry()->isSensorReadingInsideTable(fR, 1);
		if (fR_filter)
		{
			if ((!obstacleZone_.ignoreFrontRight() && (fR < obstacleZone_.frontRightThreshold())))
			{
				if (display) logger().debug() << "1 frontRight= " << fR << logs::end;
				//                tfR = fR;
				if (fR > 60) if (tfMin > fR) tfMin = fR;
				level = 3;
			}
			if ((!obstacleZone_.ignoreFrontRight() && (fR < obstacleZone_.frontRightVeryClosedThreshold())))
			{
				if (display) logger().debug() << "2 frontRight= " << fR << logs::end;
				level = 4;
			}
		}
	}

	if (getAvailableFrontCenter())
	{
		// detection avec la balise (on se sert de la variable du capteur au centre)
		vpos = Sensors::setPositionsAdvByBeacon();
		int level_filtered = false;
		bool inside_table = true;
		float x_pos_adv_table = -1;
		float y_pos_adv_table = -1;
		int nb = 0;

		for (auto botpos : vpos)
		{
			nb++;

			logger().debug() << __FUNCTION__ << " " << nb << " bots=" << botpos.nbDetectedBots << " x,y,deg= "
					<< botpos.x << ", " << botpos.y << ", " << botpos.theta_deg << logs::end;

			// Projection beacon→table avec posAtSync (position robot au moment du sync I2C)
			ROBOTPOSITION posAtSync = {lastDetection_.x_robot_mm, lastDetection_.y_robot_mm,
					lastDetection_.theta_robot_rad, 0, 0, 0};
			// Convention beacon : 0° = devant du robot, pas de décalage -PI/2
			float a = (posAtSync.theta + (botpos.theta_deg * M_PI / 180.0f));
			a = WrapAngle2PI(a);
			x_pos_adv_table = posAtSync.x + (botpos.d * cos(a));
			y_pos_adv_table = posAtSync.y + (botpos.d * sin(a));
			ROBOTPOSITION pos_robot_instantane = posAtSync;

			//filtre sur la table avec transformation de repere
			inside_table = this->robot()->tableGeometry()->isPointInsideTable((int) x_pos_adv_table,
					(int) y_pos_adv_table);

			// LOG DEBUG : afficher la projection meme quand elle est filtree
			logger().debug() << __FUNCTION__ << " PROJ robot=(" << posAtSync.x << "," << posAtSync.y
					<< "," << (posAtSync.theta * 180.0f / M_PI) << "deg)"
					<< " beacon=(d=" << botpos.d << ",theta=" << botpos.theta_deg << "deg)"
					<< " a_table=" << (a * 180.0f / M_PI) << "deg"
					<< " adv_table=(" << x_pos_adv_table << "," << y_pos_adv_table << ")"
					<< " inside=" << inside_table << logs::end;

			if (!obstacleZone_.removeOutsideTable())
			{
				inside_table = true;
			}

			if (inside_table)
			{
				//affichage gris (sans detection de seuils)
				this->robot()->svgw().writePosition_AdvPos(x_pos_adv_table, y_pos_adv_table, pos_robot_instantane.x,
						pos_robot_instantane.y, 0);

				//filtre is_in_front devant ou coté
				//0  0  0
				//3  3  3  // level 1 obstacleZone_.frontCenterThreshold()
				//2G 4  1D // level 2 obstacleZone_.frontCenterVeryClosedThreshold() et obstacleZone_.backCenterVeryClosedThreshold()
				//-3  -3  -3 // // level 1 obstacleZone_.backCenterThreshold()
				//0  0  0




				//rayon robot + espace coté + rayon adv //TODO thresholdLR = 140 + 20 + 250;???
				int thresholdLR = 140 + 20 + 250;




				level_filtered = this->filtre_levelInFront(thresholdLR, obstacleZone_.frontCenterThreshold(),
						obstacleZone_.frontCenterVeryClosedThreshold(), botpos.d, botpos.x, botpos.y, botpos.theta_deg);
				logger().debug() << __FUNCTION__ << " " << nb << " nbbots=" << botpos.nbDetectedBots
						<< " level_filtered= " << level_filtered << logs::end;

				// Capture position adversaire pour DetectionEvent
				if (level_filtered >= 1)
				{
					x_adv_mm = x_pos_adv_table;
					y_adv_mm = y_pos_adv_table;
					lastDetection_.d_adv_mm = botpos.d;
					lastDetection_.x_robot_mm = pos_robot_instantane.x;
					lastDetection_.y_robot_mm = pos_robot_instantane.y;
					lastDetection_.theta_robot_rad = pos_robot_instantane.theta;
				}

				if (level_filtered == 1)
				{
					// DROITE
					if (display)
						logger().debug() << level_filtered << " DROITE frontCenter xy= " << botpos.x << " " << botpos.y
								<< logs::end;
					level = 1;
					obstacleZone_.setDetectedFrontRight(true);
				} else if (level_filtered == 2)
				{
					// GAUCHE
					if (display)
						logger().debug() << level_filtered << " GAUCHE frontCenter xy= " << botpos.x << " " << botpos.y
								<< logs::end;
					level = 2;
					obstacleZone_.setDetectedFrontLeft(true);

				} else if (level_filtered == 3)
				{
					level = 3;
					if (display)
						logger().debug() << level_filtered << " frontCenter xy= " << botpos.x << " " << botpos.y
								<< logs::end;
					if (x_pos_adv_table != -1 && y_pos_adv_table != -1)
						this->robot()->svgw().writePosition_AdvPos(x_pos_adv_table, y_pos_adv_table,
								pos_robot_instantane.x, pos_robot_instantane.y, 1);

				} else if (level_filtered == 4)
				{ //seuil de level 4 ARRET
					level = 4;
					if (x_pos_adv_table != -1 && y_pos_adv_table != -1)
						this->robot()->svgw().writePosition_AdvPos(x_pos_adv_table, y_pos_adv_table,
								pos_robot_instantane.x, pos_robot_instantane.y, 2);
					if (display)
						logger().debug() << level_filtered << " frontCenter xy= " << botpos.x << " " << botpos.y
								<< logs::end;
				}
			} else
			{
				if (display)
					logger().debug() << inside_table << " NOT INSIDE TABLE frontCenter xy=" << botpos.x << " "
							<< botpos.y << logs::end;
			}
			// on traite un seul robot par boucle //TODO a garder ? donc un tous les 200ms ? bof bof
			//TODO creer le vecteur d'approche avec l'ancienne valeur
//            if (botpos.nbDetectedBots == 1)
//                break;
		}

//TODO : a reinitialiser apres 1sec
//                        obstacleZone_.setDetectedFrontRight( = false;
//                        adv_is_detected_front_left_ = false;

	}
//
//	//Mise à jour de la position de l'adversaire via les capteurs de proximité
//	if (level >= 1)
//	{
//		//logger().info() << "front  level >=1 tfMin= " << tfMin << logs::end;
//		if (tfMin != 0)
//		{
//			//au centre
//			//logger().info() << "front         tfMin= " << tfMin << logs::end;
//			x_adv_mm = (float) tfMin;
//			y_adv_mm = 0.0;
//		} else
//		{
//			x_adv_mm = -1.0;
//			y_adv_mm = -1.0;
//		}
//	}

	// Publication du DetectionEvent (front)
	lastDetection_.frontLevel = level;
	lastDetection_.x_adv_mm = x_adv_mm;
	lastDetection_.y_adv_mm = y_adv_mm;
	if (level >= 1) {
		lastDetection_.valid = true;
	}

	return level;
}

int Sensors::back(bool display)
{
	//on recupere les distances de detection
	int bL = sync("bL");
	int bR = sync("bR");

	int tfMin = 9999;
	int level = 0;

	ASensorsDriver::bot_positions vpos;

	if (obstacleZone_.enableBackLeft())
	{
		bool bL_filter = this->robot()->tableGeometry()->isSensorReadingInsideTable(-bL, -1);
		if (bL_filter)
		{ //negatif = capteur placé à gauche
			if ((!obstacleZone_.ignoreBackLeft() && (bL < obstacleZone_.backLeftThreshold())))
			{
				if (display) logger().info() << "1 backLeft= " << bL << logs::end;
				//tfMin = bL;
				if (bL > 60) if (tfMin > bL) tfMin = bL;
				level = 3;
			}
			if ((!obstacleZone_.ignoreBackLeft() && (bL < obstacleZone_.backLeftVeryClosedThreshold())))
			{
				if (display) logger().info() << "2 backLeft= " << bL << logs::end;
				level = 4;
			}
		}
	}

	if (obstacleZone_.enableBackRight())
	{
		bool bR_filter = this->robot()->tableGeometry()->isSensorReadingInsideTable(-bR, 1);
		if (bR_filter)
		{
			if ((!obstacleZone_.ignoreBackRight() && (bR < obstacleZone_.backRightThreshold())))
			{
				if (display) logger().info() << "1 backRight= " << bR << logs::end;
				if (bR > 60) if (tfMin > bR) tfMin = bR;
				level = 3;
			}
			if ((!obstacleZone_.ignoreBackRight() && (bR < obstacleZone_.backRightVeryClosedThreshold())))
			{
				if (display) logger().info() << "2 backRight= " << bR << logs::end;
				level = 4;
			}
		}
	}

	if (getAvailableBackCenter())
	{
		// detection avec la balise (on se sert de la variable du capteur au centre)
		vpos = Sensors::setPositionsAdvByBeacon();
		int level_filtered = false;
		bool inside_table = true;
		float x_pos_adv_table = -1;
		float y_pos_adv_table = -1;
		int nb = 0;

		for (auto botpos : vpos)
		{
			nb++;

			logger().debug() << __FUNCTION__ << " " << nb << " bots=" << botpos.nbDetectedBots << " x,y,deg= "
					<< botpos.x << ", " << botpos.y << ", " << botpos.theta_deg << logs::end;

			// Projection beacon→table avec posAtSync (position robot au moment du sync I2C)
			ROBOTPOSITION posAtSync = {lastDetection_.x_robot_mm, lastDetection_.y_robot_mm,
					lastDetection_.theta_robot_rad, 0, 0, 0};
			// Convention beacon : 0° = devant du robot, pas de décalage -PI/2
			float a = (posAtSync.theta + (botpos.theta_deg * M_PI / 180.0f));
			a = WrapAngle2PI(a);
			x_pos_adv_table = posAtSync.x + (botpos.d * cos(a));
			y_pos_adv_table = posAtSync.y + (botpos.d * sin(a));
			ROBOTPOSITION pos_robot_instantane = posAtSync;

			//filtre sur la table avec transformation de repere
			inside_table = this->robot()->tableGeometry()->isPointInsideTable((int) x_pos_adv_table,
					(int) y_pos_adv_table);
			if (!obstacleZone_.removeOutsideTable())
			{
				inside_table = true;
			}

			if (inside_table)
			{
				//affichage gris
				this->robot()->svgw().writePosition_AdvPos(x_pos_adv_table, y_pos_adv_table, pos_robot_instantane.x,
						pos_robot_instantane.y, 0);

				//filtre is_in_front devant ou coté
				//0  0  0
				//3  3  3  // level 1 obstacleZone_.frontCenterThreshold()
				//-2G -4  -1D // level 2 obstacleZone_.frontCenterVeryClosedThreshold() et obstacleZone_.backCenterVeryClosedThreshold()
				//-3  -3  -3 // // level 1 obstacleZone_.backCenterThreshold() //TODO -3 ??? ou bien 4 et mettre 5 au milieu
				//0  0  0



				//rayon robot + espace coté + rayon adv //TODO thresholdLR = 140 + 20 + 250;???
				int thresholdLR = 140 + 20 + 250;





				level_filtered = this->filtre_levelInBack(thresholdLR, obstacleZone_.backCenterThreshold(),
						obstacleZone_.backCenterVeryClosedThreshold(), botpos.d, botpos.x, botpos.y, botpos.theta_deg);
//				logger().error() << __FUNCTION__ << " BACKWARD___ " << nb << " nbbots=" << botpos.nbDetectedBots
//						<< " level_filtered= " << level_filtered << logs::end;

				// Capture position adversaire pour DetectionEvent
				if (level_filtered <= -1)
				{
					x_adv_mm = x_pos_adv_table;
					y_adv_mm = y_pos_adv_table;
					lastDetection_.d_adv_mm = botpos.d;
					lastDetection_.x_robot_mm = pos_robot_instantane.x;
					lastDetection_.y_robot_mm = pos_robot_instantane.y;
					lastDetection_.theta_robot_rad = pos_robot_instantane.theta;
				}

				if (level_filtered == -1)
				{
					// DROITE
					if (display)
						logger().debug() << level_filtered << " DROITE backCenter xy= " << botpos.x << " " << botpos.y
								<< logs::end;
					level = -1;
					obstacleZone_.setDetectedFrontRight(true);
				} else if (level_filtered == -2)
				{
					// GAUCHE
					if (display)
						logger().debug() << level_filtered << " GAUCHE backCenter xy= " << botpos.x << " " << botpos.y
								<< logs::end;
					level = -2;
					obstacleZone_.setDetectedFrontLeft(true);

				} else if (level_filtered == -3)
				{
					level = -3;
					if (display)
						logger().debug() << level_filtered << " backCenter xy= " << botpos.x << " " << botpos.y
								<< logs::end;
					if (x_pos_adv_table != -1 && y_pos_adv_table != -1)
						this->robot()->svgw().writePosition_AdvPos(x_pos_adv_table, y_pos_adv_table,
								pos_robot_instantane.x, pos_robot_instantane.y, 1);

				} else if (level_filtered == -4)
				{ //seuil de level 4 ARRET
					level = -4;
					if (x_pos_adv_table != -1 && y_pos_adv_table != -1)
						this->robot()->svgw().writePosition_AdvPos(x_pos_adv_table, y_pos_adv_table,
								pos_robot_instantane.x, pos_robot_instantane.y, 2);
					if (display)
						logger().debug() << level_filtered << " backCenter xy= " << botpos.x << " " << botpos.y
								<< logs::end;
				}
			} else
			{
				if (display)
					logger().debug() << inside_table << " NOT INSIDE TABLE backCenter xy=" << botpos.x << " "
							<< botpos.y << logs::end;
			}

		}

		//TODO : a reinitialiser apres 1sec
		//                        obstacleZone_.setDetectedFrontRight( = false;
		//                        adv_is_detected_front_left_ = false;

	}

	/*
	 if (obstacleZone_.enableBackLeft())
	 if (this->robot()->asserv()->filtre_IsInsideTable(-bL, -1)) {
	 if ((!obstacleZone_.ignoreBackLeft() && (bL < obstacleZone_.backLeftVeryClosedThreshold()))) {
	 logger().info() << "2 backLeft= " << bL << logs::end;
	 level = 2;
	 }
	 }
	 if (obstacleZone_.enableBackCenter())
	 if (this->robot()->asserv()->filtre_IsInsideTable(-bC, 0)) {
	 if ((!ignoreBackCenter_ && (bC < obstacleZone_.backCenterVeryClosedThreshold()))) {
	 logger().info() << "2 backCenter= " << bC << logs::end;
	 level = 2;
	 }
	 }
	 if (obstacleZone_.enableBackRight())
	 if (this->robot()->asserv()->filtre_IsInsideTable(-bR, 1)) {
	 if ((!obstacleZone_.ignoreBackRight() && (bR < obstacleZone_.backRightVeryClosedThreshold()))) {
	 logger().info() << "2 backRight= " << bR << logs::end;
	 level = 2;
	 }
	 }*/
	/*
	 //Mise à jour de la position de l'adversaire
	 if (level >= 1) {
	 if (tfMin != 0) {
	 //au centre
	 x_adv_mm = -tfMin;
	 y_adv_mm = 0.0;
	 }
	 }*/

////Mise à jour de la position de l'adversaire via les capteurs de proximité
//	if (level >= 3)
//	{
//		//logger().info() << "back  level >=1 tfMin= " << tfMin << logs::end;
//		if (tfMin != 0)
//		{
//			//au centre
//			//logger().info() << "back         tfMin= " << tfMin << logs::end;
//			x_adv_mm = (float) -tfMin;
//			y_adv_mm = 0.0;
//		} else
//		{
//			x_adv_mm = -1.0;
//			y_adv_mm = -1.0;
//		}
//	}
	/*
	 //Mise à jour de la position de l'adversaire via les capteurs de proximité
	 if (level >= 1)
	 {
	 //logger().info() << "back  level >=1 tfMin= " << tfMin << logs::end;
	 if (tfMin != 0)
	 {
	 //au centre
	 //logger().info() << "back         tfMin= " << tfMin << logs::end;
	 x_adv_mm = (float) -tfMin;
	 y_adv_mm = 0.0;
	 } else
	 {
	 x_adv_mm = -1.0;
	 y_adv_mm = -1.0;
	 }
	 }
	 */
	// Publication du DetectionEvent (back)
	lastDetection_.backLevel = level;
	lastDetection_.x_adv_mm = x_adv_mm;
	lastDetection_.y_adv_mm = y_adv_mm;
	if (level <= -1) {
		lastDetection_.valid = true;
	}

	return level;
}

void Sensors::startSensorsThread(int period_ms)
{
	if (sensorsThread_ != nullptr)
	{
		logger().debug() << "SensorsThread already running, stopping first" << logs::end;
		stopSensorsThread();
	}

	logger().debug() << "startSensorsThread period=" << period_ms << "ms" << logs::end;
	sensorsThread_ = new SensorsThread(*this, period_ms);
	sensorsThread_->start("SensorsThread", 0);
}

void Sensors::stopSensorsThread()
{
	if (sensorsThread_ == nullptr) return;

	logger().debug() << "stopSensorsThread" << logs::end;
	sensorsThread_->stopThread();
	sensorsThread_->waitForEnd();
	delete sensorsThread_;
	sensorsThread_ = nullptr;
}

void SensorsThread::sensorOnTimer()
{
	// Reset detection event pour ce cycle
	sensors_.lastDetection_.clear();
	sensors_.lastDetection_.timestamp_us = 0;  // TODO: chronometer si besoin

	// 1. LECTURE — sync beacon I2C
	int err = sensors_.sync("beacon_sync");
	if (err < 0)
	{
		logger().error() << ">> SYNC BAD DATA! NO UPDATE" << logs::end;
		return;
	}
	if (err == 0)
	{
		// Pas de nouvelles donnees beacon, on skip le traitement
		return;
	}

	// t_sync_ms = timestamp capture par le driver JUSTE apres le readFlag,
	// avant la longue lecture getData() (~5-10ms). Plus precis qu'un timestamp
	// pris en debut de onTimer() qui inclurait la latence readFlag.
	uint32_t t_sync_ms = sensors_.sensorsdriver_->getLastSyncMs();

	// Copier le seq beacon (debug)
	sensors_.lastDetection_.beacon_seq = sensors_.sensorsdriver_->getBeaconSeq();

	// Copier le t_us du premier robot détecté et chercher la position robot au moment de la mesure
	ASensorsDriver::bot_positions vpos = sensors_.sensorsdriver_->getvPositionsAdv();
	uint16_t beacon_delay_us = 0;
	if (!vpos.empty())
	{
		beacon_delay_us = vpos[0].t_us;
		sensors_.lastDetection_.beacon_delay_us = beacon_delay_us;
	}

	// Position robot au moment estimé de la mesure beacon.
	// beacon_delay_us = delta depuis le DEBUT du cycle Teensy jusqu'a la mesure ToF.
	// t_sync_ms est capture cote driver juste apres readFlag (proche de la fin
	// du cycle Teensy, latence I2C deja ecartee), donc :
	//   T_debut_cycle = t_sync - duree_cycle_teensy
	//   T_mesure       = T_debut_cycle + beacon_delay_us
	//                  = t_sync - duree_cycle_teensy + beacon_delay_us
	// Duree cycle Teensy mesuree empiriquement entre 40 et 60ms.
	//
	// TODO precision restante :
	//  1) Recevoir la duree reelle du cycle Teensy dans un registre I2C (au lieu de la constante)
	//  2) Reduire la periode du SensorsThread pour reduire le jitter de polling
	//     (entre la fin de cycle Teensy et la lecture du flag I2C par OPOS6UL : 0-20ms aleatoire)
	//  3) Mieux : GPIO interrupt Teensy -> OPOS6UL a chaque fin de cycle, capture timestamp exact
	static const uint32_t TEENSY_CYCLE_MS = 60;
	uint32_t t_mesure_ms = t_sync_ms - TEENSY_CYCLE_MS + (beacon_delay_us / 1000);

	ROBOTPOSITION posAtMeasure = sensors_.robot()->sharedPosition()->getPositionAt(t_mesure_ms);
	sensors_.lastDetection_.x_robot_mm = posAtMeasure.x;
	sensors_.lastDetection_.y_robot_mm = posAtMeasure.y;
	sensors_.lastDetection_.theta_robot_rad = posAtMeasure.theta;

	logger().debug() << "SYNC t_sync=" << t_sync_ms << "ms t_mesure=" << t_mesure_ms
			<< "ms posAt=(" << posAtMeasure.x << "," << posAtMeasure.y
			<< "," << (posAtMeasure.theta * 180.0f / M_PI) << "deg)"
			<< " delay=" << beacon_delay_us << "us" << logs::end;

	// 2. FILTRAGE AVANT — classification + debounce
	if (sensors_.getAvailableFrontCenter())
	{
		int frontLevel = sensors_.front(true);

		// Debounce : compteur de stabilité
		if (frontLevel <= 3)
			nb_sensor_front_a_zero++;
		else
			nb_sensor_front_a_zero = 0;

		if (frontLevel == 4)
			nb_ensurefront4++;
		else
			nb_ensurefront4 = 0;

		// Publication du level debounced dans le DetectionEvent
		// Level 4 confirmé seulement après 2 cycles consécutifs
		if (nb_ensurefront4 >= 2)
			sensors_.lastDetection_.frontLevel = 4;
		else if (frontLevel >= 3)
			sensors_.lastDetection_.frontLevel = 3;
		else
			sensors_.lastDetection_.frontLevel = frontLevel;

		lastdetect_front_level_ = frontLevel;
		sensors_.robot()->displayObstacle(frontLevel);
	}

	// 3. FILTRAGE ARRIERE — classification + debounce
	if (sensors_.getAvailableBackCenter())
	{
		int backLevel = sensors_.back(true);

		if (backLevel >= -3)
			nb_sensor_back_a_zero++;
		else
			nb_sensor_back_a_zero = 0;

		if (backLevel == -4)
			nb_ensureback4++;
		else
			nb_ensureback4 = 0;

		// Publication du level debounced
		if (nb_ensureback4 >= 2)
			sensors_.lastDetection_.backLevel = -4;
		else if (backLevel <= -3)
			sensors_.lastDetection_.backLevel = -3;
		else
			sensors_.lastDetection_.backLevel = backLevel;

		lastdetect_back_level_ = backLevel;
		sensors_.robot()->displayObstacle(backLevel);
	}

	// Le DetectionEvent est maintenant prêt.
	// waitEndOfTrajWithDetection() le consultera à 1ms.
	// Aucun appel à l'Asserv ici.
}

