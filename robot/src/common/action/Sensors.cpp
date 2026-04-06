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
SensorsTimer::SensorsTimer(Sensors &sensors, int timeSpan_ms, std::string name) :
		sensors_(sensors)
{
	name_ = name;

//    lastdetect_front_nb_ = 0;
//    lastdetect_back_nb_ = 0;
	lastdetect_front_level_ = 0;
	lastdetect_back_level_ = 0;

	nb_ensurefront4 = 0;
	nb_ensureback4 = 0;

	lastfrontl2_temp_ = false;
	lastbackl2_temp_ = false;

	nb_sensor_front_a_zero = 0;
	nb_sensor_back_a_zero = 0;
	nb_sensor_level2 = 0;
	nb_sensor_b_level2 = 0;


	//initialise le timer avec le nom et la periode.
	this->init(name_, timeSpan_ms * 1000);
}

// Methodes de config, seuils, ignore, filtres : delegues inline vers obstacleZone_ (voir Sensors.hpp)

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
// filtre_levelInFront : delegue inline vers obstacleZone_ (voir Sensors.hpp)

// filtre_levelInBack : delegue inline vers obstacleZone_ (voir Sensors.hpp)

// right() et left() : delegues inline vers obstacleZone_ (voir Sensors.hpp)

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

			//Attention la position du robot doit etre setter avant pour le calcul
			ROBOTPOSITION pos_robot_instantane = this->robot()->sharedPosition()->convertPositionBeaconToRepereTable(
					botpos.d, botpos.x, botpos.y, botpos.theta_deg, &x_pos_adv_table, &y_pos_adv_table);

			//filtre sur la table avec transformation de repere
			inside_table = this->robot()->tableGeometry()->isPointInsideTable((int) x_pos_adv_table,
					(int) y_pos_adv_table);
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

	} else
	{
		logger().error() << "ERROR NO getAvailableFrontCenter !!!!" << logs::end;

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

	return level; // TODO Ajouter x_pos_adv_table et ypos de l'adversaire detecter, pourra servir de log dans l'asserv au lieu d'ici...
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

			//Attention la position du grobot doit etre setter avant pour le calcul
			ROBOTPOSITION pos_robot_instantane = this->robot()->sharedPosition()->convertPositionBeaconToRepereTable(
					botpos.d, botpos.x, botpos.y, botpos.theta_deg, &x_pos_adv_table, &y_pos_adv_table);

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

	} else
	{
		logger().error() << "ERROR NO getAvailableBackCenter !!!!" << logs::end;
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
	return level; //TODO retourner la position du robot qui a causé ce level  x_pos_adv_table, y_pos_adv_table

}

void Sensors::addTimerSensors(int timeSpan_ms)
{
//On supprime s'il existe déjà

//    if (this->actions().findPTimer("Sensors"))
//    {
//        logger().debug() << "PT Sensors already exists! stop then restart it!" << logs::end;
//        this->actions().stopPTimer("Sensors");
//    }

	logger().debug() << "startSensors" << logs::end;
	this->actions().addTimer(new SensorsTimer(*this, timeSpan_ms, "Sensors"));
}

void Sensors::stopTimerSensors()
{
	logger().debug() << "stopSensors" << logs::end;
	this->actions().stopPTimer("Sensors");
//this->actions().stopTimer("Sensors");

}

void SensorsTimer::onTimer(utils::Chronometer chrono)
{
//    logger().error() << ">> SensorsTimer::onTimer sensors_.getAvailableFrontCenter()="
//            << sensors_.getAvailableFrontCenter() << logs::end;

	//get all data sync adn save the position of the robot and precedent position
	int err = sensors_.sync("beacon_sync");
	if (err < 0)
	{
		logger().error() << ">> SYNC BAD DATA! NO UPDATE" << logs::end;
		return;
	}
	if (sensors_.getAvailableFrontCenter())
	{
		int frontLevel = sensors_.front(true);
		//printf("SensorsTimer::onTimer frontLevel=%d\n", frontLevel);

		//si 1,2,3 on incremente, sinon si 0 on resette le compteur
		if (frontLevel <= 3)
		{
			nb_sensor_front_a_zero++;
		} else
		{
			nb_sensor_front_a_zero = 0;
		}

		//si 4, on attend et on compte le nombre pour etre sur
		if (frontLevel == 4)
		{
			nb_ensurefront4++;
		} else
		{
			nb_ensurefront4 = 0;
		}

		//si 0,1,2,3,4 puis 4 ; arret du robot warn l'asserv en mode 4
		if (lastdetect_front_level_ <= 4 && nb_ensurefront4 >= 2)
		{            //=frontLevel ==4
//            logger().error() << ">>  frontLevel=" << frontLevel
//                    <<  "lastdetect_front_level_=" << lastdetect_front_level_
//
//                    << " nb_ensurefront4=" << nb_ensurefront4
//                    << logs::end;
			sensors_.robot()->passerv()->warnFrontDetectionOnTraj(frontLevel, sensors_.x_adv_mm, sensors_.y_adv_mm);
		}

		//si 4 puis 0,1,2,3 ; resetEmergency (pas besoin pour 3 puisque qu'on ne fait pas d'emergencystop pour 3)
		if (lastdetect_front_level_ >= 4 && nb_sensor_front_a_zero >= 4)
		{
			sensors_.robot()->passerv()->resetEmergencyOnTraj("SensorsTimer front=0");
			sensors_.robot()->resetDisplayObstacle();
		}

		//si 0,1,2,3,4 puis 3 ; low speed
		if (frontLevel >= 3)
		{
			// TODO Ca pourrait etre mis dans la fonction Warn
			//sensors_.robot()->passerv()->setLowSpeedForward(true, sensors_.robot()->passerv()->getLowSpeedvalue());
			//sensors_.robot()->passerv()->setMaxSpeed(true, sensors_.robot()->passerv()->getMaxSpeedDistValue());
			sensors_.robot()->passerv()->warnFrontDetectionOnTraj(frontLevel, sensors_.x_adv_mm, sensors_.y_adv_mm);
		}

//        if (lastdetect_front_level_ >= 3 && nb_sensor_front_a_zero >= 4) {
//            //stop du robot et attente en remontant à l'ia
//        }

		//si 3 puis 0,1,2 ; vitesse normale => si 3 ou 4 puis 0,1,2 ; vitesse normale
		if (lastdetect_front_level_ >= 3 && frontLevel <= 2)
		{
			//sensors_.robot()->passerv()->setLowSpeedForward(false);
			//sensors_.robot()->passerv()->setMaxSpeed(false);
			sensors_.robot()->passerv()->warnFrontDetectionOnTraj(frontLevel, sensors_.x_adv_mm, sensors_.y_adv_mm);
			//sensors_.robot()->passerv()->resetEmergencyOnTraj("SensorsTimer front=0");
			sensors_.robot()->resetDisplayObstacle();
		}

		lastdetect_front_level_ = frontLevel;
		sensors_.robot()->displayObstacle(frontLevel); //TODO a separer en displayObstacleFront et displayObstacleBack

	}

//ARRIERE/////////////////////////////////////////////////////////////////

	if (sensors_.getAvailableBackCenter())
	{
		int backLevel = sensors_.back(true);
		//printf("SensorsTimer::onTimer backLevel=%d\n", backLevel);

		if (backLevel >= -3)
		{
			nb_sensor_back_a_zero++;
		} else
		{
			nb_sensor_back_a_zero = 0;
		}

		if (backLevel == -4)
		{
			nb_ensureback4++;
		} else
		{
			nb_ensureback4 = 0;
		}


		//si 0,1,2,3,4 puis 4 ; arret du robot warn
		if (lastdetect_back_level_ >= -4 && nb_ensureback4 >= 2)
		{
			sensors_.robot()->passerv()->warnBackDetectionOnTraj(backLevel, sensors_.x_adv_mm, sensors_.y_adv_mm);
		}

		//si 4 puis 0,1,2,3 ; resetEmergency
		if (lastdetect_back_level_ <= -4 && nb_sensor_back_a_zero >= 4)
		{
			sensors_.robot()->passerv()->resetEmergencyOnTraj("SensorsTimer back=si 4 puis 0,1,2,3 ; resetEmergency");
			sensors_.robot()->resetDisplayObstacle();
		}

		//si 0,1,2,3,4 puis 3 ; low speed
		if (backLevel <= -3)
		{
			//sensors_.robot()->passerv()->setLowSpeedForward(true, sensors_.robot()->passerv()->getLowSpeedvalue());
			//sensors_.robot()->passerv()->setMaxSpeed(true, sensors_.robot()->passerv()->getMaxSpeedDistValue());
			sensors_.robot()->passerv()->warnBackDetectionOnTraj(backLevel, sensors_.x_adv_mm, sensors_.y_adv_mm);
		}

		//si 3 puis 0,1,2 ; vitesse normale => si 3 ou 4 puis 0,1,2 ; vitesse normale
		if (lastdetect_back_level_ <= -3 && backLevel >= -2)
		{
			sensors_.robot()->passerv()->warnBackDetectionOnTraj(backLevel, sensors_.x_adv_mm, sensors_.y_adv_mm);
			sensors_.robot()->passerv()->resetEmergencyOnTraj("SensorsTimer back=si 3 puis 0,1,2 ; vitesse normale => si 3 ou 4 puis 0,1,2 ; vitesse normale");
			sensors_.robot()->resetDisplayObstacle();
		}

		lastdetect_back_level_ = backLevel;
		sensors_.robot()->displayObstacle(backLevel); //TODO a separer en displayObstacleFront et displayObstacleBack
	}
}

void SensorsTimer::onTimerEnd(utils::Chronometer chrono)
{

}

std::string SensorsTimer::info()
{
	std::ostringstream oss;
	oss << "SensorsTimer [" << name() << "] for " << sensors_.robot()->getID();
	return oss.str();
}

