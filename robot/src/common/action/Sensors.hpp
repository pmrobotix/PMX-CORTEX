/*!
 * \file
 * \brief Gestion des capteurs de detection et de distance.
 *
 * Fournit la detection d'obstacles (avant/arriere), le filtrage
 * des mesures et un timer periodique de surveillance.
 */

#ifndef SENSORS_HPP_
#define SENSORS_HPP_

#include <sstream>
#include <string>

#include "log/LoggerFactory.hpp"
#include "interface/ASensorsDriver.hpp"
#include "../Robot.hpp"
#include "utils/Chronometer.hpp"
#include "AActionsElement.hpp"
#include "timer/ITimerScheduledListener.hpp"
#include "geometry/ObstacleZone.hpp"
#include "geometry/DetectionEvent.hpp"

class ASensorsDriver;
class Robot;

/*!
 * \brief Gestion des capteurs de detection et de distance.
 */
class Sensors: public AActionsElement {
	friend class SensorsTimer;  ///< SensorsTimer accede a lastDetection_ pour le timestamp
private:

	static inline const logs::Logger& logger()
	{
		static const logs::Logger &instance = logs::LoggerFactory::logger("Sensors");
		return instance;
	}

	Robot *robot_;
	ASensorsDriver *sensorsdriver_;
	ObstacleZone obstacleZone_;
	DetectionEvent lastDetection_;   ///< Cycle courant (ecrase toutes les 62ms)
	DetectionEvent stopDetection_;   ///< Fige au moment d'un STOP (pour Navigator/IA)

	ASensorsDriver::bot_positions opponents_last_positions;
public:

	//distance de ce qu'il y a devant le robot (legacy, utiliser lastDetection())
	float x_adv_mm;
	float y_adv_mm;

	/*!
	 * \brief Constructor.
	 */
	Sensors(Actions &actions, Robot *robot);

	/*!
	 * \brief Destructor.
	 */
	~Sensors();

	Robot* robot()
	{
		return robot_;
	}

	bool is_connected(); //is connected and alive

	void display(int n);

	inline bool getAvailableFrontCenter() { return obstacleZone_.getAvailableFrontCenter(); }
	inline bool getAvailableBackCenter() { return obstacleZone_.getAvailableBackCenter(); }

	ObstacleZone& obstacleZone() { return obstacleZone_; }

	/*!
	 * \brief Retourne le dernier evenement de detection (cycle courant, 62ms).
	 * Utilise par waitEndOfTrajWithDetection() pour decision temps reel.
	 */
	const DetectionEvent& lastDetection() const { return lastDetection_; }

	/*!
	 * \brief Retourne la detection figee au moment du dernier STOP.
	 * Utilise par Navigator/DecisionMaker pour savoir pourquoi et ou.
	 */
	const DetectionEvent& stopDetection() const { return stopDetection_; }

	/*!
	 * \brief Fige le DetectionEvent courant (appele par Asserv au moment du STOP).
	 */
	void setStopDetection(const DetectionEvent& det) { stopDetection_ = det; }

	float MultipleRightSide(int nb);
	float MultipleLeftSide(int nb);

	//capteur de distance
	int rightSide();
	int leftSide();
	float multipleRightSide(int nb);
	float multipleLeftSide(int nb);

	//acces directement aux capteurs
	int sync(std::string sensorname);

	//recupere la liste des positions adverses par la balise
	ASensorsDriver::bot_positions setPositionsAdvByBeacon();

	//lecture thread-safe des positions adverses (pour affichage/test, sans ecrire dans opponents_last_positions)
	ASensorsDriver::bot_positions getPositionsAdv() { return sensorsdriver_->getvPositionsAdv(); }

	void clearPositionsAdv();

	//accès detection
	int front(bool display = false); //retourne le niveau de detection
	int back(bool display = false);

	//detection adversaire
	int right(bool display = false) { return obstacleZone_.right(); }
	int left(bool display = false) { return obstacleZone_.left(); }

	// --- Delegation vers ObstacleZone (config, seuils, filtres) ---

	void addConfigFront(bool l, bool c, bool r) { obstacleZone_.addConfigFront(l, c, r); }
	void addConfigBack(bool l, bool c, bool r) { obstacleZone_.addConfigBack(l, c, r); }

	void remove_outside_table(bool enable) { obstacleZone_.setRemoveOutsideTable(enable); }

	int filtre_levelInFront(int threshold_LR_mm, int threshold_Front_mm, int threshold_veryclosed_front_mm,
			float dist_adv_mm, float x_adv_mm, float y_adv_mm, float theta_adv_deg)
	{
		return obstacleZone_.filtre_levelInFront(threshold_LR_mm, threshold_Front_mm,
				threshold_veryclosed_front_mm, dist_adv_mm, x_adv_mm, y_adv_mm, theta_adv_deg);
	}

	int filtre_levelInBack(int threshold_LR_mm, int threshold_Back_mm, int threshold_veryclosed_back_mm,
			float dist_adv_mm, float x_adv_mm, float y_adv_mm, float theta_adv_deg)
	{
		return obstacleZone_.filtre_levelInBack(threshold_LR_mm, threshold_Back_mm,
				threshold_veryclosed_back_mm, dist_adv_mm, x_adv_mm, y_adv_mm, theta_adv_deg);
	}

	void addThresholdFront(int l, int c, int r) { obstacleZone_.addThresholdFront(l, c, r); }
	void addThresholdBack(int l, int c, int r) { obstacleZone_.addThresholdBack(l, c, r); }
	void addThresholdFrontVeryClosed(int l, int c, int r) { obstacleZone_.addThresholdFrontVeryClosed(l, c, r); }
	void addThresholdBackVeryClosed(int l, int c, int r) { obstacleZone_.addThresholdBackVeryClosed(l, c, r); }

	void setIgnoreFrontNearObstacle(bool l, bool c, bool r) { obstacleZone_.setIgnoreFrontNearObstacle(l, c, r); }
	void setIgnoreBackNearObstacle(bool l, bool c, bool r) { obstacleZone_.setIgnoreBackNearObstacle(l, c, r); }
	void setIgnoreAllFrontNearObstacle(bool ignore) { obstacleZone_.setIgnoreAllFrontNearObstacle(ignore); }
	void setIgnoreAllBackNearObstacle(bool ignore) { obstacleZone_.setIgnoreAllBackNearObstacle(ignore); }

	//Ajoute le timer des sensors de detection
	void addTimerSensors(int timespan_ms);

	//supprime le timer des sensors
	void stopTimerSensors();
};

/*!
 * \brief Le timer associe aux sensors de detection.
 *        Tick par le scheduler unique ActionTimerScheduler.
 */
class SensorsTimer: public ITimerScheduledListener
{
private:

	static const logs::Logger& logger()
	{
		static const logs::Logger &instance = logs::LoggerFactory::logger("SensorsTimer");
		return instance;
	}

	Sensors &sensors_;

	int lastdetect_front_level_;
	int lastdetect_back_level_;

	int nb_sensor_front_a_zero;
	int nb_sensor_back_a_zero;

	int nb_ensurefront4;
	int nb_ensureback4;

public:

	SensorsTimer(Sensors &sensors, int timeSpan_ms, std::string name);

	virtual inline ~SensorsTimer()
	{
	}

	virtual void onTimer(utils::Chronometer chrono);

	virtual void onTimerEnd(utils::Chronometer chrono);

	virtual std::string info();

};

#endif
