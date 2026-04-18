/*!
 * \file
 * \brief Gestion des capteurs de detection et de distance.
 *
 * Fournit la detection d'obstacles (avant/arriere), le filtrage
 * des mesures et un thread dedie de surveillance periodique.
 */

#ifndef SENSORS_HPP_
#define SENSORS_HPP_

#include <atomic>
#include <sstream>
#include <string>

#include "log/LoggerFactory.hpp"
#include "interface/ASensorsDriver.hpp"
#include "../Robot.hpp"
#include "utils/Chronometer.hpp"
#include "AActionsElement.hpp"
#include "thread/Thread.hpp"
#include "geometry/ObstacleZone.hpp"
#include "geometry/DetectionEvent.hpp"

class ASensorsDriver;
class Robot;
class SensorsThread;

/*!
 * \brief Gestion des capteurs de detection et de distance.
 */
class Sensors: public AActionsElement {
	friend class SensorsThread;  ///< SensorsThread accede a lastDetection_ pour le timestamp
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
	void writeLedLuminosity(uint8_t lum);

	// --- Wrappers Settings balise pour MenuBeaconLCDTouch ---
	// Voir robot/md/O_STATE_NEW_INIT.md section 6.
	// En init, ces methodes travaillent sur le cache (zero I2C).
	// L'I2C est fait dans syncFull() appele depuis la boucle O_State_NewInit.
	bool readMatchSettings(MatchSettingsData& out) { return sensorsdriver_->readMatchSettings(out); }
	bool writeMatchColor(uint8_t c)   { return sensorsdriver_->writeMatchColor(c); }
	bool writeStrategy(uint8_t s)     { return sensorsdriver_->writeStrategy(s); }
	bool writeAdvDiameter(uint8_t d)  { return sensorsdriver_->writeAdvDiameter(d); }
	bool writeMatchState(uint8_t s)   { return sensorsdriver_->writeMatchState(s); }
	bool writeMatchPoints(uint8_t p)  { return sensorsdriver_->writeMatchPoints(p); }
	bool writeNumOfBots(int8_t n)     { return sensorsdriver_->writeNumOfBots(n); }
	bool writeActionReq(uint8_t v)    { return sensorsdriver_->writeActionReq(v); }

	int syncFull() { return sensorsdriver_->syncFull(); }

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

	void startSensorsThread(int period_ms);

	void stopSensorsThread();

private:
	SensorsThread* sensorsThread_ = nullptr;
};

/*!
 * \brief Thread dedie a la detection d'obstacles.
 *
 * Sorti du scheduler unifie ActionTimerScheduler car le sync I2C beacon
 * prend 5-10ms (budget scheduler < 1ms). Thread propre avec PeriodicTimer
 * pour ne pas bloquer les autres timers (ServoObjects, LedBar...).
 */
class SensorsThread: public utils::Thread
{
private:

	static const logs::Logger& logger()
	{
		static const logs::Logger &instance = logs::LoggerFactory::logger("SensorsThread");
		return instance;
	}

	Sensors &sensors_;
	int period_us_;
	std::atomic<bool> stopRequested_;

	int lastdetect_front_level_;
	int lastdetect_back_level_;

	int nb_sensor_front_a_zero;
	int nb_sensor_back_a_zero;

	int nb_ensurefront4;
	int nb_ensureback4;

protected:

	virtual void execute() override;

public:

	SensorsThread(Sensors &sensors, int period_ms);

	virtual inline ~SensorsThread()
	{
	}

	void stopThread();

	void sensorOnTimer();
};

#endif
