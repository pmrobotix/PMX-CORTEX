#ifndef COMMON_ASSERV_HPP_
#define COMMON_ASSERV_HPP_

#include <cmath>
#include <string>
#include <tuple>
#include <vector>

#include "log/LoggerFactory.hpp"
#include "interface/AAsservDriver.hpp"
#include "interface/ARobotPositionShared.hpp"

class AAsservDriver;
class AsservEsialR;
class Robot;

/*!
 * Asservissement of the robot.It contains default elements.
 */
class Asserv
{
private:

    /*!
     * \brief Return \ref Logger linked to \ref Asserv.
     */
    static inline const logs::Logger& logger()
    {
        static const logs::Logger &instance = logs::LoggerFactory::logger("Asserv");
        return instance;
    }

    bool emergencyStop_;


protected:

    /*!
     * \brief type d'asservissement utilisé
     * \0=Asserdriver 1=ESIAL internal 2=INSA internal (deprecated)
     */
    //int useAsservType_;
    enum AsservType
    {
        ASSERV_EXT, ASSERV_INT_ESIALR
    };

    AsservType useAsservType_;

    std::string botId_;

    AAsservDriver *asservdriver_;

    /*!
     * \brief asservissement interne ESIAL.
     * NULL si non defini
     */
    AsservEsialR *pAsservEsialR_;


    bool temp_ignoreBackDetection_;
    bool temp_ignoreFrontDetection_;
    bool temp_forceRotation_;

    //0=>LEFT with coordinate x, y, angle
    //1=>RIGHT with coordinate 3000-x, y , -angle
    bool matchColorPosition_;

    Robot *probot_; //reference du parent

    ROBOTPOSITION adv_pos_centre_;

    //automatic conversion
    int x_ground_table_;

    int lowSpeedvalue_;
    int maxSpeedDistValue_;


public:

    /*!
     * \brief Vérifie si le driver d'asservissement est connecté (liaison série ou simulateur).
     * \return true si la connexion est active.
     */
    bool is_connected() { return asservdriver_->is_connected(); }

    /*!
     * \brief Constructeur.
     * \param botId Identifiant du robot (ex: "OPOS6UL"), utilisé pour instancier le bon driver.
     * \param robot Pointeur vers l'objet Robot parent (accès aux positions partagées).
     */
    Asserv(std::string botId, Robot *robot);

    /*!
     * \brief Destructeur. Libère le driver et l'asserv interne.
     */
    virtual ~Asserv();

    /*!
     * \brief Retourne le nom lisible d'un état de trajectoire (pour les logs).
     * \param ts État de trajectoire (enum TRAJ_STATE).
     * \return Chaîne décrivant l'état (ex: "TRAJ_FINISHED", "TRAJ_NEAR_OBSTACLE").
     */
    std::string getTraj(TRAJ_STATE ts)
    {
    	switch (ts) {
    	case TRAJ_IDLE:
    		return "TRAJ_IDLE";
    	case TRAJ_FINISHED:
    		return "TRAJ_FINISHED";
    	case TRAJ_INTERRUPTED:
    		return "TRAJ_INTERRUPTED";
    	case TRAJ_IMPOSSIBLE:
    		return "TRAJ_IMPOSSIBLE";
    	case TRAJ_NEAR_OBSTACLE:
    		return "TRAJ_NEAR_OBSTACLE";
    	case TRAJ_COLLISION:
    		return "TRAJ_COLLISION";
    	case TRAJ_REAR_OBSTACLE:
    		return "TRAJ_REAR_OBSTACLE";
    	case TRAJ_REAR_COLLISION:
    		return "TRAJ_REAR_COLLISION";
    	case TRAJ_ERROR:
    		return "TRAJ_ERROR";
    	default:
    		return "DEFAULT_IMPOSSIBLE!";
    	}
    }

    // ========== FONCTIONS DE BASE — ENCODEURS ET MOTEURS ==========

    /*!
     * \brief Remet à zéro les compteurs des encodeurs gauche et droit.
     */
    void resetEncoders();

    /*!
     * \brief Lit les compteurs accumulés des encodeurs (ticks absolus).
     * \param[out] countR Compteur encodeur droit.
     * \param[out] countL Compteur encodeur gauche.
     */
    void getEncodersCounts(int *countR, int *countL);

    /*!
     * \brief Lit le delta de ticks des encodeurs depuis le dernier appel.
     *        Utile pour l'odométrie incrémentale.
     * \param[out] deltaCountR Delta encodeur droit.
     * \param[out] deltaCountL Delta encodeur gauche.
     */
    void getDeltaEncodersCounts(int *deltaCountR, int *deltaCountL);

    /*!
     * \brief Commande le moteur gauche.
     * \param power Puissance (positif = avant, négatif = arrière).
     * \param timems Durée en ms. <0 : duty cycle permanent, 0 : régulation vitesse, >0 : régulation avec durée.
     */
    void runMotorLeft(int power, int timems);

    /*!
     * \brief Commande le moteur droit.
     * \param power Puissance (positif = avant, négatif = arrière).
     * \param timems Durée en ms. <0 : duty cycle permanent, 0 : régulation vitesse, >0 : régulation avec durée.
     */
    void runMotorRight(int power, int timems);

    /*!
     * \brief Arrête les deux moteurs immédiatement.
     */
    void stopMotors();

    // ========== CONFIGURATION VITESSE ==========

    /*!
     * \brief Retourne la valeur de vitesse lente (en %).
     */
    int getLowSpeedvalue();

    /*!
     * \brief Définit la valeur de vitesse lente (en %).
     * \param value Pourcentage de vitesse lente (ex: 30 = 30%).
     */
    void setLowSpeedvalue(int value);

    /*!
     * \brief Retourne la valeur max de vitesse en distance (en %).
     */
    int getMaxSpeedDistValue();

    /*!
     * \brief Définit la valeur max de vitesse en distance (en %).
     */
    void setMaxSpeedDistValue(int value);

    // ========== FILTRAGE POSITION ET DÉTECTION ==========

    /*!
     * \brief Vérifie si une position (x,y) est à l'intérieur de la table.
     *        Fonction pure virtuelle — doit être implémentée par chaque robot.
     * \param x_botpos Position X en mm.
     * \param y_botpos Position Y en mm.
     * \return true si la position est dans les limites de la table.
     */
    virtual bool filtre_IsInsideTableXY(int x_botpos, int y_botpos) = 0;

    /*!
     * \brief Callback appelé en fin de trajectoire pour gérer les actions post-mouvement.
     *        Peut être surchargé par les robots spécifiques.
     */
    virtual void endWhatTodo();

    // ========== GESTION DE L'ASSERVISSEMENT ==========

    /*!
     * \brief Démarre le timer de mouvement et l'odométrie.
     *        Active le thread d'asservissement et optionnellement le maintien en position.
     * \param assistedHandlingEnabled true pour activer le maintien en position (assistedHandling).
     */
    virtual void startMotionTimerAndOdo(bool assistedHandlingEnabled);

    /*!
     * \brief Active/désactive la vitesse lente en marche avant.
     * \param enable true pour activer.
     * \param percent Pourcentage de vitesse (0 = valeur par défaut).
     */
    void setLowSpeedForward(bool enable, int percent = 0);

    /*!
     * \brief Active/désactive la vitesse lente en marche arrière.
     * \param enable true pour activer.
     * \param percent Pourcentage de vitesse (0 = valeur par défaut).
     */
    virtual void setLowSpeedBackward(bool enable, int percent = 0);

    /*!
     * \brief Définit la vitesse maximale en distance et en angle.
     * \param enable true pour activer la limitation, false pour revenir à la vitesse normale.
     * \param speed_dist_percent Vitesse max en distance (0-100%).
     * \param speed_angle_percent Vitesse max en angle (0-100%).
     */
    void setMaxSpeed(bool enable, int speed_dist_percent=0, int speed_angle_percent=0);

    /*!
     * \brief Définit la position initiale du robot et la couleur de match.
     *        Applique automatiquement la symétrie si matchColor != 0 (inversion X et angle).
     * \param x_mm Position X en mm (dans le repère couleur primaire).
     * \param y_mm Position Y en mm.
     * \param theta_degrees Angle en degrés.
     * \param matchColor false = couleur primaire (bas-gauche), true = couleur secondaire (symétrie X).
     */
    virtual void setPositionAndColor(float x_mm, float y_mm, float theta_degrees, bool matchColor);

    /*!
     * \brief Définit directement la position réelle du robot (sans conversion couleur).
     * \param x_mm Position X brute en mm.
     * \param y_mm Position Y brute en mm.
     * \param thetaInRad Angle en radians.
     */
    virtual void setPositionReal(float x_mm, float y_mm, float thetaInRad);

    /*!
     * \brief Vérifie si un obstacle détecté est à l'intérieur de la table.
     *        Doit être surchargé par chaque robot pour tenir compte de sa géométrie.
     * \param dist_detect_mm Distance de détection en mm.
     * \param lateral_pos_sensor_mm Position latérale du capteur en mm.
     * \param desc Description pour le log.
     * \return true si l'obstacle est dans la table.
     */
    virtual bool filtre_IsInsideTable(int dist_detect_mm, int lateral_pos_sensor_mm, std::string desc = "");

    /*!
     * \brief Callback de détection d'obstacle devant le robot pendant une trajectoire.
     *        Gère les niveaux d'alerte : 2=vitesse normale, 3=ralentir, 4=arrêt d'urgence.
     * \param frontlevel Niveau de détection (2, 3 ou 4).
     * \param x_adv__mm Position X de l'adversaire dans le repère robot.
     * \param y_adv_mm Position Y de l'adversaire dans le repère robot.
     */
    virtual void warnFrontDetectionOnTraj(int frontlevel, float x_adv__mm, float y_adv_mm);

    /*!
     * \brief Callback de détection d'obstacle derrière le robot pendant une trajectoire.
     *        Gère les niveaux d'alerte : -2=vitesse normale, -3=ralentir, -4=arrêt d'urgence.
     * \param backlevel Niveau de détection (-2, -3 ou -4).
     * \param x_adv_mm Position X de l'adversaire dans le repère robot.
     * \param y_adv_mm Position Y de l'adversaire dans le repère robot.
     */
    virtual void warnBackDetectionOnTraj(int backlevel, float x_adv_mm, float y_adv_mm);

    /*!
     * \brief Met à jour la position de l'adversaire. À surcharger par le robot.
     */
    virtual void update_adv();

    /*!
     * \brief Réinitialise l'arrêt d'urgence après une interruption de trajectoire.
     *        Permet au robot de reprendre ses mouvements après un obstacle.
     * \param message Description pour le log (contexte de l'appel).
     */
    void resetEmergencyOnTraj(std::string message = "default");

    /*!
     * \brief Déclenche un arrêt d'urgence : interrompt la trajectoire en cours.
     */
    void setEmergencyStop();

    // ========== MODES D'ARRÊT ==========

    /*!
     * \brief Arrête le timer de mouvement et l'odométrie.
     *        Stoppe le thread d'asservissement.
     */
    virtual void stopMotionTimerAndOdo();

    /*!
     * \brief Désactive le PID et active la QuadRamp (rampe d'accélération).
     */
    void disablePID();

    /*!
     * \brief Libère les moteurs (roue libre, plus d'asservissement).
     *        Le robot peut être poussé à la main.
     */
    void freeMotion();

    /*!
     * \brief Active le maintien en position (les moteurs résistent aux perturbations).
     */
    void assistedHandling();

    // ========== LECTURE DE POSITION ==========

    /*!
     * \brief Retourne la dernière position connue de l'adversaire.
     * \return Position (x, y, theta) de l'adversaire en mm/rad.
     */
    ROBOTPOSITION pos_getAdvPosition();

    /*!
     * \brief Retourne la position courante du robot (depuis l'odométrie).
     *        Met aussi à jour la position partagée (sharedPosition) pour les capteurs.
     * \return Position (x, y, theta) du robot en mm/rad.
     */
    ROBOTPOSITION pos_getPosition();

    /*!
     * \brief Retourne la position X courante du robot en mm.
     */
    float pos_getX_mm();

    /*!
     * \brief Retourne la position Y courante du robot en mm.
     */
    float pos_getY_mm();

    /*!
     * \brief Retourne l'angle courant du robot en radians.
     */
    float pos_getTheta();

    /*!
     * \brief Retourne l'angle courant du robot en degrés.
     */
    float pos_getThetaInDegree();

    // ========== DÉPLACEMENTS GOTO (position absolue sur la table) ==========

    /*!
     * \brief Déplacement vers (x,y) en mode chaîné (n'attend pas l'arrêt complet avant le prochain ordre).
     *        Applique la symétrie couleur sur X. Utile pour enchaîner des points de passage.
     * \param xMM Position X cible en mm (repère couleur primaire).
     * \param yMM Position Y cible en mm.
     * \return État de la trajectoire.
     */
    TRAJ_STATE goToChain(float xMM, float yMM);

    /*!
     * \brief Déplacement vers (x,y) avec arrêt complet à l'arrivée.
     *        Le robot tourne d'abord face au point, puis avance en ligne droite.
     * \param xMM Position X cible en mm (repère couleur primaire).
     * \param yMM Position Y cible en mm.
     * \return État de la trajectoire.
     */
    TRAJ_STATE goTo(float xMM, float yMM);

    /*!
     * \brief Déplacement en marche arrière vers (x,y).
     *        Le robot tourne dos au point, puis recule en ligne droite.
     * \param xMM Position X cible en mm (repère couleur primaire).
     * \param yMM Position Y cible en mm.
     * \return État de la trajectoire.
     */
    TRAJ_STATE goToReverse(float xMM, float yMM);

    /*!
     * \brief Déplacement en marche arrière chaîné vers (x,y).
     * \param xMM Position X cible en mm (repère couleur primaire).
     * \param yMM Position Y cible en mm.
     * \return État de la trajectoire.
     */
    TRAJ_STATE goToReverseChain(float xMM, float yMM);

    /*!
     * \brief Definit le multiplicateur de temps pour la simulation (SIMU uniquement).
     *        0.0 = instantane, 1.0 = temps reel. Ignore en ARM.
     *
     *        Note : a 0 (instantane), les points bleus odometriques (writePosition_BotPos)
     *        sont quasi absents du SVG car la boucle PID n'a pas le temps de tourner
     *        entre les mouvements. A 1.0 (temps reel), la trace complete point par point
     *        est visible sur le SVG.
     */
    void setSimuSpeedMultiplier(float multiplier);

    // ========== ENVOI SANS ATTENTE (pour mode CHAIN) ==========

    /*!
     * \brief Envoie un goto au driver sans attendre la fin (pas de waitEndOfTraj).
     *        Utilise la commande "g" (arret au point).
     */
    void goToSend(float xMM, float yMM);

    /*!
     * \brief Envoie un goToChain au driver sans attendre (commande "e", pas d'arret).
     */
    void goToChainSend(float xMM, float yMM);

    /*!
     * \brief Envoie un goToReverse au driver sans attendre.
     */
    void goToReverseSend(float xMM, float yMM);

    /*!
     * \brief Envoie un goToReverseChain au driver sans attendre.
     */
    void goToReverseChainSend(float xMM, float yMM);

    /*!
     * \brief Attend la fin de la trajectoire en cours (ou de la queue de commandes).
     * \return Etat final de la trajectoire.
     */
    TRAJ_STATE waitTraj();

    // ========== DÉPLACEMENTS RELATIFS (par rapport à la position courante) ==========

    /*!
     * \brief Avance (ou recule) en ligne droite d'une distance donnée.
     *        Pendant l'avance, la détection arrière est ignorée (et inversement).
     * \param dist_mm Distance en mm. Positif = avancer, négatif = reculer.
     * \return État de la trajectoire.
     */
    TRAJ_STATE line(float dist_mm);

    /*!
     * \brief Rotation relative d'un angle en degrés par rapport à l'orientation courante.
     *        Positif = sens trigonométrique (gauche), négatif = sens horaire (droite).
     * \param degreesRelative Angle relatif en degrés.
     * \param rotate_ignoring_opponent true pour ignorer la détection adverse pendant la rotation.
     * \return État de la trajectoire.
     */
    TRAJ_STATE rotateDeg(float degreesRelative, bool rotate_ignoring_opponent = true);

    /*!
     * \brief Rotation relative d'un angle en radians par rapport à l'orientation courante.
     * \param radRelative Angle relatif en radians.
     * \param rotate_ignoring_opponent true pour ignorer la détection adverse pendant la rotation.
     * \return État de la trajectoire.
     */
    TRAJ_STATE rotateRad(float radRelative, bool rotate_ignoring_opponent = true);

    /*!
     * \brief Rotation relative avec inversion automatique selon la couleur de match.
     *        Couleur primaire : angle tel quel. Couleur secondaire : angle inversé (-angle).
     * \param thetaInDegreeRelative Angle relatif en degrés (repère couleur primaire).
     * \param rotate_ignoring_opponent true pour ignorer la détection adverse.
     * \return État de la trajectoire.
     */
    TRAJ_STATE rotateByMatchColorDeg(float thetaInDegreeRelative, bool rotate_ignoring_opponent = true);

    /*!
     * \brief Rotation absolue vers un angle donné sur le terrain.
     *        Calcule la rotation relative nécessaire depuis l'angle courant,
     *        en appliquant la symétrie couleur de match.
     * \param thetaInDegreeAbsolute Angle cible en degrés (repère couleur primaire, 0° = axe X+).
     * \param rotate_ignore_opponent true pour ignorer la détection adverse.
     * \return État de la trajectoire.
     */
    TRAJ_STATE rotateAbsDeg(float thetaInDegreeAbsolute, bool rotate_ignore_opponent = true);

    /*!
     * \brief Tourne le robot face à un point (x,y) du terrain.
     * \param xMM Position X du point en mm (repère couleur primaire).
     * \param yMM Position Y du point en mm.
     * \param back_face true pour tourner le dos au point au lieu de lui faire face.
     * \return État de la trajectoire.
     */
    TRAJ_STATE faceTo(float xMM, float yMM, bool back_face = false);

    // ========== CALAGE (recalage contre un mur/bordure) ==========

    /*!
     * \brief Calage contre un mur avec timeout. Le robot avance lentement jusqu'au blocage.
     *        Désactive la régulation angulaire pendant le calage.
     * \param dist_mm Distance max en mm (positif=avant, négatif=arrière).
     * \param percent Pourcentage de vitesse lente.
     * \param timeout_ms Timeout en ms (non encore implémenté).
     * \return État de la trajectoire.
     */
    TRAJ_STATE calageNew(float dist_mm, int percent, float timeout_ms);

    /*!
     * \brief Calage simple : avance lentement et s'arrête au blocage mécanique.
     * \param d Distance en mm (positif=avant, négatif=arrière).
     * \param percent Pourcentage de vitesse lente.
     * \return État de la trajectoire.
     */
    TRAJ_STATE calage(int d, int percent);

    /*!
     * \brief Calage en 2 phases : phase 1 avec régulation, phase 2 en ligne directe.
     * \param d Distance en mm.
     * \param percent Pourcentage de vitesse lente.
     * \return État de la trajectoire.
     */
    TRAJ_STATE calage2(int d, int percent);

    // ========== PIVOT (rotation autour d'une roue) ==========

    /*!
     * \brief Pivote autour de la roue gauche (la roue gauche reste quasi immobile).
     * \param powerL Puissance moteur gauche.
     * \param powerR Puissance moteur droit.
     * \param timemsR Durée en ms.
     */
    /*!
     * \brief Rotation orbitale asservie autour d'une roue.
     *        Le robot tourne d'un angle donne autour de la roue pivot.
     * \param angleDeg Angle de rotation en degres.
     * \param forward true = marche avant, false = marche arriere.
     * \param turnRight true = pivot roue droite, false = pivot roue gauche.
     * \return Etat de la trajectoire.
     */
    TRAJ_STATE orbitalTurnDeg(float angleDeg, bool forward, bool turnRight);

    [[deprecated("Utiliser orbitalTurnDeg() a la place (asservi)")]]
    void pivotLeft(int powerL, int powerR, int timemsR);

    [[deprecated("Utiliser orbitalTurnDeg() a la place (asservi)")]]
    void pivotRight(int powerL, int powerR, int timemsL);

    // ========== DÉPLACEMENTS COMPOSÉS (rotation explicite + ligne droite) ==========
    // Note : ces méthodes sont utilisées en interne par Navigator.
    // Le code client doit utiliser Navigator::moveForwardTo() / moveBackwardTo() etc.

    [[deprecated("Utiliser Navigator::moveForwardTo()")]]
    TRAJ_STATE moveForwardTo(float xMM, float yMM, bool rotate_ignored = false, float adjustment = 0);

    [[deprecated("Utiliser Navigator::moveBackwardTo()")]]
    TRAJ_STATE moveBackwardTo(float xMM, float yMM, bool rotate_ignored = false);

    [[deprecated("Utiliser Navigator::moveForwardToAndRotateAbsDeg()")]]
    TRAJ_STATE moveForwardAndRotateTo(float xMM, float yMM, float thetaInDegree, bool rotate_ignore_opponent = true);

    [[deprecated("Utiliser Navigator::moveBackwardTo() + Navigator::rotateAbsDeg()")]]
    TRAJ_STATE moveBackwardAndRotateTo(float xMM, float yMM, float thetaInDegree);

    // ========== RECALAGE PAR TRIANGULATION (intersection de 2 cercles) ==========

    /*!
     * \brief Calcule l'intersection de 2 cercles pour déterminer la position réelle.
     *        Ref: http://nains-games.com/2014/12/intersection-de-deux-cercles.html
     * \param x1,y1,d1 Centre et rayon du cercle 1 (position départ, distance parcourue).
     * \param x2,y2,d2 Centre et rayon du cercle 2 (position capteur, distance mesurée).
     * \param robot_size_l_mm Demi-largeur du robot (pour filtrer la bonne solution).
     * \return tuple(erreur, x, y) : erreur<=0 si pas de solution, >0 sinon.
     */
    std::tuple<int, float, float> eq_2CirclesCrossed_getXY(float x1, float y1, float d1, float x2, float y2, float d2,
            float robot_size_l_mm);

    /*!
     * \brief Résout l'équation du 2nd degré issue de l'intersection des cercles.
     * \return tuple(erreur, x, y).
     */
    std::tuple<int, float, float> eq_2nd_deg_getXY(float a, float b, float A, float B, float C, float robot_size_l_mm);

    /*!
     * \brief Calcule le discriminant delta = B² - 4AC.
     */
    float eq_2nd_deg_getDelta(float A, float B, float C);

    /*!
     * \brief Corrige la position réelle du robot par triangulation avec un capteur de distance.
     *        Utilise la distance parcourue depuis un point connu + une mesure capteur
     *        pour recalculer (x, y, theta) par intersection de cercles.
     * \param pos_x_start_mm Position X de départ en mm.
     * \param pos_y_start_mm Position Y de départ en mm.
     * \param p Position courante du robot (odométrie).
     * \param delta_j_mm Offset X du capteur par rapport au centre du robot.
     * \param delta_k_mm Offset Y du capteur par rapport au centre du robot.
     * \param mesure_mm Distance mesurée par le capteur en mm.
     * \param robot_size_l_mm Demi-largeur du robot en mm (pour filtrer les solutions).
     * \return >0 si recalage réussi, <=0 sinon.
     */
    int adjustRealPosition(float pos_x_start_mm, float pos_y_start_mm, ROBOTPOSITION p, float delta_j_mm,
            float delta_k_mm, float mesure_mm, float robot_size_l_mm);

    /*!
     * \brief Corrige la dérive latérale du robot côté droit par mesure de bordure.
     *        Compare la distance théorique à la bordure avec la distance mesurée.
     *        Attention : la couleur de match doit déjà être appliquée.
     * \param d2_theo_bordure_mm Distance théorique à la bordure en mm.
     * \param d2b_bordure_mm Distance mesurée à la bordure en mm.
     * \param x_depart_mm Position X de départ en mm.
     * \param y_depart_mm Position Y de départ en mm.
     * \return true si correction appliquée (écart >= 5mm), false sinon.
     */
    bool calculateDriftRightSideAndSetPos(float d2_theo_bordure_mm, float d2b_bordure_mm, float x_depart_mm,
            float y_depart_mm);

    /*!
     * \brief Corrige la dérive latérale du robot côté gauche par mesure de bordure.
     * \param d2_theo_bordure_mm Distance théorique à la bordure en mm.
     * \param d2b_bordure_mm Distance mesurée à la bordure en mm.
     * \param x_depart_mm Position X de départ en mm.
     * \param y_depart_mm Position Y de départ en mm.
     * \return true si correction appliquée (écart >= 5mm), false sinon.
     */
    bool calculateDriftLeftSideAndSetPos(float d2_theo_bordure_mm, float d2b_bordure_mm, float x_depart_mm,
            float y_depart_mm);

    /*!
     * \brief Définit la couleur de match (détermine la symétrie X du terrain).
     * \param c false = couleur primaire (pas de symétrie), true = couleur secondaire (symétrie X).
     */
    void setMatchColorPosition(bool c)
    {
        matchColorPosition_ = c;
    }

    // ========== UTILITAIRES DE CONVERSION COULEUR/ANGLE ==========

    /*!
     * \brief Convertit une coordonnée X selon la couleur de match.
     *        Couleur primaire : retourne x_mm + width.
     *        Couleur secondaire : retourne (3000 - x_mm - width) pour la symétrie.
     * \param x_mm Coordonnée X en mm (repère couleur primaire).
     * \param width Décalage additionnel en mm (ex: demi-largeur robot).
     * \return Coordonnée X convertie.
     */
    inline float changeMatchX(float x_mm, float width = 0.0)
    {
        //printf("matchcolor:%d", matchColorPosition_);
        //logger().error() << "color==" << matchColorPosition_ << " width=" << width<< logs::end;
        if (matchColorPosition_ != 0) {
            return x_ground_table_ - x_mm - width;
        }
        return x_mm + width;
    }

    /*!
     * \brief Variante de changeMatchX : en couleur primaire, ne rajoute pas width.
     */
    inline float changeMatchXMin(float x_mm, float width = 0.0)
    {
        //printf("matchcolor:%d", matchColorPosition_);
        //logger().error() << "color==" << matchColorPosition_ << " width=" << width<< logs::end;
        if (matchColorPosition_ != 0) {
            return x_ground_table_ - x_mm - width;
        }
        return x_mm;
    }

    /*!
     * \brief Convertit un angle en radians selon la couleur de match.
     *        Couleur primaire : angle inchangé.
     *        Couleur secondaire : applique la symétrie (PI - angle), wrappé sur [0, 2PI].
     * \param rad Angle en radians (repère couleur primaire).
     * \return Angle converti en radians.
     */
    inline float changeMatchAngleRad(float rad)
    {
        if (matchColorPosition_ != 0) {
            float limit = (M_PI - rad);
            limit = WrapAngle2PI(limit);
            return limit;
        }
        return rad;
    }

    /*!
     * \brief Convertit des degrés en radians.
     */
    inline float degToRad(float deg)
    {
        return deg * M_PI / 180.0;
    }

    /*!
     * \brief Convertit des radians en degrés.
     */
    inline float radToDeg(float rad)
    {
        return rad * 180.0 / M_PI;
    }

};

#endif
