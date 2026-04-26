/*!
 * \file
 * \brief Implémentation de la classe OPOS6UL_AsservExtended.
 */

#include "OPOS6UL_AsservExtended.hpp"

#include <cmath>
#include <cstdio>

#include "asserv.esial/AsservEsialR.hpp"
#include "config/config.h"
#include "log/Logger.hpp"
#include "thread/Thread.hpp"

#include "OPOS6UL_IAExtended.hpp"
#include "OPOS6UL_RobotExtended.hpp"

OPOS6UL_AsservExtended::OPOS6UL_AsservExtended(std::string botId, OPOS6UL_RobotExtended *robot) :
        Asserv(botId, robot) //on appelle le constructeur pere
{
    botId_ = botId;

    //useAsservType_ = ASSERV_INT_ESIALR; //ASSERV_EXT;
    useAsservType_ = ASSERV_EXT;

    robot_extended_ = robot;

    //set the value setLowSpeedForward for asserv
    setLowSpeedvalue(10);

    setMaxSpeedDistValue(20);

    //TODO essayer de surcharger les asservdriver pour avoir accès que log SVG?
    //TODO asservdriver_ = AAsservDriver::create(botId, robot->svgw());
}

//TODO séparer config du lancement de l'asserv
void OPOS6UL_AsservExtended::startMotionTimerAndOdo(bool assistedHandlingEnabled)
{
    // internal asserv config
    if (useAsservType_ == ASSERV_INT_ESIALR) {

        std::string filename = "config_";
        filename.append(botId_);
        filename.append(".txt");
        logger().debug() << filename.c_str() << logs::end;
        Config::loadFile(filename.c_str());
        logger().debug() << "Version configuration : " << Config::configVersion << logs::end;
        logger().debug() << Config::dumpConfig() << logs::end;

        float periodSec = Config::asservPeriod;

        //on active le thread de check de position et les drivers //TODO a faire dans esialR et non dans asservdriver
        //asservdriver_->motion_ActivateManager(true);

        //start asserv thread et timer
        pAsservEsialR_->startAsserv(1.0f / periodSec); //f=20 Hz => every 50ms

        pAsservEsialR_->motion_ActivateManager(true); //init and start
        if (assistedHandlingEnabled) {
            pAsservEsialR_->motion_AssistedHandling();
        } else {
            pAsservEsialR_->motion_FreeMotion();
        }

    } else if (useAsservType_ == ASSERV_EXT) {

        // Stabilisation Nucleo : la frame CBOR position est emise a 10 Hz (100ms).
        // Apres setPositionAndColor (envoye juste avant cet appel), sharedPosition
        // reflete encore l'ancienne pose (souvent (0,0,0) au boot OPOS6UL). Sans
        // cette pause, le 1er Navigator::line() qui suit capture pBefore=(0,0)
        // -> calcul d_parcourue/d_restant errone (le robot bouge bien la bonne
        // distance physique demandee a la Nucleo, mais le code sait pas qu'il a
        // commence a (230, 130) et croit avoir parcouru 309mm pour une LINE 80).
        // 200ms = 2 frames CBOR + marge.
        utils::sleep_for_millis(200);

        asservdriver_->motion_ActivateManager(true); //on active la carte d'asserv externe et le thread de position
        if (assistedHandlingEnabled)
            asservdriver_->motion_AssistedHandling();
        else
            asservdriver_->motion_FreeMotion();
    }
}

/* old version 2021
 bool OPOS6UL_AsservExtended::filtre_IsInsideTable(int dist_detect_mm, int lateral_pos_sensor_mm, std::string desc)
 {

 //logger().error() << "==== filtreInsideTable" << logs::end;
 float distmm = dist_detect_mm;
 //On filtre si c'est pas à l'exterieur du terrain
 float x = 0.0;
 float y = 0.0;
 bool result = false;
 RobotPosition p = pos_getPosition();
 x = p.x + ((lateral_pos_sensor_mm * 140.0 ) * cos(p.theta - M_PI_2)) + (distmm * cos(p.theta));
 y = p.y + ((lateral_pos_sensor_mm * 140.0 ) * sin(p.theta - M_PI_2)) + (distmm * sin(p.theta));
 if ((x > 150 && x < 2850) && (y > 150 && y < 1850)) //en mètre
 result = true;
 else
 result = false;
 logger().debug() << desc << " filtreInsideTable : dist=" << dist_detect_mm
 << " capteur:" << lateral_pos_sensor_mm
 << " p.x=" << p.x << " p.y=" << p.y << " p.T=" << p.theta << " x=" << x
 << " y=" << y << " result = " << result << logs::end;

 if (result) {
 logger().debug() << desc << " filtreInsideTable : dist=" << dist_detect_mm
 << " capteur:" << lateral_pos_sensor_mm
 << " p.x=" << p.x << " p.y=" << p.y << " p.T=" << p.theta << " x=" << x
 << " y=" << y << " result = " << result << logs::end;
 return true; //si ok
 } else
 return false; //si en dehors de la table
 }*/

//void OPOS6UL_AsservExtended::setLowSpeedForward(bool enable, int percent)
//{
//    if (percent < 0) percent = 1;
//    if (percent > 100) percent = 100;
//
//    logger().debug() << "OPOS6UL_AsservExtended::setLowSpeedForward = " << enable << logs::end;
//    //Asserv::setLowSpeedForward(enable, 45);
//    asservdriver_->motion_setLowSpeedForward(enable, percent);
//}
//void OPOS6UL_AsservExtended::setLowSpeedBackward(bool enable, int)
//{
//    logger().debug() << "OPOS6UL_AsservExtended::setLowSpeedBackward =" << enable << logs::end;
//    //Asserv::setLowSpeedBackward(enable, 45);
//    asservdriver_->motion_setLowSpeedBackward(enable, 55);
//}
//TODO a deplacer dans les actions avec l'update des sensors
void OPOS6UL_AsservExtended::update_adv()
{
    //TODO a deplacer en IA ?
    robot_extended_->ia().move_adv(pos_getAdvPosition().x, pos_getAdvPosition().y);

}
