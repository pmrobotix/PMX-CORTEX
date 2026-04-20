//drivers...SIMU

#include "SensorsDriver.hpp"

#include <cmath>
#include <sstream>
#include <vector>

#include "log/Level.hpp"

using namespace std;

SensorsDriverSimu::SensorsDriverSimu(ARobotPositionShared *aRobotPositionShared)
{
    pos_pour_calcul_ = { 0 };
    pos_pour_calcul_prec_ = { 0 };
    robotPositionShared_ = aRobotPositionShared;
//    x_adv_ = 1000;
//    y_adv_ = 1400;

}

SensorsDriverSimu::~SensorsDriverSimu()
{
}

bool SensorsDriverSimu::is_connected()
{
    return true;
}

void SensorsDriverSimu::displayNumber(int number)
{

}

//a deplacer dans RobotPositionShared ?transformPosREPTABLE_to_PosREPROBOT Ne sert uniquement a la simu des sensors ?
//
// Convention alignee sur Sensors.cpp (docstring "Convention beacon : 0° = devant du robot") :
//   - theta_deg  : angle vers l'adv vu du robot. 0° = adv pile devant, sens trigo positif = gauche.
//   - y_rep_robot : composante avant (y > 0 = adv devant robot).
//   - x_rep_robot : composante laterale (x > 0 = adv a droite, x < 0 = gauche).
//
// Sensors::front reconstruit la position table avec :
//   a_table = robot.theta + theta_deg_rad
//   x_adv_table = robot.x + d * cos(a_table)
//   y_adv_table = robot.y + d * sin(a_table)
// Ce qui doit redonner (x_table, y_table). On verifie la coherence ci-dessous.
RobotPos SensorsDriverSimu::transformPosTableToPosRobot(int nb, float x_table, float y_table)
{
    loggerSvg().info() << "<circle cx=\"" << x_table << "\" cy=\"" << -y_table << "\" r=\"5\" fill=\"red\" />"
            << logs::end;

    ROBOTPOSITION p = pos_pour_calcul_;

    loggerSvg().info() << "<circle cx=\"" << p.x << "\" cy=\"" << -p.y << "\" r=\"3\" fill=\"blue\" />" << "<line x1=\""
            << p.x << "\" y1=\"" << -p.y << "\" x2=\"" << p.x + cos(p.theta) * 25 << "\" y2=\""
            << -p.y - sin(p.theta) * 25 << "\" stroke-width=\"0.1\" stroke=\"grey\"  />" << logs::end;

    // Vecteur robot -> adv en coord table
    float dx = x_table - p.x;
    float dy = y_table - p.y;
    float d  = std::sqrt(dx * dx + dy * dy);

    // Angle absolu du vecteur puis relatif au robot (0 = adv devant)
    float alpha_rad = std::atan2(dy, dx) - p.theta;
    alpha_rad = WrapAngle2PI(alpha_rad);
    float alpha_deg = alpha_rad * 180.0f / (float)M_PI;

    // Projection dans le repere robot :
    //  axe "avant" (y_rob) = direction theta  : cos(theta), sin(theta)
    //  axe "droite" (x_rob) = perpendiculaire droite : sin(theta), -cos(theta)
    float y_rep_robot = dx * std::cos(p.theta) + dy * std::sin(p.theta);
    float x_rep_robot = dx * std::sin(p.theta) - dy * std::cos(p.theta);

    logger().debug() << __FUNCTION__ << " (" << x_table << "," << y_table
            << ") dx=" << dx << " dy=" << dy << " d=" << d
            << " p.theta=" << (p.theta * 180.0f / M_PI) << "deg"
            << " -> x_rob=" << x_rep_robot << " y_rob=" << y_rep_robot
            << " theta_rob=" << alpha_deg << "deg" << logs::end;

    RobotPos pos = { nb, x_rep_robot, y_rep_robot, alpha_deg, d };

    return pos;
}

ASensorsDriver::bot_positions SensorsDriverSimu::getvPositionsAdv()
{
    return vadv_;
}


//FOR TEST ONLY
void SensorsDriverSimu::addvPositionsAdv(float x, float y)
{
	int nb = 1;
	RobotPos pos1 = transformPosTableToPosRobot(nb, x, y);
	vadv_.push_back(pos1);
}
//FOR TEST ONLY
void SensorsDriverSimu::clearPositionsAdv()
{
	vadv_.clear();
}

void SensorsDriverSimu::setInjectedAdv(float x_table_mm, float y_table_mm)
{
	injectedAdvX_ = x_table_mm;
	injectedAdvY_ = y_table_mm;
	injectedAdvEnabled_ = true;
}

void SensorsDriverSimu::clearInjectedAdv()
{
	injectedAdvEnabled_ = false;
}



int SensorsDriverSimu::sync()
{
    // Capture de la position robot au moment du sync (utile pour la conversion
    // table -> repere robot dans transformPosTableToPosRobot).
    pos_pour_calcul_ = robotPositionShared_->getRobotPosition(0);

    // Timestamp du sync, meme reference (chrono_) que pushHistory() cote asserv.
    // Sans ca, Sensors::sensorOnTimer calcule t_mesure_ms en underflow et
    // getPositionAt renvoie une position invalide -> frontLevel reste a 0.
    last_sync_ms_ = (uint32_t)(robotPositionShared_->chrono_.getElapsedTimeInMicroSec() / 1000);

    vadv_.clear();

    // Republication de l'adv injecte (persistant entre sync) pour les tests
    // de scenarios. Les drivers ARM ne font rien : l'adv vient de la balise.
    if (injectedAdvEnabled_) {
        RobotPos pos = transformPosTableToPosRobot(1, injectedAdvX_, injectedAdvY_);
        vadv_.push_back(pos);
    }

    // Retourne > 0 pour signaler "nouvelle frame beacon disponible" au SensorsThread.
    // Sans ca, sensorOnTimer skipperait tout le traitement (filtrage + publication
    // du DetectionEvent). En ARM ce code simule ce que fait la vraie balise quand
    // son seq beacon change.
    return 1;
}

int SensorsDriverSimu::rightSide()
{
    return 400;
}
int SensorsDriverSimu::leftSide()
{
    return 400;
}

int SensorsDriverSimu::frontLeft()
{
    return 999;
}
int SensorsDriverSimu::frontCenter()
{
    return 999;

}
int SensorsDriverSimu::frontRight()
{
    //TODO temp mettre un ifdef pour simulation gros robot et petit robot
    /*
     OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();

     if (robot.asserv().pos_getX_mm() > 800)
     {
     robot.svgPrintPosition(3);
     return 300;
     }
     else*/
    return 999;
}

int SensorsDriverSimu::backLeft()
{
    return 999;
}
int SensorsDriverSimu::backCenter()
{
    return 999;
}
int SensorsDriverSimu::backRight()
{
    return 999;
}

SvgWriterExtended::SvgWriterExtended(std::string botId) :
        SvgWriter(botId) //on appelle le constructeur pere
{

    std::ostringstream symbol;

    symbol << "<symbol id=\"bot-legoev3\">"
            << "<circle cx=\"140\" cy=\"140\" r=\"140\" fill=\"none\" stroke=\"slategray\" stroke-width=\"0.5\" stroke-dasharray=\"4,8\"/>"
            << "<circle cx=\"140\" cy=\"140\" r=\"10\" fill=\"none\" stroke=\"slategray\" />"
            << "<line x1=\"100\" y1=\"15\" x2=\"180\" y2=\"15\" stroke=\"slategray\"/>"
            << "<line x1=\"100\" y1=\"265\" x2=\"180\" y2=\"265\" stroke=\"slategray\"/>"
            << "<line x1=\"15\" y1=\"100\" x2=\"15\" y2=\"180\" stroke=\"slategray\"/>"
            << "<line x1=\"265\" y1=\"100\" x2=\"265\" y2=\"180\" stroke=\"slategray\" stroke-width=\"2\"/>"
            << "<path d=\"M100,15A85,85 0 0,0 15,100\" fill=\"none\" stroke=\"slategray\" ></path>"
            << "<path d=\"M180,15A85,85 0 0,1 265,100\" fill=\"none\" stroke=\"slategray\" ></path>"
            << "<path d=\"M265,180A85,85 0 0,1 180,265\" fill=\"none\" stroke=\"slategray\" ></path>"
            << "<path d=\"M15,180A85,85 0 0,0 100,265\" fill=\"none\" stroke=\"slategray\" ></path>" << "</symbol>";
    symbol << "<symbol id=\"bot-legoev3-RED\">"
            << "<circle cx=\"140\" cy=\"140\" r=\"140\" fill=\"none\" stroke=\"red\" stroke-width=\"0.5\" stroke-dasharray=\"4,8\"/>"
            << "<circle cx=\"140\" cy=\"140\" r=\"8\" fill=\"none\" stroke=\"red\" />"
            << "<line x1=\"100\" y1=\"15\" x2=\"180\" y2=\"15\" stroke=\"red\" stroke-width=\"2\"/>"
            << "<line x1=\"100\" y1=\"265\" x2=\"180\" y2=\"265\" stroke=\"red\" stroke-width=\"2\"/>"
            << "<line x1=\"15\" y1=\"100\" x2=\"15\" y2=\"180\" stroke=\"red\" stroke-width=\"2\"/>"
            << "<line x1=\"265\" y1=\"100\" x2=\"265\" y2=\"180\" stroke=\"red\" stroke-width=\"4\"/>"
            << "<path d=\"M100,15A85,85 0 0,0 15,100\" fill=\"none\" stroke=\"red\" ></path>"
            << "<path d=\"M180,15A85,85 0 0,1 265,100\" fill=\"none\" stroke=\"red\" ></path>"
            << "<path d=\"M265,180A85,85 0 0,1 180,265\" fill=\"none\" stroke=\"red\" ></path>"
            << "<path d=\"M15,180A85,85 0 0,0 100,265\" fill=\"none\" stroke=\"red\" ></path>" << "</symbol>";
    symbol << "<symbol id=\"bot-legoev3-ORANGE\">"
            << "<circle cx=\"140\" cy=\"140\" r=\"140\" fill=\"none\" stroke=\"orange\" stroke-width=\"0.5\" stroke-dasharray=\"4,8\"/>"
            << "<circle cx=\"140\" cy=\"140\" r=\"7\" fill=\"none\" stroke=\"orange\" />"
            << "<line x1=\"100\" y1=\"15\" x2=\"180\" y2=\"15\" stroke=\"orange\" stroke-width=\"2\"/>"
            << "<line x1=\"100\" y1=\"265\" x2=\"180\" y2=\"265\" stroke=\"orange\" stroke-width=\"2\"/>"
            << "<line x1=\"15\" y1=\"100\" x2=\"15\" y2=\"180\" stroke=\"orange\" stroke-width=\"2\"/>"
            << "<line x1=\"265\" y1=\"100\" x2=\"265\" y2=\"180\" stroke=\"orange\" stroke-width=\"4\"/>"
            << "<path d=\"M100,15A85,85 0 0,0 15,100\" fill=\"none\" stroke=\"orange\" ></path>"
            << "<path d=\"M180,15A85,85 0 0,1 265,100\" fill=\"none\" stroke=\"orange\" ></path>"
            << "<path d=\"M265,180A85,85 0 0,1 180,265\" fill=\"none\" stroke=\"orange\" ></path>"
            << "<path d=\"M15,180A85,85 0 0,0 100,265\" fill=\"none\" stroke=\"orange\" ></path>" << "</symbol>";
    symbol << "<symbol id=\"bot-legoev3-GREEN\">"
            << "<circle cx=\"140\" cy=\"140\" r=\"140\" fill=\"none\" stroke=\"green\" stroke-width=\"0.5\" stroke-dasharray=\"4,8\"/>"
            << "<circle cx=\"140\" cy=\"140\" r=\"6\" fill=\"none\" stroke=\"green\" />"
            << "<line x1=\"100\" y1=\"15\" x2=\"180\" y2=\"15\" stroke=\"green\" stroke-width=\"2\"/>"
            << "<line x1=\"100\" y1=\"265\" x2=\"180\" y2=\"265\" stroke=\"green\" stroke-width=\"2\"/>"
            << "<line x1=\"15\" y1=\"100\" x2=\"15\" y2=\"180\" stroke=\"green\" stroke-width=\"2\"/>"
            << "<line x1=\"265\" y1=\"100\" x2=\"265\" y2=\"180\" stroke=\"green\" stroke-width=\"4\"/>"
            << "<path d=\"M100,15A85,85 0 0,0 15,100\" fill=\"none\" stroke=\"green\" ></path>"
            << "<path d=\"M180,15A85,85 0 0,1 265,100\" fill=\"none\" stroke=\"green\" ></path>"
            << "<path d=\"M265,180A85,85 0 0,1 180,265\" fill=\"none\" stroke=\"green\" ></path>"
            << "<path d=\"M15,180A85,85 0 0,0 100,265\" fill=\"none\" stroke=\"green\" ></path>" << "</symbol>";
    symbol << "<symbol id=\"bot-legoev3-BLUE\">"
            << "<circle cx=\"140\" cy=\"140\" r=\"140\" fill=\"none\" stroke=\"blue\" stroke-width=\"0.5\" stroke-dasharray=\"4,8\"/>"
            << "<circle cx=\"140\" cy=\"140\" r=\"5\" fill=\"none\" stroke=\"blue\" />"
            << "<line x1=\"100\" y1=\"15\" x2=\"180\" y2=\"15\" stroke=\"blue\" stroke-width=\"2\"/>"
            << "<line x1=\"100\" y1=\"265\" x2=\"180\" y2=\"265\" stroke=\"blue\" stroke-width=\"2\"/>"
            << "<line x1=\"15\" y1=\"100\" x2=\"15\" y2=\"180\" stroke=\"blue\" stroke-width=\"2\"/>"
            << "<line x1=\"265\" y1=\"100\" x2=\"265\" y2=\"180\" stroke=\"blue\" stroke-width=\"4\"/>"
            << "<path d=\"M100,15A85,85 0 0,0 15,100\" fill=\"none\" stroke=\"blue\" ></path>"
            << "<path d=\"M180,15A85,85 0 0,1 265,100\" fill=\"none\" stroke=\"blue\" ></path>"
            << "<path d=\"M265,180A85,85 0 0,1 180,265\" fill=\"none\" stroke=\"blue\" ></path>"
            << "<path d=\"M15,180A85,85 0 0,0 100,265\" fill=\"none\" stroke=\"blue\" ></path>" << "</symbol>";
    symbol << "<symbol id=\"bot-legoev3-BLACK\">"
            << "<circle cx=\"140\" cy=\"140\" r=\"140\" fill=\"none\" stroke=\"black\" stroke-width=\"0.5\" stroke-dasharray=\"4,8\"/>"
            << "<circle cx=\"140\" cy=\"140\" r=\"4\" fill=\"none\" stroke=\"black\" />"
            << "<line x1=\"100\" y1=\"15\" x2=\"180\" y2=\"15\" stroke=\"black\" stroke-width=\"2\"/>"
            << "<line x1=\"100\" y1=\"265\" x2=\"180\" y2=\"265\" stroke=\"black\" stroke-width=\"2\"/>"
            << "<line x1=\"15\" y1=\"100\" x2=\"15\" y2=\"180\" stroke=\"black\" stroke-width=\"2\"/>"
            << "<line x1=\"265\" y1=\"100\" x2=\"265\" y2=\"180\" stroke=\"black\" stroke-width=\"4\"/>"
            << "<path d=\"M100,15A85,85 0 0,0 15,100\" fill=\"none\" stroke=\"black\" ></path>"
            << "<path d=\"M180,15A85,85 0 0,1 265,100\" fill=\"none\" stroke=\"black\" ></path>"
            << "<path d=\"M265,180A85,85 0 0,1 180,265\" fill=\"none\" stroke=\"black\" ></path>"
            << "<path d=\"M15,180A85,85 0 0,0 100,265\" fill=\"none\" stroke=\"black\" ></path>" << "</symbol>";

    symbol << "<symbol id=\"botadv-legoev3\">"
            << "<circle cx=\"150\" cy=\"150\" r=\"150\" fill=\"none\" stroke=\"slategray\" stroke-width=\"0.5\" stroke-dasharray=\"4,8\"/>"
            << "<circle cx=\"140\" cy=\"140\" r=\"10\" fill=\"none\" stroke=\"slategray\" />" << "</symbol>";
    symbol << "<symbol id=\"botadv-legoev3-RED\">"
            << "<circle cx=\"150\" cy=\"150\" r=\"150\" fill=\"none\" stroke=\"slategray\" stroke-width=\"0.5\" stroke-dasharray=\"4,8\"/>"
            << "<circle cx=\"140\" cy=\"140\" r=\"10\" fill=\"none\" stroke=\"slategray\" />" << "</symbol>";

    //<circle cx="140" cy="140" r="140" fill="none" stroke="slategray" stroke-width="0.5" stroke-dasharray="4,8"/>
    //<circle cx="140" cy="140" r="4" fill="none" stroke="slategray" />
    //<!--rect x="15" y="15" width="250" height="250" style="fill:none;stroke:orange;stroke-width:2px;" /-->
    //<line x1="100" y1="15" x2="180" y2="15" stroke="slategray"/>
    //<line x1="100" y1="265" x2="180" y2="265" stroke="slategray"/>
    //
    //<line x1="15" y1="100" x2="15" y2="180" stroke="slategray"/>
    //
    //<line x1="265" y1="100" x2="265" y2="180" stroke="slategray" stroke-width="4"/>
    //
    //<path d="M100,15A85,85 0 0,0 15,100" fill="none" stroke="slategray" ></path>
    //<path d="M180,15A85,85 0 0,1 265,100" fill="none" stroke="slategray" ></path>
    //<path d="M265,180A85,85 0 0,1 180,265" fill="none" stroke="slategray" ></path>
    //<path d="M15,180A85,85 0 0,0 100,265" fill="none" stroke="slategray" ></path>

    addDefsSymbol(symbol.str());

    //optimisation du log pour ne créer qu'un seul objet
    fLogBuffer = new logs::Logger::LoggerBuffer(logger(), logs::Level::INFO);
}

//angle en radian
void SvgWriterExtended::writePosition_BotPos(float x, float y, float angle_rad)
{
    if (!done_) {
        float delta_y = 50.0 * sin(angle_rad);
        float delta_x = 50.0 * cos(angle_rad);
        // inversion du y pour affichage dans le bon sens dans le SVG
        this->lock();
        *fLogBuffer << "<circle cx=\"" << x << "\" cy=\"" << -y << "\" r=\"1\" fill=\"blue\"/>" "<line x1=\"" << x
                << "\" y1=\"" << -y << "\" x2=\"" << x + delta_x << "\" y2=\"" << -y - delta_y
                << "\" stroke-width=\"0.1\" stroke=\"grey\"/>" << logs::flush;
        this->unlock();
    }
}

void SvgWriterExtended::writePosition_Bot(float x, float y, float angle_rad, int color)
{
    if (!done_) {
        this->lock();
        if (color == 0) {
            *fLogBuffer << "<use x=\"" << x - 140 << "\" y=\"" << -y - 140
                    << "\" xlink:href=\"#bot-legoev3\" transform=\"rotate(" << -((angle_rad * 180) / M_PI) << "," << x
                    << "," << -y << ")\" />" << logs::flush;
        } else if (color == 1) {
            *fLogBuffer << "<use x=\"" << x - 140 << "\" y=\"" << -y - 140
                    << "\" xlink:href=\"#bot-legoev3-ORANGE\" transform=\"rotate(" << -((angle_rad * 180) / M_PI) << ","
                    << x << "," << -y << ")\" />" << logs::flush;
        } else if (color == 2) {
            *fLogBuffer << "<use x=\"" << x - 140 << "\" y=\"" << -y - 140
                    << "\" xlink:href=\"#bot-legoev3-RED\" transform=\"rotate(" << -((angle_rad * 180) / M_PI) << ","
                    << x << "," << -y << ")\" />" << logs::flush;
        } else if (color == 3) {
            *fLogBuffer << "<use x=\"" << x - 140 << "\" y=\"" << -y - 140
                    << "\" xlink:href=\"#bot-legoev3-GREEN\" transform=\"rotate(" << -((angle_rad * 180) / M_PI) << ","
                    << x << "," << -y << ")\" />" << logs::flush;
        } else if (color == 4) {
            *fLogBuffer << "<use x=\"" << x - 140 << "\" y=\"" << -y - 140
                    << "\" xlink:href=\"#bot-legoev3-BLUE\" transform=\"rotate(" << -((angle_rad * 180) / M_PI) << ","
                    << x << "," << -y << ")\" />" << logs::flush;
        } else if (color == 5) {
            *fLogBuffer << "<use x=\"" << x - 140 << "\" y=\"" << -y - 140
                    << "\" xlink:href=\"#bot-legoev3-BLACK\" transform=\"rotate(" << -((angle_rad * 180) / M_PI) << ","
                    << x << "," << -y << ")\" />" << logs::flush;
        }
        this->unlock();
    }
}

void SvgWriterExtended::writePosition_AdvPos(float x, float y, float x_pos_mm, float y_pos_mm, int color)
{

    if (!done_) {
        int r_adv = 400 / 2; //TODO rendre parametrable
        this->lock();

        if (color == 0) {
            *fLogBuffer //<< "<use x=\"" << x - dd << "\" y=\"" << -y - dd << "\" xlink:href=\"#bot-OPOS6UL\" />"
            << "<circle cx=\"" << x << "\" cy=\"" << -y << "\" r=\"" << r_adv
                    << "\" fill=\"none\" stroke=\"slategray\" />" << "<line x1=\"" << x_pos_mm << "\" y1=\""
                    << -y_pos_mm << "\" x2=\"" << x << "\" y2=\"" << -y
                    << "\" stroke=\"slategray\" stroke-width=\"1\"/>" << logs::flush;
        } else if (color == 1) {
            *fLogBuffer //<< "<use x=\"" << x - dd << "\" y=\"" << -y - dd << "\" xlink:href=\"#bot-OPOS6UL-ORANGE\"  />"
            << "<circle cx=\"" << x << "\" cy=\"" << -y << "\" r=\"" << r_adv + 6
                    << "\" fill=\"none\" stroke=\"orange\" />" << "<line x1=\"" << x_pos_mm << "\" y1=\"" << -y_pos_mm
                    << "\" x2=\"" << x << "\" y2=\"" << -y << "\" stroke=\"ORANGE\" stroke-width=\"1\"/>"
                    << logs::flush;
        } else if (color == 2) {
            *fLogBuffer //<< "<use x=\"" << x - dd << "\" y=\"" << -y - dd << "\" xlink:href=\"#bot-OPOS6UL-RED\" />"
            << "<circle cx=\"" << x << "\" cy=\"" << -y << "\" r=\"" << r_adv + 10
                    << "\" fill=\"none\" stroke=\"red\" />" << "<circle cx=\"" << x << "\" cy=\"" << -y << "\" r=\""
                    << 150 << "\" fill=\"none\" stroke=\"red\" />" << "<line x1=\"" << x_pos_mm << "\" y1=\""
                    << -y_pos_mm << "\" x2=\"" << x << "\" y2=\"" << -y << "\" stroke=\"RED\" stroke-width=\"1\"/>"
                    << logs::flush;
        } else if (color == 3) {
            *fLogBuffer //<< "<use x=\"" << x - dd << "\" y=\"" << -y - dd << "\" xlink:href=\"#bot-OPOS6UL-GREEN\"  />"
            << "<circle cx=\"" << x << "\" cy=\"" << -y << "\" r=\"" << r_adv + 4
                    << "\" fill=\"none\" stroke=\"green\" />" << "<line x1=\"" << x_pos_mm << "\" y1=\"" << -y_pos_mm
                    << "\" x2=\"" << x << "\" y2=\"" << -y << "\" stroke=\"GREEN\" stroke-width=\"1\"/>" << logs::flush;
        } else if (color == 4) {
            *fLogBuffer //<< "<use x=\"" << x - dd << "\" y=\"" << -y - dd << "\" xlink:href=\"#bot-OPOS6UL-BLUE\" />"
            << "<circle cx=\"" << x << "\" cy=\"" << -y << "\" r=\"" << r_adv + 2
                    << "\" fill=\"none\" stroke=\"blue\" />" << "<line x1=\"" << x_pos_mm << "\" y1=\"" << -y_pos_mm
                    << "\" x2=\"" << x << "\" y2=\"" << -y << "\" stroke=\"BLUE\" stroke-width=\"1\"/>" << logs::flush;
        } else if (color == 5) {
            *fLogBuffer //<< "<use x=\"" << x - dd << "\" y=\"" << -y - dd << "\" xlink:href=\"#bot-OPOS6UL-BLACK\" />"
            << "<circle cx=\"" << x << "\" cy=\"" << -y << "\" r=\"" << r_adv + 8
                    << "\" fill=\"none\" stroke=\"black\" />" << "<line x1=\"" << x_pos_mm << "\" y1=\"" << -y_pos_mm
                    << "\" x2=\"" << x << "\" y2=\"" << -y << "\" stroke=\"BLACK\" stroke-width=\"1\"/>" << logs::flush;
        }
        this->unlock();
    }

}
