#include "LcdShield.hpp"

#include "interface/ALcdShieldDriver.hpp"

using namespace std;

LcdShield::LcdShield(std::string botId, Actions & actions) :
        AActionsElement(actions), botId_(botId)
{
    lcdshielddriver_ = ALcdShieldDriver::create(botId);
    if(!(lcdshielddriver_->is_connected()))
    {
        logger().debug() << "lcdshielddriver_->is_connected() FALSE !" << logs::end;
    }
}

LcdShield::~LcdShield()
{
    delete lcdshielddriver_;
}

void LcdShield::clear()
{
    lcdshielddriver_->clear();
}

void LcdShield::home()
{
    lcdshielddriver_->home();
}
void LcdShield::setBacklightOn()
{
    lcdshielddriver_->setBacklightOn();
}
void LcdShield::setBacklightOff()
{
    lcdshielddriver_->setBacklightOff();
}

void LcdShield::setCursor(uint8_t col, uint8_t row)
{
    lcdshielddriver_->setCursor(col, row);
}

bool LcdShield::is_connected()
{
    return lcdshielddriver_->is_connected();
}

void LcdShield::init()
{
    clear();
    setBacklightOn();
}

void LcdShield::reset()
{
    clear();
    setCursor(0, 0);
}

void LcdShield::print(const char* str)
{
    lcdshielddriver_->print_content_string(str, 0, 0);
}

void LcdShield::print(const std::string& str)
{
    lcdshielddriver_->print_content_string(str, 0, 0);
}

void LcdShield::print(int value)
{
    lcdshielddriver_->print_content_integer(value, 0, 0);
}

void LcdShield::println(const char* str)
{
    lcdshielddriver_->print_content_string(str, 0, 0);
}
