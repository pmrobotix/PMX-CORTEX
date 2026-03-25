/***************************************************
 This is a library for the MCP23017 i2c port expander

 These displays use I2C to communicate, 2 pins are required to
 interface
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 Written by Limor Fried/Ladyada for Adafruit Industries.
 BSD license, all text above must be included in any redistribution
 ***************************************************/

#include "Adafruit_MCP23017.hpp"

Adafruit_MCP23017::Adafruit_MCP23017() :
        MCP_i2c_(1)
{
}

int Adafruit_MCP23017::begin(void)
{
    int ret = -1;

    //open i2c and setslave
    ret = MCP_i2c_.setSlaveAddr(MCP23017_ADDRESS);
    if (ret == -1)
        return ret;

    //setup
    ret = write_i2c(MCP23017_IODIRA, 0xFF); // all inputs on port A
    if (ret == -1)
        return ret;
    ret = write_i2c(MCP23017_IODIRB, 0xFF); // all inputs on port B
    if (ret == -1)
        return ret;

    return 0;
}

void Adafruit_MCP23017::pinMode(uint8_t p, uint8_t d)
{
    uint8_t iodir;
    uint8_t iodiraddr;

    // only 16 bits!
    if (p > 15)
        return;

    if (p < 8)
        iodiraddr = MCP23017_IODIRA;
    else {
        iodiraddr = MCP23017_IODIRB;
        p -= 8;
    }

    iodir = read_i2c(iodiraddr);

    // set the pin and direction
    if (d == INPUT) {
        iodir |= 1 << p;
    } else {
        iodir &= ~(1 << p);
    }

    write_i2c(iodiraddr, iodir);
}

uint16_t Adafruit_MCP23017::readGPIOAB()
{
    uint16_t ba = 0;
    uint8_t a;

    a = read_i2c(MCP23017_GPIOA); // TODO use readI2c_2Bytes ??
    ba = read_i2c(MCP23017_GPIOA + 1);
    ba <<= 8;
    ba |= a;

    return ba;
}

void Adafruit_MCP23017::writeGPIOAB(uint16_t ba)
{
    unsigned char buf[3];
    buf[0] = MCP23017_GPIOA;
    buf[1] = ba & 0xFF;
    buf[2] = ba >> 8;
    writeI2c_3Bytes(buf);
}

void Adafruit_MCP23017::digitalWrite(uint8_t p, uint8_t d)
{
    uint8_t gpio;
    uint8_t gpioaddr, olataddr;

    // only 16 bits!
    if (p > 15)
        return;

    if (p < 8) {
        olataddr = MCP23017_OLATA;
        gpioaddr = MCP23017_GPIOA;
    } else {
        olataddr = MCP23017_OLATB;
        gpioaddr = MCP23017_GPIOB;
        p -= 8;
    }

    gpio = read_i2c(olataddr);

    // set the pin and direction
    if (d == 1) //HIGH
    {
        gpio |= 1 << p;
    } else {
        gpio &= ~(1 << p);
    }

    // write the new GPIO
    write_i2c(gpioaddr, gpio);
}

void Adafruit_MCP23017::pullUp(uint8_t p, uint8_t d)
{
    uint8_t gppu;
    uint8_t gppuaddr;

    // only 16 bits!
    if (p > 15)
        return;

    if (p < 8)
        gppuaddr = MCP23017_GPPUA;
    else {
        gppuaddr = MCP23017_GPPUB;
        p -= 8;
    }

    gppu = read_i2c(gppuaddr);

    // set the pin and direction
    if (d == 1) //HIGH
    {
        gppu |= 1 << p;
    } else {
        gppu &= ~(1 << p);
    }

    // write the new GPIO
    write_i2c(gppuaddr, gppu);
}

uint8_t Adafruit_MCP23017::digitalRead(uint8_t p)
{
    uint8_t gpioaddr;

    // only 16 bits!
    if (p > 15)
        return 0;

    if (p < 8)
        gpioaddr = MCP23017_GPIOA;
    else {
        gpioaddr = MCP23017_GPIOB;
        p -= 8;
    }

    int ret = read_i2c(gpioaddr);
    return (ret >> p) & 0x1;
}

long Adafruit_MCP23017::write_i2c(unsigned char command, unsigned char value)
{
    long ret = -1;
    ret = MCP_i2c_.writeRegByte(command, value);
    return ret;
}

long Adafruit_MCP23017::read_i2c(unsigned char command)
{
    long ret = -1;
    ret = MCP_i2c_.readRegByte(command); //TODO  si la valeur vaut reelement -1 ?????
    return ret;
}

long Adafruit_MCP23017::writeI2c_3Bytes(unsigned char *buf)
{
    long ret = -1;
    ret = MCP_i2c_.write(buf, 3);
    return ret;
}

long Adafruit_MCP23017::readI2c_2Bytes(unsigned char *buf)
{
    long ret = -1;
    ret = MCP_i2c_.read(buf, 2);
    return ret;
}
