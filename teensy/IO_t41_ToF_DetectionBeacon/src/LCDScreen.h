/*
 * LCDScreen.hpp
 *
 *  Created on: Apr 12, 2024
 *      Author: pmx
 */

#ifndef LCDSCREEN_H_
#define LCDSCREEN_H_

#include <ILI9341_T4.h>

#include <lvgl.h> // see the comment above for infos about installing and configuring LVGL.


//
// DEFAULT WIRING USING SPI 0 ON TEENSY 4/4.1
//
#define PIN_SCK     13      // mandatory
#define PIN_MISO    12      // mandatory
#define PIN_MOSI    11      // mandatory
#define PIN_DC      10      // mandatory, can be any pin but using pin 10 (or 36 or 37 on T4.1) provides greater performance
#define PIN_CS      9       // mandatory when using the touchscreen on the same SPI bus, can be any pin.
#define PIN_RESET   255       // optional (but recommended), can be any pin.
#define PIN_BACKLIGHT 255   // optional. Set this only if the screen LED pin is connected directly to the Teensy

// XPT2046 TOUSCHSCREEN CONNECTED ON THE SAME SPI PORT AS ILI9341:
//
// - connect T_DIN to the same pin as PIN_MOSI
// - connect T_DO to the same pin as PIN_MISO
//
#define PIN_TOUCH_CS  7   // mandatory, can be any pin
#define PIN_TOUCH_IRQ 255   // (optional) can be any digital pin with interrupt capabilities




// 40MHz SPI. Can do much better with short wires
#define SPI_SPEED       40000000

// screen size in portrait mode
#define LX  320
#define LY  240


// number of lines in lvgl's internal draw buffer
#define BUF_LY 40


// Registers that the caller can both read and write
struct SettingsLCD {
    int8_t numOfBots = 3;         // Register 0. Number of Robot which may to be detected, default 3.
    int8_t ledDisplay;        // Register 1. Writable. Sets the mode for led display. 0 => OFF. 100 => FULL ON. 50 => Half luminosity.
    uint8_t tempNumber;              // Register 2.//TODO shift_rad angle en parametre ? //TODO ajouter le decalage d'angle en settings ?
    int8_t reserved = 0;      //NOT use yet
};

// Registers that the caller can only read
struct RegistersLCD {
    uint8_t flags = 0x80;        // Register 4. bit 0 => new data available
                                            //bit 7 => alive=1
    //TODO FLAGS
    //overflow more than 3 beacons
    //more than 6 contigus
    //default config settings is changed by master or not. in case of reset, the master can know and reconfigure

    //TODO UINT a mettre attention à la valeur -1 par defaut

    uint8_t nbDetectedBots = 0; //Register 5.Nombre de balises détectées.
    int16_t c1_mm = 0;        // Register 6.


};

void my_disp_flush(lv_disp_drv_t* disp, const lv_area_t* area, lv_color_t* color_p);
void my_touchpad_read(lv_indev_drv_t* indev_driver, lv_indev_data_t* data);
void setup_screen();
void ta_event_cb(lv_event_t* e);
void screen_loop();
//void on_read_isr_lcd(uint8_t reg_num);


#endif /* LCDSCREEN_H_ */
