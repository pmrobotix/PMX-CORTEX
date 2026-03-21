/*
 * LedPanels.h
 *
 *  Created on: Jan 25, 2021
 *      Author: cho (PM-ROBOTIX)
 */

#ifndef LEDPANELS_H_
#define LEDPANELS_H_

#include <WS2812Serial.h>
#define USE_WS2812SERIAL //doit etre placé après #include <WS2812Serial.h> pour utiliser la librairie compatible teensy4

#include <Adafruit_GFX.h>
#include <FastLED.h>
#include <FastLED_NeoMatrix.h>
#include <LEDMatrix.h>
#include "TeensyThreads.h"
#include "fonts.h"
#include <Fonts/Picopixel.h>

#define BRIGHTNESS 10//70

#define DISABLE_WHITE

// Used by LEDMatrix
#define MATRIX_TILE_WIDTH   4 // width of EACH NEOPIXEL MATRIX (not total display)
#define MATRIX_TILE_HEIGHT  4 // height of each matrix
#define MATRIX_TILE_H       9  // number of matrices arranged horizontally
#define MATRIX_TILE_V       2  // number of matrices arranged vertically

// Used by NeoMatrix
#define mw (MATRIX_TILE_WIDTH *  MATRIX_TILE_H)
#define mh (MATRIX_TILE_HEIGHT * MATRIX_TILE_V)
#define NUMMATRIX (mw*mh)

// Compat for some other demos
#define NUM_LEDS NUMMATRIX
#define MATRIX_HEIGHT mh
#define MATRIX_WIDTH mw

#define PIN 8 //serial PIN for WS2812

// This could also be defined as matrix->color(255,0,0) but those defines
// are meant to work for adafruit_gfx backends that are lacking color()
#define LED_BLACK             0

#define LED_RED_VERYLOW       (3 <<  11)
#define LED_RED_LOW                     (7 <<  11)
#define LED_RED_MEDIUM                  (15 << 11)
#define LED_RED_HIGH                    (31 << 11)

#define LED_GREEN_VERYLOW     (1 <<  5)
#define LED_GREEN_LOW                   (15 << 5)
#define LED_GREEN_MEDIUM      (31 << 5)
#define LED_GREEN_HIGH                  (63 << 5)

#define LED_BLUE_VERYLOW      3
#define LED_BLUE_LOW                    7
#define LED_BLUE_MEDIUM       15
#define LED_BLUE_HIGH                   31

#define LED_ORANGE_VERYLOW    (LED_RED_VERYLOW + LED_GREEN_VERYLOW)
#define LED_ORANGE_LOW                  (LED_RED_LOW     + LED_GREEN_LOW)
#define LED_ORANGE_MEDIUM     (LED_RED_MEDIUM  + LED_GREEN_MEDIUM)
#define LED_ORANGE_HIGH                 (LED_RED_HIGH    + LED_GREEN_HIGH)

#define LED_PURPLE_VERYLOW    (LED_RED_VERYLOW + LED_BLUE_VERYLOW)
#define LED_PURPLE_LOW                  (LED_RED_LOW     + LED_BLUE_LOW)
#define LED_PURPLE_MEDIUM     (LED_RED_MEDIUM  + LED_BLUE_MEDIUM)
#define LED_PURPLE_HIGH                 (LED_RED_HIGH    + LED_BLUE_HIGH)

#define LED_CYAN_VERYLOW      (LED_GREEN_VERYLOW + LED_BLUE_VERYLOW)
#define LED_CYAN_LOW                    (LED_GREEN_LOW     + LED_BLUE_LOW)
#define LED_CYAN_MEDIUM                 (LED_GREEN_MEDIUM  + LED_BLUE_MEDIUM)
#define LED_CYAN_HIGH                   (LED_GREEN_HIGH    + LED_BLUE_HIGH)

#define LED_WHITE_VERYLOW     (LED_RED_VERYLOW + LED_GREEN_VERYLOW + LED_BLUE_VERYLOW)
#define LED_WHITE_LOW                   (LED_RED_LOW     + LED_GREEN_LOW     + LED_BLUE_LOW)
#define LED_WHITE_MEDIUM      (LED_RED_MEDIUM  + LED_GREEN_MEDIUM  + LED_BLUE_MEDIUM)
#define LED_WHITE_HIGH                  (LED_RED_HIGH    + LED_GREEN_HIGH    + LED_BLUE_HIGH)

void matrix_clear();

void ledPanels_loop(int debug = 0);
void ledPanels_setup();
void ledPanels_setup2();

void thread_display();
void showFirstLedLine();
void add_display_VL_out();
void display_intro();

void add_display_collision();
void add_display_black_dist();
void add_display_pmx();
void add_display_data();
void add_display_turn_infinite();
void add_display_turnhalf_infinite();

void display_UA();
void add_display_PointsNumber();
void display_INVscrollTextPerStep(String txt, int delay_nb_period);

//OLD A TRIER
void add_display_pmx_OLD();
void add_display_dist_OLD();
void display_leds_thread_old();


void display_panOrBounceBitmap(uint8_t bitmapSize);
void fixdrawRGBBitmap(int16_t x, int16_t y, const uint16_t *bitmap, int16_t w, int16_t h);
void display_rgbBitmap(uint8_t bmp_num, uint16_t bmx = 0, uint16_t bmy = 0, bool clearAndShow = 1);
void display_scrollRgbBitmap();
void display_four_white();
void display_text(String txt);

void display_resolution();
void display_CountInv(int fromNumber);
void display_INVscrollText(String txt, int delay_ms = 75);
void display_INVscrollTextWithBitmap(String txt, int bmp_num, int delay_ms = 75);
void display_INVscrollTextWithBitmapAndUA(String txt, int bitmap, int delay_ms = 75);
void display_scrollText(String txt);
void display_scrollText_old();
void display_dist();
void sprite_setup();
void flag();
void count_pixels_on_by_one();
void count_pixels();
void display_lines();
void display_boxes();
void display_circles();
void display_textes();



void write_PMX(int x, int y, uint32_t color);

uint16_t Color24toColor16(uint32_t color);
uint32_t Wheel(byte WheelPos);
uint8_t font_zoom(uint8_t zoom_type, uint8_t speed);
void font_loop();

#endif /* LEDPANELS_H_ */
