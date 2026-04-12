#include "Arduino.h"

#include "LCDScreen.h"

#include <ILI9341_T4.h>
#include <i2c_register_slave.h>

#include <lvgl.h> // see the comment above for infos about installing and configuring LVGL.

#include "TofSensors.h" // struct Settings (slave I2C 0x2D) - menu pre-match
/********************************************************************
 *
 * ILI9341_T4 library example. Interfacing with the LVGL library
 *
 *
 * Minimal example of using the ILI9341_T4 driver to drive a
 * screen + touchscreen for the LVGL library.
 *
 * *** THIS EXAMPLES REQUIRES THAT THE ILI9341 SCREEN HAS AN XPT4046 TOUCHSCREEN ***
 *
 * This example demonstrates how to use the ILI9341_T4 driver with the
 * LVGL library. Here, allocate an internal framebuffer of size 320x240
 * for the ILI9341_T4 driver but only a small 320x40 buffer for LVGL
 * to draw onto. Then we use the updateRegion() method from the library
 * to update the internal framebuffer and sync with the screen using
 * differential updates for maximum performance.
 *
 * The total memory consumption for the 'graphic part' is 191KB:
 *
 *   - 150KB for the internal framebuffer
 *   - 25KB for LVGL draw buffer
 *   - 16KB for 2 diffs buffer with 8Kb each.
 *
 -----------------------------------
 * BUILDING THE EXAMPLE (FOR ARDUINO)
 * -----------------------------------
 *
 * (1) Install the 'lvgl' libraries in Arduino's library folder.
 *     from the github repo: https://github.com/lvgl/lvgl/ directly
 *     into Arduino's library folder (tested here with LVGL v9.0.0).
 *
 * (2) Copy and rename the file 'libraries/lvgl/lv_conf_template.h' to
 *     'libraries/lv_conf.h' (i.e. put this file directly in Arduino's
 *     libraries root folder).
 *
 * (3) Edit the file 'lv_conf.h' such that:
 *
 *     -> Replace '#if 0' by '#if 1'               (at the begining of the file)
 *     -> set #define LV_COLOR_DEPTH 16            (should be already set to the correct value)
 *     -> set #define LV_TICK_CUSTOM 1
 *     -> set #define LV_USE_PERF_MONITOR 1        (if you want to to show the FPS counter)
 *     -> set #define LV_FONT_MONTSERRAT_12  1     (should be already set to the correct value)
 *     -> set #define LV_FONT_MONTSERRAT_14  1
 *     -> set #define LV_FONT_MONTSERRAT_16  1
 *
 ********************************************************************/

//initialise the settings
SettingsLCD settings_lcd = { 0x00, 0x00, 0x00, 0x00 };
//extern Registers registers;
RegistersLCD registers_lcd;
I2CRegisterSlave registerSlaveLCD = I2CRegisterSlave(Slave2,
		(uint8_t*) &settings_lcd, sizeof(SettingsLCD),
		(uint8_t*) &registers_lcd, sizeof(RegistersLCD));

// === Menu pre-match : acces aux registres Settings du slave 0x2D ===
// Le menu LCD ecrit directement dans `settings` (defini dans TofSensors.cpp)
// sans passer par un nouveau slave I2C. Voir ARCHITECTURE_BEACON.md
// section "Menu pre-match (LCD tactile)".
extern Settings settings;

// Flag indiquant si l'ecran ILI9341 a ete detecte au boot.
// Si false, screen_loop() et les widgets LVGL sont desactives,
// le reste du firmware (ToF, LED, I2C slave) fonctionne normalement.
static bool screen_available = false;

// Labels LVGL read-only mis a jour periodiquement dans screen_loop()
// avec les valeurs ecrites par l'OPOS6UL (reg 0 numOfBots, reg 2 matchPoints).
static lv_obj_t *lbl_numOfBots_value = nullptr;
static lv_obj_t *lbl_matchPoints_value = nullptr;

// 2 diff buffers with about 8K memory each
ILI9341_T4::DiffBuffStatic<8000> diff1;
ILI9341_T4::DiffBuffStatic<8000> diff2;

// the internal framebuffer for the ILI9341_T4 driver (150KB)
// in DMAMEM to save space in the lower (faster) part of RAM.
DMAMEM uint16_t internal_fb[LX * LY];

// the screen driver object
ILI9341_T4::ILI9341Driver tft(PIN_CS, PIN_DC, PIN_SCK, PIN_MOSI, PIN_MISO,
PIN_RESET, PIN_TOUCH_CS, PIN_TOUCH_IRQ);

lv_color_t lvgl_buf[LX * BUF_LY]; // memory for lvgl draw buffer (25KB)

lv_disp_draw_buf_t draw_buf;    // lvgl 'draw buffer' object
lv_disp_drv_t disp_drv;         // lvgl 'display driver'
lv_indev_drv_t indev_drv;       // lvgl 'input device driver'
lv_disp_t *disp;                // pointer to lvgl display object

/** Callback to draw on the screen */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area,
		lv_color_t *color_p) {
	const bool redraw_now = lv_disp_flush_is_last(disp); // check if when should update the screen (or just buffer the changes).
	tft.updateRegion(redraw_now, (uint16_t*) color_p, area->x1, area->x2,
			area->y1, area->y2); // update the interval framebuffer and then redraw the screen if requested
	lv_disp_flush_ready(disp); // tell lvgl that we are done and that the lvgl draw buffer can be reused immediately
}

/** Call back to read the touchpad */
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
	int touchX, touchY, touchZ;
	bool touched = tft.readTouch(touchX, touchY, touchZ); // read the touchpad
	if (!touched) { // nothing
		data->state = LV_INDEV_STATE_REL;
	} else { // pressed
		data->state = LV_INDEV_STATE_PR;
		data->point.x = touchX;
		data->point.y = touchY;
	}
}

/**
 * A simple grid
 */
void lv_example_grid_1(void) {
	static lv_coord_t col_dsc[] = { 70, 70, 70, LV_GRID_TEMPLATE_LAST };
	static lv_coord_t row_dsc[] = { 50, 50, 50, LV_GRID_TEMPLATE_LAST };

	/*Create a container with grid*/
	lv_obj_t *cont = lv_obj_create(lv_scr_act());
	lv_obj_set_style_grid_column_dsc_array(cont, col_dsc, 0);
	lv_obj_set_style_grid_row_dsc_array(cont, row_dsc, 0);
	lv_obj_set_size(cont, 300, 220);
	lv_obj_center(cont);
	lv_obj_set_layout(cont, LV_LAYOUT_GRID);

	lv_obj_t *label;
	lv_obj_t *obj;

	uint32_t i;
	for (i = 0; i < 9; i++) {
		uint8_t col = i % 3;
		uint8_t row = i / 3;

		obj = lv_btn_create(cont);
		/*Stretch the cell horizontally and vertically too
		 *Set span to 1 to make the cell 1 column/row sized*/
		lv_obj_set_grid_cell(obj, LV_GRID_ALIGN_STRETCH, col, 1,
				LV_GRID_ALIGN_STRETCH, row, 1);

		label = lv_label_create(obj);
		lv_label_set_text_fmt(label, "c%d, r%d", col, row);
		lv_obj_center(label);
	}
}

/**
 * Demonstrate cell placement and span
 */
void lv_example_grid_2(void) {
	static lv_coord_t col_dsc[] = { 70, 70, 70, LV_GRID_TEMPLATE_LAST };
	static lv_coord_t row_dsc[] = { 50, 50, 50, LV_GRID_TEMPLATE_LAST };

	/*Create a container with grid*/
	lv_obj_t *cont = lv_obj_create(lv_scr_act());
	lv_obj_set_grid_dsc_array(cont, col_dsc, row_dsc);
	lv_obj_set_size(cont, 300, 220);
	lv_obj_center(cont);

	lv_obj_t *label;
	lv_obj_t *obj;

	/*Cell to 0;0 and align to to the start (left/top) horizontally and vertically too*/
	obj = lv_obj_create(cont);
	lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
	lv_obj_set_grid_cell(obj, LV_GRID_ALIGN_START, 0, 1, LV_GRID_ALIGN_START, 0,
			1);
	label = lv_label_create(obj);
	lv_label_set_text(label, "c0, r0");

	/*Cell to 1;0 and align to to the start (left) horizontally and center vertically too*/
	obj = lv_obj_create(cont);
	lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
	lv_obj_set_grid_cell(obj, LV_GRID_ALIGN_START, 1, 1, LV_GRID_ALIGN_CENTER,
			0, 1);
	label = lv_label_create(obj);
	lv_label_set_text(label, "c1, r0");

	/*Cell to 2;0 and align to to the start (left) horizontally and end (bottom) vertically too*/
	obj = lv_obj_create(cont);
	lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
	lv_obj_set_grid_cell(obj, LV_GRID_ALIGN_START, 2, 1, LV_GRID_ALIGN_END, 0,
			1);
	label = lv_label_create(obj);
	lv_label_set_text(label, "c2, r0");

	/*Cell to 1;1 but 2 column wide (span = 2).Set width and height to stretched.*/
	obj = lv_obj_create(cont);
	lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
	lv_obj_set_grid_cell(obj, LV_GRID_ALIGN_STRETCH, 1, 2,
			LV_GRID_ALIGN_STRETCH, 1, 1);
	label = lv_label_create(obj);
	lv_label_set_text(label, "c1-2, r1");

	/*Cell to 0;1 but 2 rows tall (span = 2).Set width and height to stretched.*/
	obj = lv_obj_create(cont);
	lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
	lv_obj_set_grid_cell(obj, LV_GRID_ALIGN_STRETCH, 0, 1,
			LV_GRID_ALIGN_STRETCH, 1, 2);
	label = lv_label_create(obj);
	lv_label_set_text(label, "c0\nr1-2");
}

//void on_read_isr_lcd(uint8_t reg_num) {
//
//}

// ============================================================================
// Menu pre-match (LCD tactile) - Etape 3 de la migration
// ============================================================================
// Voir ARCHITECTURE_BEACON.md section "Menu pre-match (LCD tactile)".
// Les widgets ecrivent directement dans `settings` (slave I2C 0x2D), lu par
// l'OPOS6UL a sa cadence normale. Pas de mutex : un seul ecrivain par byte.
// ============================================================================

/**
 * Applique le style visuel du bouton couleur selon settings.matchColor.
 * 0 = fond bleu + texte "BLEU", 1 = fond jaune + texte "JAUNE".
 */
static void updateColorButton(lv_obj_t *btn) {
	lv_obj_t *label = lv_obj_get_child(btn, 0);
	if (settings.matchColor == 0) {
		lv_obj_set_style_bg_color(btn, lv_color_make(0, 80, 200), 0);
		lv_obj_set_style_text_color(label, lv_color_white(), 0);
		lv_label_set_text(label, "BLEU");
	} else {
		lv_obj_set_style_bg_color(btn, lv_color_make(240, 200, 0), 0);
		lv_obj_set_style_text_color(label, lv_color_black(), 0);
		lv_label_set_text(label, "JAUNE");
	}
}

/**
 * Callback : clic sur le bouton couleur -> toggle bleu/jaune.
 * Ecrit directement dans settings.matchColor (Reg 5, Bloc 2 LCD->OPOS6UL).
 */
static void matchColor_event_cb(lv_event_t *e) {
	lv_obj_t *btn = (lv_obj_t*) lv_event_get_target(e);
	settings.matchColor = (settings.matchColor == 0) ? 1 : 0;
	updateColorButton(btn);
}

// --- ledLuminosity : boutons [-] valeur [+] (pas de 10, 0..100) ---

static lv_obj_t *lbl_lum_value = nullptr;

static void ledLum_minus_cb(lv_event_t *e) {
	(void)e;
	if (settings.ledLuminosity <= 5)
		settings.ledLuminosity = (settings.ledLuminosity > 0) ? settings.ledLuminosity - 1 : 0;
	else
		settings.ledLuminosity -= 5;
	if (lbl_lum_value) lv_label_set_text_fmt(lbl_lum_value, "%d", settings.ledLuminosity);
}

static void ledLum_plus_cb(lv_event_t *e) {
	(void)e;
	if (settings.ledLuminosity < 5)
		settings.ledLuminosity += 1;
	else if (settings.ledLuminosity <= 95)
		settings.ledLuminosity += 5;
	else
		settings.ledLuminosity = 100;
	if (lbl_lum_value) lv_label_set_text_fmt(lbl_lum_value, "%d", settings.ledLuminosity);
}

// --- advDiameter : boutons [-5] valeur [+5] (cm, 5..250, defaut 40) ---

static lv_obj_t *lbl_adv_value = nullptr;

static void advDiam_minus_cb(lv_event_t *e) {
	(void)e;
	if (settings.advDiameter >= 10)
		settings.advDiameter -= 5;
	else
		settings.advDiameter = 5;
	if (lbl_adv_value) lv_label_set_text_fmt(lbl_adv_value, "%d cm", settings.advDiameter);
}

static void advDiam_plus_cb(lv_event_t *e) {
	(void)e;
	if (settings.advDiameter <= 245)
		settings.advDiameter += 5;
	else
		settings.advDiameter = 250;
	if (lbl_adv_value) lv_label_set_text_fmt(lbl_adv_value, "%d cm", settings.advDiameter);
}

// --- Strategy : 3 boutons radio (1/2/3) ---

#define STRATEGY_COUNT 3
static lv_obj_t *strategy_btns[STRATEGY_COUNT] = { nullptr };

static lv_color_t strategy_color_selected   = { .full = 0x07E0 }; // vert
static lv_color_t strategy_color_unselected = { .full = 0x4208 }; // gris fonce

static void updateStrategyButtons(void) {
	for (int i = 0; i < STRATEGY_COUNT; i++) {
		if (strategy_btns[i] == nullptr) continue;
		bool sel = (settings.strategy == (i + 1));
		lv_obj_set_style_bg_color(strategy_btns[i],
				sel ? strategy_color_selected : strategy_color_unselected, 0);
		lv_obj_t *label = lv_obj_get_child(strategy_btns[i], 0);
		lv_obj_set_style_text_color(label, sel ? lv_color_black() : lv_color_white(), 0);
	}
}

static void strategy_event_cb(lv_event_t *e) {
	lv_obj_t *btn = (lv_obj_t*) lv_event_get_target(e);
	for (int i = 0; i < STRATEGY_COUNT; i++) {
		if (strategy_btns[i] == btn) {
			settings.strategy = (uint8_t)(i + 1);
			break;
		}
	}
	updateStrategyButtons();
}

// --- TestMode : 5 boutons d'action (1..5), flash vert 1s puis retour gris ---
// Chaque clic ecrit la valeur dans settings.testMode (l'OPOS6UL la lit et
// lance l'action correspondante). Apres 1 seconde, le bouton repasse en gris
// et settings.testMode revient a 0.

#define TESTMODE_COUNT 5
static lv_obj_t *testMode_btns[TESTMODE_COUNT] = { nullptr };

static void testMode_reset_all_grey(void) {
	for (int i = 0; i < TESTMODE_COUNT; i++) {
		if (testMode_btns[i] == nullptr) continue;
		lv_obj_set_style_bg_color(testMode_btns[i], strategy_color_unselected, 0);
		lv_obj_t *lbl = lv_obj_get_child(testMode_btns[i], 0);
		lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
	}
}

static void testMode_timer_cb(lv_timer_t *timer) {
	settings.testMode = 0;
	testMode_reset_all_grey();
	lv_timer_del(timer); // timer one-shot, on le supprime
}

static void testMode_event_cb(lv_event_t *e) {
	lv_obj_t *btn = (lv_obj_t*) lv_event_get_target(e);
	for (int i = 0; i < TESTMODE_COUNT; i++) {
		if (testMode_btns[i] == btn) {
			settings.testMode = (uint8_t)(i + 1);
			// Flash vert sur le bouton clique, gris sur les autres
			testMode_reset_all_grey();
			lv_obj_set_style_bg_color(btn, strategy_color_selected, 0);
			lv_obj_t *lbl = lv_obj_get_child(btn, 0);
			lv_obj_set_style_text_color(lbl, lv_color_black(), 0);
			// Timer one-shot 1 seconde pour reset
			lv_timer_create(testMode_timer_cb, 1000, NULL);
			break;
		}
	}
}

/**
 * Cree le menu pre-match.
 *
 * Cette fonction remplace `lv_example_grid_1()` dans setup_screen().
 * La fonction `lv_example_grid_1()` est conservee plus haut dans le fichier
 * comme fallback / reference.
 */
static void create_match_menu(void) {
	lv_obj_t *scr = lv_scr_act();
	lv_obj_t *label;
	lv_obj_t *btn;

	// --- Ligne 0 (y=4) : LED [-] val [+]  +  Bots + Pts ---
	lv_obj_t *lbl_lum_title = lv_label_create(scr);
	lv_label_set_text(lbl_lum_title, "LED");
	lv_obj_set_pos(lbl_lum_title, 6, 8);

	btn = lv_btn_create(scr);
	lv_obj_set_size(btn, 36, 24);
	lv_obj_set_pos(btn, 32, 4);
	label = lv_label_create(btn);
	lv_label_set_text(label, "-");
	lv_obj_center(label);
	lv_obj_add_event_cb(btn, ledLum_minus_cb, LV_EVENT_CLICKED, NULL);

	lbl_lum_value = lv_label_create(scr);
	lv_label_set_text_fmt(lbl_lum_value, "%d", settings.ledLuminosity);
	lv_obj_set_pos(lbl_lum_value, 76, 8);

	btn = lv_btn_create(scr);
	lv_obj_set_size(btn, 36, 24);
	lv_obj_set_pos(btn, 100, 4);
	label = lv_label_create(btn);
	lv_label_set_text(label, "+");
	lv_obj_center(label);
	lv_obj_add_event_cb(btn, ledLum_plus_cb, LV_EVENT_CLICKED, NULL);

	lv_obj_t *lbl_bots_title = lv_label_create(scr);
	lv_label_set_text(lbl_bots_title, "Bots:");
	lv_obj_set_pos(lbl_bots_title, 200, 8);

	lbl_numOfBots_value = lv_label_create(scr);
	lv_label_set_text_fmt(lbl_numOfBots_value, "%d", settings.numOfBots);
	lv_obj_set_pos(lbl_numOfBots_value, 240, 8);

	lv_obj_t *lbl_pts_title = lv_label_create(scr);
	lv_label_set_text(lbl_pts_title, "Pts:");
	lv_obj_set_pos(lbl_pts_title, 262, 8);

	lbl_matchPoints_value = lv_label_create(scr);
	lv_label_set_text_fmt(lbl_matchPoints_value, "%d", settings.matchPoints);
	lv_obj_set_pos(lbl_matchPoints_value, 294, 8);

	// --- Ligne 1 (y=32) : testMode T1..T5 ---
	lv_obj_t *lbl_test = lv_label_create(scr);
	lv_label_set_text(lbl_test, "Test");
	lv_obj_align(lbl_test, LV_ALIGN_TOP_MID, 0, 32);

	int test_btn_w = 54;
	int test_btn_h = 28;
	int test_total = TESTMODE_COUNT * test_btn_w + (TESTMODE_COUNT - 1) * 6;
	int test_x0 = (320 - test_total) / 2;
	for (int i = 0; i < TESTMODE_COUNT; i++) {
		testMode_btns[i] = lv_btn_create(scr);
		lv_obj_set_size(testMode_btns[i], test_btn_w, test_btn_h);
		lv_obj_set_pos(testMode_btns[i], test_x0 + i * (test_btn_w + 6), 48);
		label = lv_label_create(testMode_btns[i]);
		lv_label_set_text_fmt(label, "T%d", i + 1);
		lv_obj_center(label);
		lv_obj_add_event_cb(testMode_btns[i], testMode_event_cb,
				LV_EVENT_CLICKED, NULL);
	}
	testMode_reset_all_grey();

	// --- Ligne 2 (y=82) : bouton toggle couleur (centre de l'ecran) ---
	lv_obj_t *btn_color = lv_btn_create(scr);
	lv_obj_set_size(btn_color, 240, 50);
	lv_obj_align(btn_color, LV_ALIGN_TOP_MID, 0, 82);
	label = lv_label_create(btn_color);
	lv_obj_center(label);
	updateColorButton(btn_color);
	lv_obj_add_event_cb(btn_color, matchColor_event_cb,
			LV_EVENT_CLICKED, NULL);

	// --- Ligne 3 (y=138) : strategie 1/2/3 ---
	lv_obj_t *lbl_strat = lv_label_create(scr);
	lv_label_set_text(lbl_strat, "Strategie");
	lv_obj_align(lbl_strat, LV_ALIGN_TOP_MID, 0, 138);

	int strat_btn_w = 90;
	int strat_btn_h = 36;
	int strat_total = STRATEGY_COUNT * strat_btn_w + (STRATEGY_COUNT - 1) * 8;
	int strat_x0 = (320 - strat_total) / 2;
	for (int i = 0; i < STRATEGY_COUNT; i++) {
		strategy_btns[i] = lv_btn_create(scr);
		lv_obj_set_size(strategy_btns[i], strat_btn_w, strat_btn_h);
		lv_obj_set_pos(strategy_btns[i], strat_x0 + i * (strat_btn_w + 8), 154);
		label = lv_label_create(strategy_btns[i]);
		lv_label_set_text_fmt(label, "%d", i + 1);
		lv_obj_center(label);
		lv_obj_add_event_cb(strategy_btns[i], strategy_event_cb,
				LV_EVENT_CLICKED, NULL);
	}
	updateStrategyButtons();

	// --- Ligne 4 (y=196) : diametre adversaire [-5] val [+5] gros boutons ---
	lv_obj_t *lbl_adv_title = lv_label_create(scr);
	lv_label_set_text(lbl_adv_title, "Adv:");
	lv_obj_set_pos(lbl_adv_title, 10, 206);

	btn = lv_btn_create(scr);
	lv_obj_set_size(btn, 70, 38);
	lv_obj_set_pos(btn, 50, 198);
	label = lv_label_create(btn);
	lv_label_set_text(label, "- 5");
	lv_obj_center(label);
	lv_obj_add_event_cb(btn, advDiam_minus_cb, LV_EVENT_CLICKED, NULL);

	lbl_adv_value = lv_label_create(scr);
	lv_label_set_text_fmt(lbl_adv_value, "%d cm", settings.advDiameter);
	lv_obj_set_pos(lbl_adv_value, 134, 206);

	btn = lv_btn_create(scr);
	lv_obj_set_size(btn, 70, 38);
	lv_obj_set_pos(btn, 200, 198);
	label = lv_label_create(btn);
	lv_label_set_text(label, "+ 5");
	lv_obj_center(label);
	lv_obj_add_event_cb(btn, advDiam_plus_cb, LV_EVENT_CLICKED, NULL);
}

void setup_screen() {

	// Start listening on I2C4 with address 0x2F
//	registerSlaveLCD.listen(0x2F);
//	registerSlaveLCD.after_read(on_read_isr_lcd);

	// ------------------------------
	// Init the ILI9341_T4 driver.
	// ------------------------------
	tft.output(&Serial);                // send debug info to serial port.
	if (!tft.begin(SPI_SPEED)) {        // init (retente 5x dans le driver)
		Serial.println("WARNING: ILI9341 screen not detected, LCD disabled.");
		screen_available = false;
		return; // pas d'ecran : on skip toute l'init LVGL/touch
	}
	screen_available = true;

	tft.setFramebuffer(internal_fb);    // set the internal framebuffer
	tft.setDiffBuffers(&diff1, &diff2); // set the diff buffers
	tft.setRotation(3);                 // portrait mode 3 : 320x240
	tft.setDiffGap(4); // with have large 8K diff buffers so we can use a small gap.
	tft.setVSyncSpacing(1); // lvgl is already controlling framerate: we just set this to 1 to minimize screen tearing.
	tft.setRefreshRate(60);            // 100Hz refresh, why not...

	//touchscreen XPT2046 on the same SPI bus
	//tft.calibrateTouch();                            // run calibration...

	int touch_calib[4] = { 398, 3720, 3892, 478 }; // ...or directly load calibration data
	tft.setTouchCalibration(touch_calib);  // if the values are already known...

	// ------------------------------
	// Init LVGL
	// ------------------------------
	lv_init();

	// initialize lvgl drawing buffer
	lv_disp_draw_buf_init(&draw_buf, lvgl_buf, nullptr, LX * BUF_LY);

	// Initialize lvgl display driver
	lv_disp_drv_init(&disp_drv);
	disp_drv.hor_res = LX;
	disp_drv.ver_res = LY;
	disp_drv.flush_cb = my_disp_flush;
	disp_drv.draw_buf = &draw_buf;
	disp = lv_disp_drv_register(&disp_drv);
	disp->refr_timer->period = 15; // set refresh rate around 66FPS.

	// Initialize lvgl input device driver (the touch screen)
	lv_indev_drv_init(&indev_drv);
	indev_drv.type = LV_INDEV_TYPE_POINTER;
	indev_drv.read_cb = my_touchpad_read;
	lv_indev_drv_register(&indev_drv);

	// ------------------------------
	// SHORT EXAMPLE : display a keyboard
	// c.f. https://docs.lvgl.io/master/examples.html#keyboard
	// ------------------------------
//	lv_obj_t *kb = lv_keyboard_create(lv_scr_act());
//	lv_obj_t *ta = lv_textarea_create(lv_scr_act());
//	lv_obj_align(ta, LV_ALIGN_TOP_LEFT, 10, 10);
//	lv_obj_add_event_cb(ta, ta_event_cb, LV_EVENT_ALL, kb);
//	lv_textarea_set_placeholder_text(ta, "Hello");
//	lv_obj_set_size(ta, 220, 140);

	// ------------------------------
	// Menu pre-match (etape 3 migration)
	// ------------------------------
	// Cree le menu tactile avec 3 widgets : couleur, n° match, labels read-only.
	// Pour revenir a l'exemple de demonstration, remplacer create_match_menu()
	// par lv_example_grid_1() (ou lv_example_grid_2()) - les fonctions sont
	// conservees plus haut dans ce fichier.
	create_match_menu();
}

// event callback
void ta_event_cb(lv_event_t *e) {
	lv_event_code_t code = lv_event_get_code(e);
	lv_obj_t *ta = (lv_obj_t*) lv_event_get_target(e);
	lv_obj_t *kb = (lv_obj_t*) lv_event_get_user_data(e);
	if (code == LV_EVENT_FOCUSED) {
		lv_keyboard_set_textarea(kb, ta);
		lv_obj_clear_flag(kb, LV_OBJ_FLAG_HIDDEN);
	}
	if (code == LV_EVENT_DEFOCUSED) {
		lv_keyboard_set_textarea(kb, NULL);
		lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
	}
}

void screen_loop() {
	if (!screen_available) return; // pas d'ecran : no-op

	// Rafraichissement des labels read-only (valeurs ecrites par OPOS6UL via I2C).
	// 5 Hz suffit largement pour l'affichage, evite le cout d'un set_text a chaque frame.
	static uint32_t last_refresh_ms = 0;
	uint32_t now = millis();
	if (now - last_refresh_ms > 200) {
		last_refresh_ms = now;
		if (lbl_numOfBots_value != nullptr) {
			lv_label_set_text_fmt(lbl_numOfBots_value, "%d", settings.numOfBots);
		}
		if (lbl_matchPoints_value != nullptr) {
			lv_label_set_text_fmt(lbl_matchPoints_value, "%d", settings.matchPoints);
		}
	}
	lv_task_handler(); // lvgl gui handler
}

