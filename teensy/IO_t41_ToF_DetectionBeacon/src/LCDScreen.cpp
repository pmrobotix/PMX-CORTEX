#include "Arduino.h"

#include "LCDScreen.h"

#include <ILI9341_T4.h>
#include <i2c_register_slave.h>

#include <lvgl.h> // see the comment above for infos about installing and configuring LVGL.

#include "logo_pmx.h"
#include "qr_code_pmx.h"
#include "favicon_pmx.h"
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
// Handle file-scope vers le bouton couleur (assigne dans setup_screen()).
// Permet a screen_loop() de rafraichir l'affichage quand settings.matchColor
// est modifie par l'OPOS6UL via I2C (pas uniquement par un clic tactile).
static lv_obj_t *btn_color_handle = nullptr;

// Handle du bouton ZONES (acces a l'ecran config zones de prise).
// Null apres invalidate_menu_handles() ou quand on n'est pas sur le menu.
static lv_obj_t *btn_zones_handle = nullptr;

// Handles de l'ecran config zones de prise.
static lv_obj_t *pickup_back_btn_handle = nullptr;
static lv_obj_t *pickup_zone_lbls[8] = { nullptr };

// Reperes visuels "nid bleu" (bas-gauche) et "nid jaune" (bas-droite) :
// carres dessines avec 4 lignes en pointille.
// Les points sont conserves en static pour que lv_line_set_points() y reste attache.
static lv_point_t pickup_bluesq_pts[4][2];
static lv_point_t pickup_yelsq_pts[4][2];
static lv_obj_t *pickup_bluesq_lines[4] = { nullptr };
static lv_obj_t *pickup_yelsq_lines[4]  = { nullptr };

// Cycle canonique des 6 combinaisons : index -> pattern 4 caracteres.
// Ordre figé dans MATCH_CONFIG_UI.md section "Ordre canonique".
static const char *PICKUP_CYCLE[6] = {
	"BBYY", "YYBB", "BYYB", "YBBY", "BYBY", "YBYB"
};

// Meta-donnees d'une zone (fixe) + handles runtime vers les rectangles couleur.
struct PickupZone {
	const char *name;       // "P1", "P2", ...
	bool horizontal;        // true pour P3/P4/P13/P14, false pour P1/P2/P11/P12
	int x, y;               // position top-left du widget sur l'ecran
	uint8_t *value_ptr;     // pointeur vers settings.pickup_Pn
	lv_obj_t *rects[4];     // handles runtime des 4 rectangles colores
};

// Tableau des 8 zones. Toutes les zones ont la meme bounding box 72x74 :
// label 14 + combi 72x60. Grille 4 cols x 2 rows (vue operateur 180°).
//
//   cols (gap 8) : V   H   H   V
//   rows (gap 11): top P3/P13 (H), P2/P12 (V decale +10 pour matcher la table)
//                  bot P4/P14 (H), P1/P11 (V decale +10)
//
// x cols : 4, 84, 164, 244  (widget 72 + gap 8)
// y H    : 0 (row 1), 85 (row 2)
// y V    : 10 (row 1), 95 (row 2)  -- decales +10 vs H (disposition terrain)
//
// value_ptr est cable dans create_pickup_config() une fois `settings` reference.
static PickupZone pickup_zones[8] = {
	// name, horizontal,  x,   y,   value_ptr, rects
	{ "P2",  false,    4,  10, nullptr, { nullptr, nullptr, nullptr, nullptr } },
	{ "P3",  true,    84,   0, nullptr, { nullptr, nullptr, nullptr, nullptr } },
	{ "P13", true,   164,   0, nullptr, { nullptr, nullptr, nullptr, nullptr } },
	{ "P12", false, 244,  10, nullptr, { nullptr, nullptr, nullptr, nullptr } },
	{ "P1",  false,    4,  95, nullptr, { nullptr, nullptr, nullptr, nullptr } },
	{ "P4",  true,    84,  85, nullptr, { nullptr, nullptr, nullptr, nullptr } },
	{ "P14", true,   164,  85, nullptr, { nullptr, nullptr, nullptr, nullptr } },
	{ "P11", false, 244,  95, nullptr, { nullptr, nullptr, nullptr, nullptr } },
};

// Etat courant de l'ecran LCD : 0=menu, 1=match (logo), 2=fin (favicon+score),
// 3=config zones de prise. Declare ici pour etre accessible depuis les callbacks
// de navigation.
static uint8_t lcd_screen_state = 0;

// Forward declarations pour la navigation menu <-> pickup config.
static void create_match_menu(void);
static void create_pickup_config(void);
static void invalidate_pickup_handles(void);
static void invalidate_menu_handles(void);
static void updateZonesButtonLock(void);
static void zones_event_cb(lv_event_t *e);
static void back_to_menu_cb(lv_event_t *e);

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
	// Ignore le clic si matchState >= 1 (couleur lockee en ARMED/PRIMED/MATCH).
	// La protection visuelle (LV_STATE_DISABLED) est appliquee par screen_loop,
	// mais ce guard couvre le cas ou matchState est stale d'un run precedent
	// avant que l'OPOS6UL ait pu pousser matchState=0.
	if (settings.matchState >= 1) return;
	lv_obj_t *btn = (lv_obj_t*) lv_event_get_target(e);
	settings.matchColor = (settings.matchColor == 0) ? 1 : 0;
	settings.seq_touch++;
	updateColorButton(btn);
}

// --- ledLuminosity : boutons [-] valeur [+] (pas de 10, 0..100) ---

static lv_obj_t *lbl_lum_value = nullptr;

static void ledLum_minus_cb(lv_event_t *e) {
	(void)e;
	if (settings.ledLuminosity <= 10)
		settings.ledLuminosity = (settings.ledLuminosity > 0) ? settings.ledLuminosity - 1 : 0;
	else
		settings.ledLuminosity -= 5;
	settings.seq_touch++;
	if (lbl_lum_value) lv_label_set_text_fmt(lbl_lum_value, "%d", settings.ledLuminosity);
}

static void ledLum_plus_cb(lv_event_t *e) {
	(void)e;
	if (settings.ledLuminosity < 10)
		settings.ledLuminosity += 1;
	else if (settings.ledLuminosity <= 95)
		settings.ledLuminosity += 5;
	else
		settings.ledLuminosity = 100;
	settings.seq_touch++;
	if (lbl_lum_value) lv_label_set_text_fmt(lbl_lum_value, "%d", settings.ledLuminosity);
}

static void ledLum_set10_cb(lv_event_t *e) {
	(void)e;
	settings.ledLuminosity = 10;
	settings.seq_touch++;
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
	settings.seq_touch++;
	if (lbl_adv_value) lv_label_set_text_fmt(lbl_adv_value, "%d cm", settings.advDiameter);
}

static void advDiam_plus_cb(lv_event_t *e) {
	(void)e;
	if (settings.advDiameter <= 245)
		settings.advDiameter += 5;
	else
		settings.advDiameter = 250;
	settings.seq_touch++;
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
			settings.seq_touch++;
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
	settings.seq_touch++;
	testMode_reset_all_grey();
	lv_timer_del(timer);
}

// --- Bouton SETPOS / RESET (partage toute la largeur de l'ecran avec btn_color) ---
// matchState=0 (CONFIG) : libelle "SETPOS", vert, clic -> actionReq=1 (setPos)
// matchState=1 (ARMED)  : libelle "METTRE TIR", rouge, clic -> actionReq=1 (reset)
// matchState=2 (PRIMED) : libelle "ENLEVE TIR", rouge, clic -> actionReq=1 (reset)
// matchState>=3 (MATCH+): libelle "MATCH", gris (clic ignore)
static lv_obj_t *btn_setpos_handle = nullptr;
static lv_obj_t *lbl_setpos_handle = nullptr;

static lv_color_t setpos_color_green = { .full = 0x07E0 };
static lv_color_t setpos_color_red   = { .full = 0xF800 };
static lv_color_t setpos_color_grey  = { .full = 0x4208 };

static void updateSetposButton() {
	if (!btn_setpos_handle || !lbl_setpos_handle) return;
	if (settings.matchState == 0) {          // CONFIG
		lv_obj_set_style_bg_color(btn_setpos_handle, setpos_color_green, 0);
		lv_label_set_text(lbl_setpos_handle, "SETPOS");
		lv_obj_set_style_text_color(lbl_setpos_handle, lv_color_black(), 0);
	} else if (settings.matchState == 1) {   // ARMED -> attente insertion tirette
		lv_obj_set_style_bg_color(btn_setpos_handle, setpos_color_red, 0);
		lv_label_set_text(lbl_setpos_handle, "METTRE TIR");
		lv_obj_set_style_text_color(lbl_setpos_handle, lv_color_white(), 0);
	} else if (settings.matchState == 2) {   // PRIMED -> attente retrait tirette
		lv_obj_set_style_bg_color(btn_setpos_handle, setpos_color_red, 0);
		lv_label_set_text(lbl_setpos_handle, "ENLEVE TIR");
		lv_obj_set_style_text_color(lbl_setpos_handle, lv_color_white(), 0);
	} else {                                  // MATCH / END
		lv_obj_set_style_bg_color(btn_setpos_handle, setpos_color_grey, 0);
		lv_label_set_text(lbl_setpos_handle, "MATCH");
		lv_obj_set_style_text_color(lbl_setpos_handle, lv_color_white(), 0);
	}
}

static uint32_t setpos_press_ms = 0;   ///< Timestamp du dernier appui sur SETPOS.
static uint32_t actionReq_set_ms = 0;  ///< Timestamp quand actionReq a ete mis a 1.

static void setpos_pressed_cb(lv_event_t *e) {
	(void)e;
	setpos_press_ms = millis();
	Serial.printf("[SETPOS] PRESSED at %lu ms\n", setpos_press_ms);
}

static void setpos_event_cb(lv_event_t *e) {
	(void)e;
	uint32_t elapsed = millis() - setpos_press_ms;
	if (settings.matchState >= 3) {
		if (lbl_setpos_handle) lv_label_set_text(lbl_setpos_handle, "MATCH!");
		return;
	}
	if (settings.actionReq != 0) {
		if (lbl_setpos_handle) lv_label_set_text(lbl_setpos_handle, "WAIT..");
		return;
	}
	if (elapsed < 250) {
		if (lbl_setpos_handle) {
			char dbg[16];
			snprintf(dbg, sizeof(dbg), "%lums", elapsed);
			lv_label_set_text(lbl_setpos_handle, dbg);
		}
		return;
	}
	settings.actionReq = 1;
	settings.seq_touch++;
	actionReq_set_ms = millis();
	if (lbl_setpos_handle) lv_label_set_text(lbl_setpos_handle, "OK!");
}

// --- Visibilite du bouton couleur selon la phase (lock en ARMED+) ---
static void updateColorButtonLock() {
	if (!btn_color_handle) return;
	if (settings.matchState >= 1) {
		// lock : assombrir + indication visuelle
		lv_obj_add_state(btn_color_handle, LV_STATE_DISABLED);
	} else {
		lv_obj_clear_state(btn_color_handle, LV_STATE_DISABLED);
	}
}

// --- Visibilite du bouton ZONES selon la phase (lock en ARMED+) ---
static void updateZonesButtonLock(void) {
	if (!btn_zones_handle) return;
	if (settings.matchState >= 1) {
		lv_obj_add_state(btn_zones_handle, LV_STATE_DISABLED);
	} else {
		lv_obj_clear_state(btn_zones_handle, LV_STATE_DISABLED);
	}
}

// --- Navigation menu <-> pickup config ---

static void zones_event_cb(lv_event_t *e) {
	(void)e;
	// Bouton desactive apres setPos (matchState>=1) : on ignore tout clic.
	if (settings.matchState >= 1) return;
	lv_obj_clean(lv_scr_act());
	invalidate_menu_handles();
	create_pickup_config();
	lcd_screen_state = 3;
}

static void back_to_menu_cb(lv_event_t *e) {
	(void)e;
	lv_obj_clean(lv_scr_act());
	invalidate_pickup_handles();
	create_match_menu();
	lcd_screen_state = 0;
}

static void testMode_event_cb(lv_event_t *e) {
	lv_obj_t *btn = (lv_obj_t*) lv_event_get_target(e);
	for (int i = 0; i < TESTMODE_COUNT; i++) {
		if (testMode_btns[i] == btn) {
					settings.testMode = (uint8_t)(i + 1);
			settings.seq_touch++;
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
	// Reset fond ecran a blanc : show_match_screen/show_endmatch_screen ont pu
	// le passer en violet (30,27,59) lors d'un run precedent. lv_obj_clean()
	// supprime les widgets mais conserve les styles de l'ecran lui-meme.
	lv_obj_set_style_bg_color(scr, lv_color_white(), 0);
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

	// Bouton "10" pour set rapide luminosite a 10
	btn = lv_btn_create(scr);
	lv_obj_set_size(btn, 42, 24);
	lv_obj_set_pos(btn, 142, 4);
	label = lv_label_create(btn);
	lv_label_set_text(label, "10");
	lv_obj_center(label);
	lv_obj_add_event_cb(btn, ledLum_set10_cb, LV_EVENT_CLICKED, NULL);

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

	// --- Labels (y=36) : Couleur / Zones / Start/Reset ---
	lv_obj_t *lbl_color_title = lv_label_create(scr);
	lv_label_set_text(lbl_color_title, "Couleur");
	lv_obj_set_pos(lbl_color_title, 28, 36);

	lv_obj_t *lbl_zones_title = lv_label_create(scr);
	lv_label_set_text(lbl_zones_title, "Zones");
	lv_obj_set_pos(lbl_zones_title, 142, 36);

	lv_obj_t *lbl_setpos_title = lv_label_create(scr);
	lv_label_set_text(lbl_setpos_title, "Start / Reset");
	lv_obj_set_pos(lbl_setpos_title, 230, 36);

	// --- Ligne 1 (y=52) : COULEUR + ZONES + SETPOS/RESET, 100x76 chacun ---
	lv_obj_t *btn_color = lv_btn_create(scr);
	btn_color_handle = btn_color;
	lv_obj_set_size(btn_color, 100, 76);
	lv_obj_set_pos(btn_color, 4, 52);
	label = lv_label_create(btn_color);
	lv_obj_center(label);
	updateColorButton(btn_color);
	lv_obj_add_event_cb(btn_color, matchColor_event_cb,
			LV_EVENT_CLICKED, NULL);

	// Bouton ZONES : acces a l'ecran config zones de prise (pre-match).
	// Desactive apres setPos (matchState>=1), comme les autres parametres.
	lv_obj_t *btn_zones = lv_btn_create(scr);
	btn_zones_handle = btn_zones;
	lv_obj_set_size(btn_zones, 100, 76);
	lv_obj_set_pos(btn_zones, 110, 52);
	lv_obj_set_style_bg_color(btn_zones, lv_color_black(), 0);
	label = lv_label_create(btn_zones);
	lv_label_set_text(label, "ZONES");
	lv_obj_set_style_text_color(label, lv_color_white(), 0);
	lv_obj_center(label);
	lv_obj_add_event_cb(btn_zones, zones_event_cb, LV_EVENT_CLICKED, NULL);
	updateZonesButtonLock();

	btn_setpos_handle = lv_btn_create(scr);
	lv_obj_set_size(btn_setpos_handle, 100, 76);
	lv_obj_set_pos(btn_setpos_handle, 216, 52);
	lbl_setpos_handle = lv_label_create(btn_setpos_handle);
	lv_obj_center(lbl_setpos_handle);
	updateSetposButton();
	lv_obj_add_event_cb(btn_setpos_handle, setpos_pressed_cb,
			LV_EVENT_PRESSED, NULL);
	lv_obj_add_event_cb(btn_setpos_handle, setpos_event_cb,
			LV_EVENT_CLICKED, NULL);

	// --- Label + Ligne 2 (y=132/148) : strategie 1/2/3 ---
	lv_obj_t *lbl_strat = lv_label_create(scr);
	lv_label_set_text(lbl_strat, "Strategie");
	lv_obj_align(lbl_strat, LV_ALIGN_TOP_MID, 0, 132);

	int strat_btn_w = 90;
	int strat_btn_h = 32;
	int strat_total = STRATEGY_COUNT * strat_btn_w + (STRATEGY_COUNT - 1) * 8;
	int strat_x0 = (320 - strat_total) / 2;
	for (int i = 0; i < STRATEGY_COUNT; i++) {
		strategy_btns[i] = lv_btn_create(scr);
		lv_obj_set_size(strategy_btns[i], strat_btn_w, strat_btn_h);
		lv_obj_set_pos(strategy_btns[i], strat_x0 + i * (strat_btn_w + 8), 148);
		label = lv_label_create(strategy_btns[i]);
		lv_label_set_text_fmt(label, "%d", i + 1);
		lv_obj_center(label);
		lv_obj_add_event_cb(strategy_btns[i], strategy_event_cb,
				LV_EVENT_CLICKED, NULL);
	}
	updateStrategyButtons();

	// --- Ligne 3 (y=186) : T1 T2 + Diam [+ 5] ---
	int test_btn_w = 68;
	int test_btn_h = 24;
	for (int i = 0; i < 2 && i < TESTMODE_COUNT; i++) {
		testMode_btns[i] = lv_btn_create(scr);
		lv_obj_set_size(testMode_btns[i], test_btn_w, test_btn_h);
		lv_obj_set_pos(testMode_btns[i], 4 + i * (test_btn_w + 6), 186);
		label = lv_label_create(testMode_btns[i]);
		lv_label_set_text_fmt(label, "T%d", i + 1);
		lv_obj_center(label);
		lv_obj_add_event_cb(testMode_btns[i], testMode_event_cb,
				LV_EVENT_CLICKED, NULL);
	}

	lv_obj_t *lbl_adv_title = lv_label_create(scr);
	lv_label_set_text(lbl_adv_title, "Diam");
	lv_obj_set_pos(lbl_adv_title, 160, 190);

	btn = lv_btn_create(scr);
	lv_obj_set_size(btn, 80, 24);
	lv_obj_set_pos(btn, 210, 186);
	label = lv_label_create(btn);
	lv_label_set_text(label, "+ 5");
	lv_obj_center(label);
	lv_obj_add_event_cb(btn, advDiam_plus_cb, LV_EVENT_CLICKED, NULL);

	// --- Ligne 4 (y=214) : T3 T4 + 40cm [- 5] ---
	for (int i = 2; i < 4 && i < TESTMODE_COUNT; i++) {
		testMode_btns[i] = lv_btn_create(scr);
		lv_obj_set_size(testMode_btns[i], test_btn_w, test_btn_h);
		lv_obj_set_pos(testMode_btns[i], 4 + (i - 2) * (test_btn_w + 6), 214);
		label = lv_label_create(testMode_btns[i]);
		lv_label_set_text_fmt(label, "T%d", i + 1);
		lv_obj_center(label);
		lv_obj_add_event_cb(testMode_btns[i], testMode_event_cb,
				LV_EVENT_CLICKED, NULL);
	}
	// T5 non cree (supprime du layout)
	testMode_reset_all_grey();

	lbl_adv_value = lv_label_create(scr);
	lv_label_set_text_fmt(lbl_adv_value, "%d cm", settings.advDiameter);
	lv_obj_set_pos(lbl_adv_value, 160, 218);

	btn = lv_btn_create(scr);
	lv_obj_set_size(btn, 80, 24);
	lv_obj_set_pos(btn, 210, 214);
	label = lv_label_create(btn);
	lv_label_set_text(label, "- 5");
	lv_obj_center(label);
	lv_obj_add_event_cb(btn, advDiam_minus_cb, LV_EVENT_CLICKED, NULL);
}

void setup_screen_splash() {

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
	// Splash screen : logo PM-ROBOTIX
	// ------------------------------
	// Le logo reste affiche pendant que tof_setup() configure les VL53.
	// setup_screen_menu() le supprimera ensuite.
	lv_obj_t *splash_img = lv_img_create(lv_scr_act());
	lv_img_set_src(splash_img, &qr_code_pmx);
	lv_obj_set_pos(splash_img, 0, 0);

	// Pomper LVGL pour envoyer le logo a l'ecran
	lv_task_handler();
}

void setup_screen_menu() {
	if (!screen_available) return;

	// Nettoyer le splash (supprimer tous les enfants de l'ecran)
	lv_obj_clean(lv_scr_act());

	// ------------------------------
	// Menu pre-match
	// ------------------------------
	// Cree le menu tactile avec les widgets : couleur, strategie, labels read-only.
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

// Handle du label score en match (haut droite) et en fin (gros centre)
static lv_obj_t *lbl_match_score = nullptr;

/**
 * Invalide les handles des widgets menu (apres lv_obj_clean).
 */
static void invalidate_menu_handles(void) {
	btn_color_handle = nullptr;
	btn_zones_handle = nullptr;
	btn_setpos_handle = nullptr;
	lbl_setpos_handle = nullptr;
	lbl_numOfBots_value = nullptr;
	lbl_matchPoints_value = nullptr;
	lbl_lum_value = nullptr;
	lbl_adv_value = nullptr;
	for (int i = 0; i < STRATEGY_COUNT; i++) strategy_btns[i] = nullptr;
	for (int i = 0; i < TESTMODE_COUNT; i++) testMode_btns[i] = nullptr;
}

/**
 * Invalide les handles des widgets de l'ecran pickup config (apres lv_obj_clean).
 */
static void invalidate_pickup_handles(void) {
	pickup_back_btn_handle = nullptr;
	for (int i = 0; i < 8; i++) pickup_zone_lbls[i] = nullptr;
	for (int z = 0; z < 8; z++) {
		for (int r = 0; r < 4; r++) pickup_zones[z].rects[r] = nullptr;
	}
	for (int i = 0; i < 4; i++) {
		pickup_bluesq_lines[i] = nullptr;
		pickup_yelsq_lines[i]  = nullptr;
	}
}

/**
 * Dessine un carre pointille de taille size a la position (x, y) avec la
 * couleur donnee, en creant 4 lv_line (haut, bas, gauche, droite) avec style
 * line_dash. Les arrays `pts` et `lines` doivent avoir une duree de vie au
 * moins egale a celle de l'ecran (typiquement static module-level).
 */
static void create_dashed_square(lv_obj_t *scr, lv_point_t pts[4][2],
		lv_obj_t *lines[4], int x, int y, int size, lv_color_t color) {
	int x1 = x, y1 = y, x2 = x + size, y2 = y + size;
	// Haut
	pts[0][0].x = x1; pts[0][0].y = y1;
	pts[0][1].x = x2; pts[0][1].y = y1;
	// Bas
	pts[1][0].x = x1; pts[1][0].y = y2;
	pts[1][1].x = x2; pts[1][1].y = y2;
	// Gauche
	pts[2][0].x = x1; pts[2][0].y = y1;
	pts[2][1].x = x1; pts[2][1].y = y2;
	// Droite
	pts[3][0].x = x2; pts[3][0].y = y1;
	pts[3][1].x = x2; pts[3][1].y = y2;
	for (int i = 0; i < 4; i++) {
		lines[i] = lv_line_create(scr);
		lv_line_set_points(lines[i], pts[i], 2);
		lv_obj_set_style_line_color(lines[i], color, 0);
		lv_obj_set_style_line_width(lines[i], 2, 0);
		lv_obj_set_style_line_dash_width(lines[i], 4, 0);
		lv_obj_set_style_line_dash_gap(lines[i], 3, 0);
	}
}

// Couleurs des cases jaune/bleu (identiques au bouton couleur du menu).
static const lv_color_t PICKUP_YELLOW = LV_COLOR_MAKE(240, 200, 0);
static const lv_color_t PICKUP_BLUE   = LV_COLOR_MAKE(0, 80, 200);

/**
 * Met a jour la couleur et la lettre des 4 rectangles d'une zone selon
 * la valeur courante pointee par z.value_ptr (index 0..5 dans PICKUP_CYCLE).
 */
static void update_zone_rects(PickupZone &z) {
	if (!z.value_ptr) return;
	uint8_t idx = *z.value_ptr;
	if (idx >= 6) idx = 0;   // garde-fou: valeur invalide -> BBYY
	const char *pattern = PICKUP_CYCLE[idx];
	for (int i = 0; i < 4; i++) {
		if (!z.rects[i]) continue;
		bool is_yellow = (pattern[i] == 'Y');
		lv_obj_set_style_bg_color(z.rects[i],
				is_yellow ? PICKUP_YELLOW : PICKUP_BLUE, 0);
		// La lettre Y/B est stockee dans le premier enfant label.
		lv_obj_t *lbl = lv_obj_get_child(z.rects[i], 0);
		if (lbl) {
			char buf[2] = { pattern[i], '\0' };
			lv_label_set_text(lbl, buf);
			lv_obj_set_style_text_color(lbl,
					is_yellow ? lv_color_black() : lv_color_white(), 0);
		}
	}
}

// Callbacks de cyclage. On recupere l'index de zone via user_data.
static void zone_prev_cb(lv_event_t *e) {
	int zi = (int)(intptr_t)lv_event_get_user_data(e);
	if (zi < 0 || zi >= 8) return;
	PickupZone &z = pickup_zones[zi];
	if (!z.value_ptr) return;
	*z.value_ptr = (*z.value_ptr + 5) % 6;  // cycle inverse
	settings.seq_touch++;
	update_zone_rects(z);
}

static void zone_next_cb(lv_event_t *e) {
	int zi = (int)(intptr_t)lv_event_get_user_data(e);
	if (zi < 0 || zi >= 8) return;
	PickupZone &z = pickup_zones[zi];
	if (!z.value_ptr) return;
	*z.value_ptr = (*z.value_ptr + 1) % 6;
	settings.seq_touch++;
	update_zone_rects(z);
}

/**
 * Cree le widget d'une zone : label "Pn" + 4 rectangles colores (pas de fleches).
 *
 * Taille uniforme pour toutes les zones (H et V) : 72 x 74.
 *   - label 14 de haut centre au-dessus de la combi
 *   - combi 72 x 60 : 4 rects 18 x 60 cote a cote (H) ou 72 x 15 empiles (V)
 *
 * Interaction :
 *   - tap court sur un rect = combinaison suivante (cycle +1)
 *   - appui long sur un rect = combinaison precedente (cycle -1)
 * Les 4 rects d'une zone sont tous cliquables et pointent vers la meme zone,
 * pas besoin de viser un endroit precis.
 */
static void create_pickup_zone_widget(lv_obj_t *scr, int zi) {
	PickupZone &z = pickup_zones[zi];

	const int WIDGET_W = 72;
	const int LABEL_H  = 14;
	const int COMBI_W  = 72;
	const int COMBI_H  = 60;

	// Label "Pn" centre au-dessus du widget.
	pickup_zone_lbls[zi] = lv_label_create(scr);
	lv_label_set_text(pickup_zone_lbls[zi], z.name);
	// Approximation simple : positionner a ~10 px a gauche du centre du widget.
	lv_obj_set_pos(pickup_zone_lbls[zi], z.x + (WIDGET_W / 2) - 10, z.y);

	// 4 rectangles couleur. H : 4 cote a cote (20x60 chacun). V : 4 empiles (80x15).
	const int rect_w = z.horizontal ? (COMBI_W / 4) : COMBI_W;
	const int rect_h = z.horizontal ? COMBI_H : (COMBI_H / 4);
	const int combi_y = z.y + LABEL_H;

	for (int i = 0; i < 4; i++) {
		lv_obj_t *rect = lv_obj_create(scr);
		lv_obj_set_size(rect, rect_w, rect_h);
		lv_obj_set_style_pad_all(rect, 0, 0);
		lv_obj_set_style_border_width(rect, 1, 0);
		lv_obj_set_style_radius(rect, 0, 0);
		if (z.horizontal) {
			lv_obj_set_pos(rect, z.x + i * rect_w, combi_y);
		} else {
			lv_obj_set_pos(rect, z.x, combi_y + i * rect_h);
		}

		// Lettre Y/B centree. Texte/couleur rafraichis par update_zone_rects().
		lv_obj_t *lbl_yb = lv_label_create(rect);
		lv_label_set_text(lbl_yb, "?");
		lv_obj_center(lbl_yb);
		z.rects[i] = rect;

		// Cliquable : tap court = +1, appui long = -1. Les 4 rects partagent
		// la meme cible (user_data = index zone).
		lv_obj_add_flag(rect, LV_OBJ_FLAG_CLICKABLE);
		lv_obj_add_event_cb(rect, zone_next_cb, LV_EVENT_SHORT_CLICKED,
				(void*)(intptr_t)zi);
		lv_obj_add_event_cb(rect, zone_prev_cb, LV_EVENT_LONG_PRESSED,
				(void*)(intptr_t)zi);
	}

	// Affichage initial selon la valeur courante du byte settings.pickup_Pn.
	update_zone_rects(z);
}

/**
 * Cree l'ecran de configuration des 8 zones de prise (P1-P4, P11-P14).
 *
 * Vue operateur rotation 180° (bleu bas-gauche) :
 *   Rangee 1       :        [P3-H]    [P13-H]
 *   Rangee 2 (Vs)  :  [P2-V]  [P4-H]  [P12-V]
 *   Rangee 3 (Vs)  :  [P1-V]  [P14-H]  [P11-V]
 *   Bouton MENU bleu centre en bas.
 *
 * Chaque tap sur une fleche cycle l'index 0..5 du byte settings.pickup_Pn
 * correspondant et incremente settings.seq_touch (protocole I2C avec OPOS6UL).
 *
 * Voir teensy/IO_t41_ToF_DetectionBeacon/MATCH_CONFIG_UI.md.
 */
static void create_pickup_config(void) {
	lv_obj_t *scr = lv_scr_act();
	lv_obj_t *label;

	// Cablage runtime des value_ptr vers les bytes de `settings`.
	// L'ordre correspond au tableau pickup_zones[] declare en tete de fichier.
	pickup_zones[0].value_ptr = &settings.pickup_P2;
	pickup_zones[1].value_ptr = &settings.pickup_P3;
	pickup_zones[2].value_ptr = &settings.pickup_P13;
	pickup_zones[3].value_ptr = &settings.pickup_P12;
	pickup_zones[4].value_ptr = &settings.pickup_P1;
	pickup_zones[5].value_ptr = &settings.pickup_P4;
	pickup_zones[6].value_ptr = &settings.pickup_P14;
	pickup_zones[7].value_ptr = &settings.pickup_P11;

	for (int zi = 0; zi < 8; zi++) create_pickup_zone_widget(scr, zi);

	// Bouton retour "MENU" noir, centre en bas de l'ecran.
	pickup_back_btn_handle = lv_btn_create(scr);
	lv_obj_set_size(pickup_back_btn_handle, 100, 64);
	lv_obj_align(pickup_back_btn_handle, LV_ALIGN_BOTTOM_MID, 0, -4);
	lv_obj_set_style_bg_color(pickup_back_btn_handle, lv_color_black(), 0);
	label = lv_label_create(pickup_back_btn_handle);
	lv_label_set_text(label, "MENU");
	lv_obj_set_style_text_color(label, lv_color_white(), 0);
	lv_obj_center(label);
	lv_obj_add_event_cb(pickup_back_btn_handle, back_to_menu_cb,
			LV_EVENT_CLICKED, NULL);

	// Reperes "nid" : carre bleu pointille en bas-gauche, jaune en bas-droite.
	// Orientation rotation 180° (bleu = cote gauche, jaune = cote droite).
	// Dimensions 48x48, places dans l'espace libre a cote du bouton MENU (100 wide).
	const int SQ_SIZE = 48;
	const int SQ_Y    = 240 - 4 - SQ_SIZE - 8;   // y=180 (aligne dans la bande MENU)
	create_dashed_square(scr, pickup_bluesq_pts, pickup_bluesq_lines,
			16, SQ_Y, SQ_SIZE, PICKUP_BLUE);
	create_dashed_square(scr, pickup_yelsq_pts, pickup_yelsq_lines,
			320 - 16 - SQ_SIZE, SQ_Y, SQ_SIZE, PICKUP_YELLOW);
}

/**
 * Ecran match : logo PMX plein ecran + score en haut a droite (font 16).
 */
static void show_match_screen(void) {
	lv_obj_clean(lv_scr_act());
	invalidate_menu_handles();
	invalidate_pickup_handles();

	lv_obj_set_style_bg_color(lv_scr_act(), lv_color_make(30, 27, 59), 0);

	lv_obj_t *img = lv_img_create(lv_scr_act());
	lv_img_set_src(img, &logo_pmx);
	lv_obj_set_pos(img, 0, 0);

	// Score en haut a droite, blanc, font 16
	lbl_match_score = lv_label_create(lv_scr_act());
	lv_obj_set_style_text_color(lbl_match_score, lv_color_white(), 0);
	lv_obj_set_style_text_font(lbl_match_score, &lv_font_montserrat_16, 0);
	lv_label_set_text_fmt(lbl_match_score, "%d", settings.matchPoints);
	lv_obj_set_pos(lbl_match_score, 290, 6);

	lcd_screen_state = 1;
}

/**
 * Ecran fin de match : favicon centré en haut + score en gros en bas (font 48).
 */
static void show_endmatch_screen(void) {
	lv_obj_clean(lv_scr_act());
	invalidate_menu_handles();
	invalidate_pickup_handles();

	lv_obj_set_style_bg_color(lv_scr_act(), lv_color_make(30, 27, 59), 0);

	lv_obj_t *img = lv_img_create(lv_scr_act());
	lv_img_set_src(img, &favicon_pmx);
	lv_obj_set_pos(img, 0, 0);

	// Score en gros, centre dans la moitie basse (y=120..240)
	lbl_match_score = lv_label_create(lv_scr_act());
	lv_obj_set_style_text_color(lbl_match_score, lv_color_white(), 0);
	lv_obj_set_style_text_font(lbl_match_score, &lv_font_montserrat_36, 0);
	lv_label_set_text_fmt(lbl_match_score, "%d", settings.matchPoints);
	lv_obj_align(lbl_match_score, LV_ALIGN_CENTER, 0, 60);

	lcd_screen_state = 2;
}

void screen_loop() {
	if (!screen_available) return; // pas d'ecran : no-op

	// --- Transitions d'ecran selon matchState ---
	// matchState: 0=CONFIG, 1=ARMED, 2=PRIMED, 3=MATCH, 4=END
	// lcd_screen_state: 0=menu, 1=match, 2=endmatch, 3=pickup config
	// En ARMED+PRIMED on reste sur le menu (config editable, hors couleur).
	// Etat 3 (pickup config) est exclu du retour force au menu : l'operateur
	// quitte la config zones uniquement via le bouton MENU (ou matchState>=3).
	if (settings.matchState == 0 && lcd_screen_state != 0 && lcd_screen_state != 3) {
		// OPOS6UL redemarré ou reset -> retour au menu
		lv_obj_clean(lv_scr_act());
		invalidate_menu_handles();
		invalidate_pickup_handles();
		create_match_menu();
		lcd_screen_state = 0;
	}
	if (settings.matchState == 3 && (lcd_screen_state == 0 || lcd_screen_state == 3)) {
		show_match_screen();
	}
	if (settings.matchState >= 4 && lcd_screen_state != 2) {
		show_endmatch_screen();
	}

	// En mode match ou fin : rafraichir le score si besoin
	if (lcd_screen_state >= 1) {
		static uint8_t last_score = 0xFF;
		if (lbl_match_score && settings.matchPoints != last_score) {
			lv_label_set_text_fmt(lbl_match_score, "%d", settings.matchPoints);
			if (lcd_screen_state == 2) {
				// Recentrer apres changement de texte (nb chiffres)
				lv_obj_align(lbl_match_score, LV_ALIGN_CENTER, 0, 60);
			}
			last_score = settings.matchPoints;
		}
		lv_task_handler();
		return;
	}

	// Timeout actionReq : si non consomme par l'OPOS6UL en 1s, reset a 0.
	// Couvre le cas ou l'OPOS6UL n'est pas demarree.
	if (settings.actionReq != 0 && (millis() - actionReq_set_ms > 1000)) {
		settings.actionReq = 0;
		updateSetposButton();
	}

	// Rafraichissement periodique des widgets LVGL a partir de settings.
	// Couvre a la fois les labels read-only ecrits par l'OPOS6UL (numOfBots,
	// matchPoints) et les widgets du menu pre-match qui peuvent aussi etre
	// modifies par l'OPOS6UL via I2C (matchColor, strategy, advDiameter,
	// ledLuminosity). Sans ca, un clic sur le shield 2x16 cote OPOS6UL ne se
	// voit pas sur l'ecran tactile : les callbacks LVGL ne sont pas rappeles.
	//
	// 5 Hz (200 ms) : compromis entre reactivite visuelle et cout LVGL.
	// Memo des valeurs pour ne pas redessiner si la valeur n'a pas change.
	static uint32_t last_refresh_ms = 0;
	static int8_t  last_numOfBots     = -1;
	static uint8_t last_matchPoints   = 0xFF;
	static uint8_t last_matchColor    = 0xFF;
	static uint8_t last_strategy      = 0xFF;
	static uint8_t last_advDiameter   = 0xFF;
	static int8_t  last_ledLuminosity = -1;
	static uint8_t last_matchState    = 0xFF;

	uint32_t now = millis();
	if (now - last_refresh_ms > 200) {
		last_refresh_ms = now;

		// Labels read-only (ecrits par OPOS6UL)
		if (lbl_numOfBots_value && settings.numOfBots != last_numOfBots) {
			lv_label_set_text_fmt(lbl_numOfBots_value, "%d", settings.numOfBots);
			last_numOfBots = settings.numOfBots;
		}
		if (lbl_matchPoints_value && settings.matchPoints != last_matchPoints) {
			lv_label_set_text_fmt(lbl_matchPoints_value, "%d", settings.matchPoints);
			last_matchPoints = settings.matchPoints;
		}

		// Widgets du menu pre-match (partages entre touch et OPOS6UL)
		if (btn_color_handle && settings.matchColor != last_matchColor) {
			updateColorButton(btn_color_handle);
			last_matchColor = settings.matchColor;
		}
		if (settings.strategy != last_strategy) {
			updateStrategyButtons();
			last_strategy = settings.strategy;
		}
		if (lbl_adv_value && settings.advDiameter != last_advDiameter) {
			lv_label_set_text_fmt(lbl_adv_value, "%d cm", settings.advDiameter);
			last_advDiameter = settings.advDiameter;
		}
		if (lbl_lum_value && settings.ledLuminosity != last_ledLuminosity) {
			lv_label_set_text_fmt(lbl_lum_value, "%d", settings.ledLuminosity);
			last_ledLuminosity = settings.ledLuminosity;
		}

		// matchState change (OPOS6UL a fait setPos ou reset) :
		// - bouton SETPOS/RESET change de libelle/couleur
		// - boutons couleur et ZONES verrouilles visuellement en ARMED+
		if (settings.matchState != last_matchState) {
			updateSetposButton();
			updateColorButtonLock();
			updateZonesButtonLock();
			last_matchState = settings.matchState;
		}
	}
	lv_task_handler(); // lvgl gui handler
}

