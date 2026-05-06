/*
 * Gestures.cpp
 *
 * Registry de gestes ToF + arbitrage non-bloquant.
 * Voir GESTURES_ARCHITECTURE.md pour la doc complete.
 */

#include "Gestures.hpp"
#include "TofSensors.h"   // UA_GESTURE_*, SWIPE_*, last_ua_gesture, last_swipe_gesture, proximity_level
#include <Arduino.h>      // millis()

extern volatile int proximity_level;
extern volatile int last_ua_gesture;
extern volatile int last_swipe_gesture;
extern int match_mode_actif;

volatile FlashState g_flash = { false, 0, 0, 0 };

// ============================================================================
// Helpers
// ============================================================================

// Couleurs RGB565 (format FastLED_NeoMatrix) :
// R5 G6 B5 = 16 bits. Pour convertir d'un RGB888 :
//   color565 = ((R & 0xF8) << 8) | ((G & 0xFC) << 3) | (B >> 3)
static const uint16_t COLOR_RED     = 0xF800;
static const uint16_t COLOR_GREEN   = 0x07E0;
static const uint16_t COLOR_CYAN    = 0x07FF;
static const uint16_t COLOR_MAGENTA = 0xF81F;
static const uint16_t COLOR_WHITE   = 0xFFFF;

static void start_flash(uint16_t color, uint32_t duration_ms) {
	g_flash.color       = color;
	g_flash.start_ms    = millis();
	g_flash.duration_ms = duration_ms;
	g_flash.active      = true;
}

// ============================================================================
// Conditions (lectures rapides de l'etat partage, sans effet de bord)
// ============================================================================

static bool cond_bilateral() {
	return last_ua_gesture == UA_GESTURE_BILATERAL;
}
static bool cond_long_hold() {
	return last_ua_gesture == UA_GESTURE_LONG_HOLD;
}
static bool cond_convergent() {
	return last_ua_gesture == UA_GESTURE_CONVERGENT;
}
static bool cond_swipe_cw() {
	return last_swipe_gesture == SWIPE_CW;
}
static bool cond_swipe_ccw() {
	return last_swipe_gesture == SWIPE_CCW;
}
static bool cond_hey() {
	return proximity_level == 2;
}

// ============================================================================
// Actions (consomment l'etat partage si pertinent, non-bloquantes)
// ============================================================================

static void act_toggle_match() {
	last_ua_gesture = UA_GESTURE_NONE;
	match_mode_actif = (match_mode_actif == 0) ? 1 : 0;
}
static void act_flash_red() {
	last_ua_gesture = UA_GESTURE_NONE;
	start_flash(COLOR_RED, 1500);
}
static void act_flash_green() {
	last_ua_gesture = UA_GESTURE_NONE;
	start_flash(COLOR_GREEN, 1000);
}
static void act_flash_cyan() {
	last_swipe_gesture = SWIPE_NONE;
	start_flash(COLOR_CYAN, 800);
}
static void act_flash_magenta() {
	last_swipe_gesture = SWIPE_NONE;
	start_flash(COLOR_MAGENTA, 800);
}
static void act_hey() {
	// L'affichage scroll "Hey!!" est non-bloquant cote rendu : pour l'instant
	// on signale juste via le flash (placeholder). A remplacer par une FSM
	// scroll dediee lors de l'integration etape 7.
	start_flash(COLOR_WHITE, 500);
}

// ============================================================================
// Registry
// ============================================================================
//
// Ordre = priorite implicite quand plusieurs conditions partagent une variable
// d'etat consommee : le premier match consomme la variable, les suivants voient
// l'etat reset (NONE).
//
// IMPORTANT : la priorite UA (LONG_HOLD > CONVERGENT > BILATERAL) est en fait
// deja imposee par TofSensors::tof_loop() qui n'ecrit qu'UNE valeur dans
// last_ua_gesture. L'ordre ici n'a donc pas d'effet pratique pour UA, mais on
// le garde aligne avec la priorite logique.

static Gesture s_registry[] = {
	{ "BILATERAL",  true,  500,  cond_bilateral,  act_toggle_match,   0 },
	{ "LONG_HOLD",  true,  1500, cond_long_hold,  act_flash_red,      0 },
	{ "CONVERGENT", true,  1000, cond_convergent, act_flash_green,    0 },
	{ "SWIPE_CW",   true,  800,  cond_swipe_cw,   act_flash_cyan,     0 },
	{ "SWIPE_CCW",  true,  800,  cond_swipe_ccw,  act_flash_magenta,  0 },
	{ "HEY",        true,  2000, cond_hey,        act_hey,            0 },
};
static const int N_GESTURES = sizeof(s_registry) / sizeof(s_registry[0]);

// ============================================================================
// Boucle d'evaluation
// ============================================================================

void gestures_evaluate() {
	uint32_t now = millis();
	for (int i = 0; i < N_GESTURES; i++) {
		Gesture &g = s_registry[i];
		if (!g.enabled) continue;
		if (g.last_fire_ms != 0 && (now - g.last_fire_ms) < g.cooldown_ms) continue;
		if (g.condition()) {
			Serial.print("GESTURE FIRE: ");
			Serial.println(g.name);
			g.action();
			g.last_fire_ms = now;
		}
	}

	// Anti-accumulation : si un geste UA/swipe a ete classifie par tof_loop
	// mais qu'aucun gesture activ ne le consomme, on reset silencieusement
	// pour eviter qu'il reste indefiniment et fire au prochain cycle ou il
	// passerait enabled=true.
	if (last_ua_gesture != UA_GESTURE_NONE) {
		last_ua_gesture = UA_GESTURE_NONE;
	}
	if (last_swipe_gesture != SWIPE_NONE) {
		last_swipe_gesture = SWIPE_NONE;
	}
}
