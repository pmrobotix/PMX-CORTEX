/*
 * Gestures.hpp
 *
 * Registry de gestes ToF non-bloquant pour la balise.
 * Voir GESTURES_ARCHITECTURE.md pour le pourquoi et les regles.
 */

#pragma once

#include <stdint.h>

/// Description d'un geste : condition + action + cooldown.
/// Tous les champs sont declaratifs ; aucune logique cachee.
struct Gesture {
	const char *name;          ///< nom pour logs/debug
	bool       enabled;        ///< ON/OFF, flipper pour activer/desactiver
	uint32_t   cooldown_ms;    ///< delai min entre 2 triggers du meme geste
	bool     (*condition)();   ///< true = geste detecte, doit etre court et non-bloquant
	void     (*action)();      ///< effet, doit etre non-bloquant (FSM/flag, jamais delay)
	uint32_t   last_fire_ms;   ///< timestamp du dernier trigger (init 0)
};

/// FSM flash non-bloquante : action() marque l'etat, le rendu LED le lit.
/// Une seule instance partagee : seul le dernier flash demande s'affiche.
struct FlashState {
	bool     active;
	uint16_t color;        ///< RGB565 (format direct FastLED_NeoMatrix matrix->drawLine)
	uint32_t start_ms;
	uint32_t duration_ms;
};
extern volatile FlashState g_flash;

/// Boucle d'evaluation : a appeler chaque cycle (thread_display, tof_loop fin, ...).
/// Non-bloquante. Itere le registry, declenche les actions des gestes dont la
/// condition est vraie ET qui ne sont pas en cooldown.
void gestures_evaluate();
