/*!
 * \file
 * \brief Implementation de MenuController.
 */

#include "MenuController.hpp"

#include "log/Logger.hpp"
#include "Robot.hpp"

void MenuController::add(std::unique_ptr<IMenuSource> s)
{
	if (!s) return;
	logger().info() << "add source: " << s->name()
			<< " (alive=" << s->isAlive() << ")" << logs::end;
	sources_.push_back(std::move(s));
}

void MenuController::tick()
{
	// Phase 1 : tous les pollInputs sans gate isAlive().
	// Si une source etait morte, c'est dans pollInputs qu'elle a une chance
	// de se reveiller (retry du hardware). Chaque source short-circuite en
	// interne si son hardware est reellement indisponible.
	for (auto& s : sources_) {
		s->pollInputs(robot_);
	}
	// Phase 2 : refreshDisplay gate sur isAlive() pour eviter d'ecrire
	// vers du hardware qu'on sait mort (typiquement I2C qui vient d'echouer
	// sur le poll precedent).
	for (auto& s : sources_) {
		if (s->isAlive()) s->refreshDisplay(robot_);
	}
}

void MenuController::pollInputsOnly()
{
	for (auto& s : sources_) {
		s->pollInputs(robot_);
	}
}

bool MenuController::anyAlive() const
{
	for (const auto& s : sources_) {
		if (s->isAlive()) return true;
	}
	return false;
}

size_t MenuController::aliveCount() const
{
	size_t n = 0;
	for (const auto& s : sources_) if (s->isAlive()) ++n;
	return n;
}
