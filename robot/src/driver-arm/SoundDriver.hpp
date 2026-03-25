/*!
 * \file
 * \brief Implementation ARM du driver de son pour l'OPOS6UL.
 *
 * Stub qui log les commandes audio (pas de hardware son sur OPOS6UL).
 */

#ifndef OPOS6UL_SOUNDDRIVER_HPP_
#define OPOS6UL_SOUNDDRIVER_HPP_

#include <string>
#include <vector>

#include "interface/ASoundDriver.hpp"
#include "log/LoggerFactory.hpp"

/*!
 * \brief Implementation ARM du driver de son.
 *
 * Log les commandes audio sans les executer (pas de peripherique
 * audio sur l'OPOS6UL).
 */
class SoundDriver: public ASoundDriver
{
private:

	static inline const logs::Logger & logger()
	{
		static const logs::Logger & instance = logs::LoggerFactory::logger("SoundDriver.OPO");
		return instance;
	}

public:

	/*!
	 * \brief Constructeur.
	 */
	SoundDriver();

	/*!
	 * \brief Destructeur.
	 */
	~SoundDriver();

	void beep(const std::string &args, bool bSynchronous) override;
	void tone(unsigned frequency, unsigned ms, bool bSynchronous) override;
	void tone(const std::vector<std::vector<float> > &sequence, bool bSynchronous) override;
	void play(const std::string &soundfile, bool bSynchronous) override;
	void speak(const std::string &text, bool bSynchronous) override;

};

#endif
