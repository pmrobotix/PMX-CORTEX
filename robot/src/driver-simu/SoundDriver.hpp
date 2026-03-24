/*!
 * \file
 * \brief Implementation SIMU du driver de son.
 *
 * Simule les sorties audio en loggant les commandes dans la console.
 */

#ifndef SIMU_SOUNDDRIVER_HPP_
#define SIMU_SOUNDDRIVER_HPP_

#include <string>
#include <vector>

#include "interface/ASoundDriver.hpp"
#include "log/LoggerFactory.hpp"

/*!
 * \brief Implementation simulation du driver de son.
 */
class SoundDriver: public ASoundDriver
{
private:

	static inline const logs::Logger& logger()
	{
		static const logs::Logger &instance = logs::LoggerFactory::logger("SoundDriver.SIMU");
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
