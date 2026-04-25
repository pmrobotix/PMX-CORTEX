/*!
 * \file
 * \brief Définition de la classe O_ServoStepTest.
 */

#ifndef OPOS6UL_SERVOSTEPTEST_HPP_
#define	OPOS6UL_SERVOSTEPTEST_HPP_

#include "utils/FunctionalTest.hpp"
#include "log/LoggerFactory.hpp"

/*!
 * \brief Effectue un test sur les servos.
 */
class O_ServoStepTest: public FunctionalTest
{
private:

	/*!
	 * \brief Retourne le \ref Logger associé à la classe \ref O_ServoStepTest.
	 */
	static inline const logs::Logger & logger()
	{
		static const logs::Logger & instance = logs::LoggerFactory::logger("O_ServoStepTest");
		return instance;
	}
public:

	/*!
	 * \brief Constructeur de la classe.
	 */
	O_ServoStepTest()
			: FunctionalTest("ServoStep", "Servo moving step by step.", "ss")
	{
	}

	std::string defaultArgs() const override { return "1"; }

	std::string usageHelp() const override
	{
		return
			"        args: <num> [step] [pos] [speed]\n"
			"              num   = numero du servo\n"
			"              step  = increment/decrement (defaut 2)\n"
			"              pos   = position initiale 0..4095 (defaut 512, AX12)\n"
			"              speed = vitesse (defaut 0)\n"
			"        ex:   ss 1            # servo 1, step 2, pos 512\n"
			"              ss 5 10 1024 50  # servo 5 pos 1024 step 10 vit 50";
	}

	/*!
	 * \brief Destructeur de la classe.
	 */
	virtual ~O_ServoStepTest()
	{
	}

	/*!
	 * \brief Execute le test.
	 */
	virtual void run(int argc, char** argv);


	void configureConsoleArgs(int argc, char** argv);
};

#endif
