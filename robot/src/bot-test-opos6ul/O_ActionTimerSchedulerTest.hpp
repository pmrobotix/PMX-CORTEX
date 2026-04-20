/*!
 * \file
 * \brief Définition de la classe O_ActionTimerSchedulerTest.
 */

#ifndef OPOS6UL_ACTIONTIMERSCHEDULERTEST_HPP_
#define	OPOS6UL_ACTIONTIMERSCHEDULERTEST_HPP_

#include <string>

#include "action/IAction.hpp"
#include "utils/FunctionalTest.hpp"
#include "utils/Chronometer.hpp"
#include "timer/ITimerScheduledListener.hpp"
#include "log/Logger.hpp"
#include "log/LoggerFactory.hpp"

/*!
 * \brief Effectue un test de l'action manager avec timer
 */
class O_ActionTimerSchedulerTest: public FunctionalTest
{
private:

    /*!
     * \brief Retourne le \ref Logger associé à la classe.
     */
    static inline const logs::Logger & logger()
    {
        static const logs::Logger & instance = logs::LoggerFactory::logger("O_ActionTimerSchedulerTest");
        return instance;
    }
public:

    /*!
     * \brief Constructeur de la classe.
     */
    O_ActionTimerSchedulerTest() :
            FunctionalTest("ActionTimerScheduler", "actions + timers", "ats")
    {
    }

    /*!
     * \brief Destructeur de la classe.
     */
    virtual ~O_ActionTimerSchedulerTest()
    {
    }

    /*!
     * \brief Execute le test.
     */
    virtual void run(int argc, char** argv);

};

/*!
 * \brief Cette action permet de
 *
 */
class TestAction: public IAction
{
private:

    /*!
     * \brief Retourne le \ref Logger associé à la classe \ref TestAction.
     */
    static const logs::Logger & logger()
    {
        static const logs::Logger & instance = logs::LoggerFactory::logger("O_ActionTimerSchedulerTest-Action");
        return instance;
    }

    /*!
     * \brief Référence vers le test.
     */
    O_ActionTimerSchedulerTest & amt_;

    std::string name_;

    utils::Chronometer chrono_;

    int i_;

public:

    /*!
     * \brief Constructeur de la classe.
     * \param amt
     *        Reference vers l'objet associée.
     */
    TestAction(O_ActionTimerSchedulerTest & amt, std::string name);

    /*!
     * \brief Destructeur de la classe.
     */
    virtual inline ~TestAction()
    {
        logger().debug() << "~TestAction()" << logs::end;
    }

    /*!
     * \brief Execution de l'action.
     */
    virtual bool execute();

    /*!
     * \brief Retourne la description de l'action.
     */
    virtual inline std::string info()
    {
        return name_;
    }
};

/*!
 * \brief Ce timer
 *
 */
class TestTimer: public ITimerScheduledListener
{
private:

    /*!
     * \brief Retourne le \ref Logger associé à la classe \ref TestTimer.
     */
    static const logs::Logger & logger()
    {
        static const logs::Logger & instance = logs::LoggerFactory::logger("O_ActionTimerSchedulerTest-Timer");
        return instance;
    }

    /*!
     * \brief Référence vers le test.
     */
    O_ActionTimerSchedulerTest & amt_;

    utils::Chronometer chrono_;

   int lasttime_;

public:

    /*!
     * \brief Constructeur de la classe.
     * \param amt
     *        Reference vers l'objet associée.
     */
    TestTimer(O_ActionTimerSchedulerTest & amt, int timeSpan_ms, std::string name);

    /*!
     * \brief Destructeur de la classe.
     */
    virtual inline ~TestTimer()
    {
        //logger().debug() << "~TestTimer()" << logs::end;
    }

    virtual void onTimer(utils::Chronometer chrono);

    virtual void onTimerEnd(utils::Chronometer chrono);

    /*!
     * \brief Retourne la description de l'action.
     */
    virtual inline std::string info()
    {
        return name();
    }
};

#endif
