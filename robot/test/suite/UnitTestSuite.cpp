/*!
 * \file
 * \brief Implementation de la classe UnitTestSuite.
 */

#include "UnitTestSuite.hpp"

#include <unistd.h>
#include <sstream>
#include <string>

#include "../../src/common/log/appender/MemoryAppender.hpp"
#include "../../src/common/log/Exception.hpp"
#include "../../src/common/log/Logger.hpp"
#include "UnitTestAppender.hpp"

UnitTestSuite::UnitTestSuite() :
        tests_()
{
}

void UnitTestSuite::run()
{
    logger().info("Start Unit tests");
    UnitTestAppender *appender = (UnitTestAppender*) &logger().appender();

    if (appender != NULL) {
        //logger().debug("flush()");
        appender->flush();
        //logger().debug("utils::PointerList<UnitTest*> begin");
        for (auto i = tests_.begin(); i != tests_.end(); i++) {
            UnitTest *test = *i;
            //logger().debug("increaseIndent()");
            appender->increaseIndent();

            logger().info() << "Début d'éxecution de <" << test->name() << "> " << logs::end;
            appender->increaseIndent();
            try {
                test->suite();
            } catch (logs::Exception *exception) {
                std::ostringstream oss;
                oss << "Le test a généré une exception: " << exception->what();
                test->fail(oss.str());
            } catch (std::exception *exception) {
                std::ostringstream oss;
                oss << "Le test a généré une exception: " << exception->what();
                test->fail(oss.str());
            }
            appender->decreaseIndent();
            logger().info() << "Fin d'éxecution de <" << test->name() << "> " << logs::end;

            appender->decreaseIndent();
            appender->flush();
        }
    }
    //usleep(1000000);
    logger().info("End of Unit tests");
}
