/*!
 * \file
 * \brief Implémentation de la classe DevUnitTest.
 */

#include "../../src/common/log/Logger.hpp"
#include "../../src/common/log/LoggerFactory.hpp"
#include "UnitTest.hpp"
#include "UnitTestAppender.hpp"

UnitTest::UnitTest(const std::string & name)
{
	name_ = name;
}

void UnitTest::assert(bool test, const std::string & message)
{
	if (!test)
	{
		fail(message);
	}
}

void UnitTest::fail(const std::string & message)
{
	logger().error() << name() << ":" << message << logs::end;
}

void UnitTest::expectedError(const std::string & expected)
{
	this->expectedError(expected, "An error was expected but was not raised");
}

void UnitTest::expectedError(const std::string & expected, const std::string & message)
{
	logs::Logger* rootLogger = logs::LoggerFactory::instance().rootLogger();
	UnitTestAppender* appender = (UnitTestAppender*) &rootLogger->appender();

	if (!appender->expectedError(expected))
	{
		this->fail(message);
	}
}
