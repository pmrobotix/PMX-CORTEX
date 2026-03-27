/*!
 * \file
 * \brief Definition de la classe O_IAByPathTest.
 */

#ifndef OPOS6UL_IABYPATHTEST_HPP_
#define	OPOS6UL_IABYPATHTEST_HPP_

#include "utils/FunctionalTest.hpp"
#include "log/LoggerFactory.hpp"

class Playground;

/*!
 * \brief Effectue un test de l'IAByPath.
 */
class O_IAByPathTest: public FunctionalTest
{
private:

    static inline const logs::Logger & logger()
    {
        static const logs::Logger & instance = logs::LoggerFactory::logger("O_IAByPathTest");
        return instance;
    }

    Playground *p_; //for new ia

public:

    O_IAByPathTest() :
            FunctionalTest("IAbyPath", "test l'ia demo IAByPath")
    {
        p_ = NULL;
    }

    virtual ~O_IAByPathTest()
    {
    }

    virtual void run(int argc, char** argv);

    void IASetup();

    void initPlayground();
};

#endif
