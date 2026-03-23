/*!
 * \file
 * \brief Définition de la classe MutexTest.
 */

#ifndef TEST_MUTEX_TEST_HPP
#define TEST_MUTEX_TEST_HPP

#include "../suite/UnitTest.hpp"

namespace test {

/*!
 * \brief Teste la classe utils::Mutex.
 */
class MutexTest : public UnitTest {
public:
    MutexTest() : UnitTest("MutexTest") {}
    virtual ~MutexTest() {}

    virtual void suite();

    /*!
     * \brief Vérifie que lock/unlock fonctionne sans deadlock.
     */
    void testLockUnlock();

    /*!
     * \brief Vérifie le comportement de tryLock sur mutex libre et occupé.
     */
    void testTryLock();
};

}

#endif
