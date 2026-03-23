/*!
 * \file
 * \brief Implémentation des tests de la classe utils::Mutex.
 */

#include "MutexTest.hpp"
#include "../../src/common/thread/Mutex.hpp"

void test::MutexTest::suite()
{
    testLockUnlock();
    testTryLock();
}

void test::MutexTest::testLockUnlock()
{
    utils::Mutex m;
    m.lock();
    m.unlock();
    this->assert(true, "lock/unlock basique doit fonctionner sans deadlock");
}

void test::MutexTest::testTryLock()
{
    utils::Mutex m;

    bool first = m.tryLock();
    this->assert(first, "tryLock sur mutex libre doit retourner true");

    bool second = m.tryLock();
    this->assert(!second, "tryLock sur mutex deja verrouille doit retourner false");

    m.unlock();

    bool third = m.tryLock();
    this->assert(third, "tryLock apres unlock doit retourner true");
    m.unlock();
}
