/*!
 * \file
 * \brief Implémentation des tests de la classe utils::PointerList.
 */

#include "PointerListTest.hpp"
#include "../../src/common/utils/PointerList.hpp"

void test::PointerListTest::suite()
{
    testEmptyList();
    testAddAndIterate();
    testSize();
    testConstructWithCopies();
}

void test::PointerListTest::testEmptyList()
{
    utils::PointerList<int*> list;

    this->assert(list.empty(), "Une liste vide doit etre empty()");
    this->assert(list.size() == 0, "Une liste vide doit avoir une taille de 0");
    this->assert(list.begin() == list.end(), "begin() doit etre egal a end() pour une liste vide");
}

void test::PointerListTest::testAddAndIterate()
{
    utils::PointerList<int*> list;

    int a = 10, b = 20, c = 30;
    list.push_back(&a);
    list.push_back(&b);
    list.push_back(&c);

    this->assert(list.size() == 3, "La liste doit contenir 3 elements");

    // Verification de l'ordre d'iteration
    auto it = list.begin();
    this->assert(*(*it) == 10, "Le premier element doit etre 10");
    ++it;
    this->assert(*(*it) == 20, "Le deuxieme element doit etre 20");
    ++it;
    this->assert(*(*it) == 30, "Le troisieme element doit etre 30");
    ++it;
    this->assert(it == list.end(), "Apres 3 iterations, on doit etre a end()");

    // Verification de front/back
    this->assert(*(list.front()) == 10, "front() doit etre 10");
    this->assert(*(list.back()) == 30, "back() doit etre 30");
}

void test::PointerListTest::testSize()
{
    utils::PointerList<int*> list;

    int vals[5] = {1, 2, 3, 4, 5};
    for (int i = 0; i < 5; ++i) {
        list.push_back(&vals[i]);
    }

    this->assert(list.size() == 5, "La liste doit contenir 5 elements");

    list.pop_front();
    this->assert(list.size() == 4, "Apres pop_front, la liste doit contenir 4 elements");

    list.pop_back();
    this->assert(list.size() == 3, "Apres pop_back, la liste doit contenir 3 elements");

    list.clear();
    this->assert(list.empty(), "Apres clear, la liste doit etre vide");
}

void test::PointerListTest::testConstructWithCopies()
{
    int val = 42;
    int* ptr = &val;
    utils::PointerList<int*> list(3, ptr);

    this->assert(list.size() == 3, "La liste doit contenir 3 copies");

    for (auto it = list.begin(); it != list.end(); ++it) {
        this->assert(*(*it) == 42, "Chaque element doit pointer vers 42");
    }
}
