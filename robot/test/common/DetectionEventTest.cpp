/*!
 * \file
 * \brief Implementation du test unitaire DetectionEvent.
 */

#include "DetectionEventTest.hpp"

#include "../../src/common/geometry/DetectionEvent.hpp"

void test::DetectionEventTest::suite()
{
	testDefaultConstruction();
	testIsBlocking();
	testIsSlowDown();
	testHasPosition();
	testClear();
}

void test::DetectionEventTest::testDefaultConstruction()
{
	DetectionEvent ev;
	this->assert(ev.frontLevel == 0, "default frontLevel == 0");
	this->assert(ev.backLevel == 0, "default backLevel == 0");
	this->assert(ev.x_adv_mm < 0, "default x_adv_mm < 0 (inconnu)");
	this->assert(ev.y_adv_mm < 0, "default y_adv_mm < 0 (inconnu)");
	this->assert(ev.valid == false, "default valid == false");
	this->assert(ev.timestamp_us == 0, "default timestamp_us == 0");
}

void test::DetectionEventTest::testIsBlocking()
{
	DetectionEvent ev;

	ev.frontLevel = 3;
	this->assert(!ev.isBlocking(), "front level 3 => pas bloquant");

	ev.frontLevel = 4;
	this->assert(ev.isBlocking(), "front level 4 => bloquant");

	ev.frontLevel = 0;
	ev.backLevel = -3;
	this->assert(!ev.isBlocking(), "back level -3 => pas bloquant");

	ev.backLevel = -4;
	this->assert(ev.isBlocking(), "back level -4 => bloquant");
}

void test::DetectionEventTest::testIsSlowDown()
{
	DetectionEvent ev;

	ev.frontLevel = 2;
	this->assert(!ev.isSlowDown(), "front level 2 => pas de ralentissement");

	ev.frontLevel = 3;
	this->assert(ev.isSlowDown(), "front level 3 => ralentissement");

	ev.frontLevel = 4;
	this->assert(ev.isSlowDown(), "front level 4 => ralentissement (aussi bloquant)");

	ev.frontLevel = 0;
	ev.backLevel = -2;
	this->assert(!ev.isSlowDown(), "back level -2 => pas de ralentissement");

	ev.backLevel = -3;
	this->assert(ev.isSlowDown(), "back level -3 => ralentissement");
}

void test::DetectionEventTest::testHasPosition()
{
	DetectionEvent ev;

	this->assert(!ev.hasPosition(), "default => pas de position");

	ev.valid = true;
	ev.x_adv_mm = 500.0f;
	ev.y_adv_mm = 800.0f;
	this->assert(ev.hasPosition(), "valid + x/y positifs => position connue");

	ev.x_adv_mm = -1.0f;
	this->assert(!ev.hasPosition(), "x negatif => pas de position");
}

void test::DetectionEventTest::testClear()
{
	DetectionEvent ev;
	ev.frontLevel = 4;
	ev.backLevel = -4;
	ev.x_adv_mm = 500.0f;
	ev.y_adv_mm = 800.0f;
	ev.d_adv_mm = 1200.0f;
	ev.valid = true;
	ev.timestamp_us = 123456;

	ev.clear();

	this->assert(ev.frontLevel == 0, "clear => frontLevel == 0");
	this->assert(ev.backLevel == 0, "clear => backLevel == 0");
	this->assert(ev.x_adv_mm < 0, "clear => x_adv_mm < 0");
	this->assert(ev.valid == false, "clear => valid == false");
	this->assert(ev.timestamp_us == 0, "clear => timestamp_us == 0");
}
