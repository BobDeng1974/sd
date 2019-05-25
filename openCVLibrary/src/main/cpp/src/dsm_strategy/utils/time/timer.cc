/*
 * timer.cpp
 *
 *  Created on: Feb 1, 2016
 *      Author: liujj
 */
/**
 * constructor and destructor
 */
#include "timer.h"

Timer::Timer(const std::string &name) : startTime(0), total(0), name(name) {
    start();
}

Timer::~Timer() {
}


/**
 * Timer start
 */
void Timer::start() {
    total = TimeStamp(0);
    startTime = TimeStamp::now();
}

/**
 * Get the time between start() and stop().
 */
double Timer::read() {
    total = (TimeStamp::now() - startTime);
    return total.toSec();
}

void Timer::print(const std::string &addtion, bool restart) {
    read();
    std::cout << name << " " << addtion << ": " << total.toSec() << std::endl;
    if (restart)
        start();
}


std::ostream &operator<<(std::ostream &out, Timer &timer) {
    out << timer.name << " " << timer.read();
    return out;
}
