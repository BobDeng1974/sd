#pragma once


#include <iostream>
#include "time_stamp.h"


class Timer {

public:

    Timer(const std::string &name = "");

    ~Timer();

    void start();

    double read();

    void print(const std::string &addtion = "", bool restart = false);

    friend std::ostream &operator<<(std::ostream &out, Timer &timer);

private:

    TimeStamp startTime;
    TimeStamp total;

    std::string name;

};
// class Timing
