/*
 * TimeStamp.h
 *
 *  Created on: Aug 10, 2016
 *      Author: liujj
 */
#pragma once

#include <iostream>
#include <stdint.h>
#include <stdlib.h>
#include <chrono>

enum TimeType {
    Second, Millisecond, Microsecond, Nanosecond
};

class TimeStamp {
private:
    class TimeScale {
    public:
        TimeScale(TimeType time_type) {
            switch (time_type) {
                case Second:
                    scale_ = 1.0;
                    break;
                case Millisecond:
                    scale_ = 1e-3;
                    break;
                case Microsecond:
                    scale_ = 1e-6;
                    break;
                case Nanosecond:
                    scale_ = 1e-9;
                    break;
            }
        }

        operator double() {
            return scale_;
        }

    private:
        double scale_;
    };

public:
    TimeStamp();

    TimeStamp(int64_t time_stamp, TimeType time_type = Nanosecond);

    virtual ~TimeStamp();

    TimeStamp operator-(const TimeStamp &rhs) const;

    TimeStamp operator+(const TimeStamp &rhs) const;

    TimeStamp operator/(const int64_t &s) const;

    TimeStamp &operator+=(const TimeStamp &rhs);

    TimeStamp &operator-=(const TimeStamp &rhs);

    TimeStamp &operator/=(const int64_t &s);

    bool operator==(const TimeStamp &rhs) const;

    bool operator!=(const TimeStamp &rhs) const;

    bool operator>(const TimeStamp &rhs) const;

    bool operator<(const TimeStamp &rhs) const;

    bool operator>=(const TimeStamp &rhs) const;

    bool operator<=(const TimeStamp &rhs) const;

    int64_t &operator()();

    TimeStamp abs() const;

    double toSec() const;

    static TimeStamp now(TimeType time_type = Nanosecond);

    static TimeStamp fromString(const std::string time, const std::string &type = "sec");

    static TimeStamp fromDate(const std::string date); //2017-09-28 05:22:52.884694
    static TimeStamp fromSecnSec(const std::string time);

    static TimeStamp fromnSec(const std::string time);

    friend std::ostream &operator<<(std::ostream &out, const TimeStamp &rhs);

    double get_double() const;

    int64_t get_int64() const;

    uint32_t get_sec() const;

    uint32_t get_nsec() const;

    TimeType get_timetype() const { return time_type_; }

private:
    int64_t time_stamp_;
    TimeType time_type_;

};

