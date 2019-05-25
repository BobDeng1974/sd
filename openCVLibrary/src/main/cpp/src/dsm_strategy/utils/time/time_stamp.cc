/*
 * TimeStamp.cpp
 *
 *  Created on: Aug 10, 2016
 *      Author: liujj
 */
#include <cmath>
#include <sstream>
#include <string.h>
#include <cassert>
#include "time_stamp.h"


TimeStamp::TimeStamp() :
        time_stamp_(0),
        time_type_(Nanosecond) {

}

TimeStamp::TimeStamp(int64_t time_stamp, TimeType time_type) :
        time_stamp_(time_stamp),
        time_type_(time_type) {
}

TimeStamp::~TimeStamp() {
}

TimeStamp TimeStamp::operator-(const TimeStamp &rhs) const {
    assert(time_type_ == rhs.time_type_);
    return TimeStamp(time_stamp_ - rhs.get_int64(), time_type_);
}

TimeStamp TimeStamp::operator+(const TimeStamp &rhs) const {
    assert(time_type_ == rhs.time_type_);
    return TimeStamp(time_stamp_ + rhs.get_int64(), time_type_);
}

TimeStamp TimeStamp::operator/(const int64_t &s) const {
    return TimeStamp(time_stamp_ / s, time_type_);
}

TimeStamp &TimeStamp::operator+=(const TimeStamp &rhs) {
    assert(time_type_ == rhs.time_type_);
    time_stamp_ += rhs.get_int64();
    return *this;
}

TimeStamp &TimeStamp::operator-=(const TimeStamp &rhs) {
    assert(time_type_ == rhs.time_type_);
    time_stamp_ -= rhs.get_int64();
    return *this;
}

TimeStamp &TimeStamp::operator/=(const int64_t &s) {
    time_stamp_ /= s;
    return *this;
}

bool TimeStamp::operator==(const TimeStamp &rhs) const {
    assert(time_type_ == rhs.time_type_);
    return time_stamp_ == rhs.get_int64();
}

bool TimeStamp::operator!=(const TimeStamp &rhs) const {
    assert(time_type_ == rhs.time_type_);
    return time_stamp_ != rhs.get_int64();
}

bool TimeStamp::operator>(const TimeStamp &rhs) const {
    assert(time_type_ == rhs.time_type_);
    return time_stamp_ > rhs.get_int64();
}

bool TimeStamp::operator<(const TimeStamp &rhs) const {
    assert(time_type_ == rhs.time_type_);
    return time_stamp_ < rhs.get_int64();
}

bool TimeStamp::operator>=(const TimeStamp &rhs) const {
    assert(time_type_ == rhs.time_type_);
    return time_stamp_ >= rhs.get_int64();
}

bool TimeStamp::operator<=(const TimeStamp &rhs) const {
    assert(time_type_ == rhs.time_type_);
    return time_stamp_ <= rhs.get_int64();
}

int64_t &TimeStamp::operator()() {
    return time_stamp_;
}

TimeStamp TimeStamp::abs() const {
    return TimeStamp(std::abs(get_int64()), time_type_);
}

double TimeStamp::toSec() const {
    return time_stamp_ * double(TimeScale(time_type_));
}

TimeStamp TimeStamp::now(TimeType time_type) {
    int64_t t;
    auto now_t = std::chrono::system_clock::now().time_since_epoch();
    switch (time_type) {
        case Second:
            t = now_t / std::chrono::seconds(1);
            break;
        case Millisecond:
            t = now_t / std::chrono::milliseconds(1);
            break;
        case Microsecond:
            t = now_t / std::chrono::microseconds(1);
            break;
        case Nanosecond:
            t = now_t / std::chrono::nanoseconds(1);
            break;
        default:
            t = now_t / std::chrono::nanoseconds(1);
            time_type = Nanosecond;
            break;
    }

    return TimeStamp(t, time_type);
}

TimeStamp TimeStamp::fromString(const std::string time, const std::string &type) {
    if (type == "sec")
        return fromnSec(time);
    else if (type == "date")
        return fromDate(time);
    else if (type == "secnsec")
        return fromSecnSec(time);

    return TimeStamp(0);
}

TimeStamp TimeStamp::fromDate(const std::string date) {
    struct tm stm;
    int ims;
    const char *str_time = date.data();

    memset(&stm, 0, sizeof(stm));
    stm.tm_year = atoi(str_time) - 1900;
    stm.tm_mon = atoi(str_time + 5) - 1;
    stm.tm_mday = atoi(str_time + 8);
    stm.tm_hour = atoi(str_time + 11);
    stm.tm_min = atoi(str_time + 14);
    stm.tm_sec = atoi(str_time + 17);
    ims = atoi(str_time + 20);

    return TimeStamp(int64_t(mktime(&stm) * 1e9) + ims * 1000);
}

TimeStamp TimeStamp::fromSecnSec(const std::string time) {
    size_t idx = time.find_first_of('.');
    if (std::string::npos == idx) {
        return fromnSec(time);
    } else {
        int64_t second, nsecond, time_stamp;
        std::stringstream ss(time.substr(0, idx));
        ss >> second;

        std::string nsecond_str = time.substr(idx + 1, time.length());
        ss.str("");
        ss.clear();
        ss << nsecond_str;
        ss >> nsecond;

        time_stamp = second * 1e9;
        time_stamp += int64_t(nsecond * pow(10.0, 9 - nsecond_str.length()));
        return TimeStamp(time_stamp);
    }
}

TimeStamp TimeStamp::fromnSec(const std::string time) {
    std::stringstream ss(time);
    int64_t time_stamp;
    ss >> time_stamp;
    return TimeStamp(time_stamp);
}


std::ostream &operator<<(std::ostream &out, const TimeStamp &rhs) {
    return out << rhs.get_int64();
}


double TimeStamp::get_double() const {
    double t = (double) time_stamp_;
    return t;
}

int64_t TimeStamp::get_int64() const {
    return time_stamp_;
}

uint32_t TimeStamp::get_sec() const {
    return uint32_t(time_stamp_ * TimeScale(time_type_));
}

uint32_t TimeStamp::get_nsec() const {
    return uint32_t(time_stamp_ % int64_t(1.0 / TimeScale(time_type_)) * TimeScale(time_type_) * 1e9);
}
