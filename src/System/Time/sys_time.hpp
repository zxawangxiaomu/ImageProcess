//
// Created by zer0 on 19-2-2.
//

#ifndef IMAGEPROCESS_SYS_TIME_HPP
#define IMAGEPROCESS_SYS_TIME_HPP

#include <chrono>
using namespace std::chrono;

typedef high_resolution_clock::time_point TimePoint;
typedef microseconds Duration;

TimePoint Now()
{
    return high_resolution_clock::now();
}

Duration operator - (const TimePoint& T1, const TimePoint& T2)
{
    Duration cost = duration_cast<Duration>(T1 - T2);
    return cost;
}

#endif //IMAGEPROCESS_SYS_TIME_HPP
