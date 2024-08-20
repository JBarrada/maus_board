#ifndef __TIMESTAMP_H__
#define __TIMESTAMP_H__

#include <stdint.h>
#include <chrono>

#define NSECS_TO_SECS 1000000000
#define NSECS_TO_MSECS 1000000
#define NSECS_TO_USECS 1000

class TimeStamp {
public:
    // Returns a nanoseconds timestamp since epoch
    static uint64_t get() {
        return std::chrono::high_resolution_clock::now().time_since_epoch().count();
    }
};

#endif
