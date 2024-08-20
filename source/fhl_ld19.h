#ifndef __FHL_LD19_H__
#define __FHL_LD19_H__

// Basic driver for FHL-LD19 lidar
// Written by Justin Barrada

#include <stdint.h>
#include <stdio.h>
#include <vector>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <thread>
#include <string.h>

#include "timestamp.h"

#define DEFAULT_SERIAL_FHL_LD19 "/dev/serial0"

class LD19 {
public:
    struct __attribute__((__packed__)) LidarPoint {
        uint16_t distance;  // Millimeters
        uint8_t intensity;  // Docs say for an object at 6M, this value should be around 200
        uint16_t angle;     // 0.01 degrees
        uint64_t timestamp; // Nanoseconds since epoch
    };

private:
    static const uint8_t crcTable[256];
    
    struct __attribute__((__packed__)) RawPoint {
        uint16_t distance; // Millimeters
        uint8_t intensity; // Docs say for an object at 6M, this value should be around 200 
    };

    static const uint8_t POINTS_PER_FRAME = 12;
    struct __attribute__((__packed__)) RawFrame {
        uint8_t header;                     // 0x54
        uint8_t verLen;                     // 0x2C
        uint16_t speed;                     // Degrees per second
        uint16_t startAngle;                // 0.01 degrees
        RawPoint points[POINTS_PER_FRAME];
        uint16_t endAngle;                  // 0.01 degrees
        uint16_t timestamp;                 // Milliseconds (max 30000)
        uint8_t crc8;
    };

    // Stores the remainder of bytes that could not be parsed completely
    static const size_t DATA_REMAINDER_SIZE = 1024;
    uint8_t dataRemainder[DATA_REMAINDER_SIZE];
    uint16_t dataRemainderLen = 0;

    uint8_t calCRC8(const uint8_t *p, const size_t len);

    // This callback gets called each time we parse out a full scan worth of points
    void (*fullScanCallback)(std::vector<LidarPoint>);

    // Temporary storage for points until we get a full scan
    std::vector<LidarPoint> pointBuffer;

    // UART related members
    static const size_t UART_BUFFER_SIZE = 256;
    bool readingUart = false;
    int uartFileStream = -1;
    std::thread readingThread;

    // Getting more accurate timestamps for points
    uint64_t timestampReferenceWorld = 0;
    uint64_t timestampReferenceLidar = 0;
    uint64_t timestampLidarFramePrevious = 0;

    // Reads the UART continuously
    void readLoop();

public:
    LD19(void (*fullScanCallback)(std::vector<LidarPoint>)) : fullScanCallback(fullScanCallback) {}
    ~LD19() { stopReading(); }

    void parse(uint8_t *data, const size_t len);

    bool startReading();
    bool stopReading();
};

#endif