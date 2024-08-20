#include "fhl_ld19.h"

const uint8_t LD19::crcTable[] = {
    0x00, 0x4D, 0x9A, 0xD7, 0x79, 0x34, 0xE3, 0xAE, 0xF2, 0xBF, 0x68, 0x25, 0x8B, 0xC6, 0x11, 0x5C,
    0xA9, 0xE4, 0x33, 0x7E, 0xD0, 0x9D, 0x4A, 0x07, 0x5B, 0x16, 0xC1, 0x8C, 0x22, 0x6F, 0xB8, 0xF5,
    0x1F, 0x52, 0x85, 0xC8, 0x66, 0x2B, 0xFC, 0xB1, 0xED, 0xA0, 0x77, 0x3A, 0x94, 0xD9, 0x0E, 0x43,
    0xB6, 0xFB, 0x2C, 0x61, 0xCF, 0x82, 0x55, 0x18, 0x44, 0x09, 0xDE, 0x93, 0x3D, 0x70, 0xA7, 0xEA,
    0x3E, 0x73, 0xA4, 0xE9, 0x47, 0x0A, 0xDD, 0x90, 0xCC, 0x81, 0x56, 0x1B, 0xB5, 0xF8, 0x2F, 0x62,
    0x97, 0xDA, 0x0D, 0x40, 0xEE, 0xA3, 0x74, 0x39, 0x65, 0x28, 0xFF, 0xB2, 0x1C, 0x51, 0x86, 0xCB,
    0x21, 0x6C, 0xBB, 0xF6, 0x58, 0x15, 0xC2, 0x8F, 0xD3, 0x9E, 0x49, 0x04, 0xAA, 0xE7, 0x30, 0x7D,
    0x88, 0xC5, 0x12, 0x5F, 0xF1, 0xBC, 0x6B, 0x26, 0x7A, 0x37, 0xE0, 0xAD, 0x03, 0x4E, 0x99, 0xD4,
    0x7C, 0x31, 0xE6, 0xAB, 0x05, 0x48, 0x9F, 0xD2, 0x8E, 0xC3, 0x14, 0x59, 0xF7, 0xBA, 0x6D, 0x20,
    0xD5, 0x98, 0x4F, 0x02, 0xAC, 0xE1, 0x36, 0x7B, 0x27, 0x6A, 0xBD, 0xF0, 0x5E, 0x13, 0xC4, 0x89,
    0x63, 0x2E, 0xF9, 0xB4, 0x1A, 0x57, 0x80, 0xCD, 0x91, 0xDC, 0x0B, 0x46, 0xE8, 0xA5, 0x72, 0x3F,
    0xCA, 0x87, 0x50, 0x1D, 0xB3, 0xFE, 0x29, 0x64, 0x38, 0x75, 0xA2, 0xEF, 0x41, 0x0C, 0xDB, 0x96,
    0x42, 0x0F, 0xD8, 0x95, 0x3B, 0x76, 0xA1, 0xEC, 0xB0, 0xFD, 0x2A, 0x67, 0xC9, 0x84, 0x53, 0x1E,
    0xEB, 0xA6, 0x71, 0x3C, 0x92, 0xDF, 0x08, 0x45, 0x19, 0x54, 0x83, 0xCE, 0x60, 0x2D, 0xFA, 0xB7,
    0x5D, 0x10, 0xC7, 0x8A, 0x24, 0x69, 0xBE, 0xF3, 0xAF, 0xE2, 0x35, 0x78, 0xD6, 0x9B, 0x4C, 0x01,
    0xF4, 0xB9, 0x6E, 0x23, 0x8D, 0xC0, 0x17, 0x5A, 0x06, 0x4B, 0x9C, 0xD1, 0x7F, 0x32, 0xE5, 0xA8
};

uint8_t LD19::calCRC8(const uint8_t *p, const size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++)
        crc = crcTable[(crc ^ p[i]) & 0xff];
    return crc;
}

void LD19::parse(uint8_t *data, const size_t len) {
    // Construct the current datablock including the remainder
    const size_t bufferLen = len + dataRemainderLen;
    uint8_t buffer[bufferLen];
    memcpy(buffer, dataRemainder, dataRemainderLen);
    memcpy(buffer + dataRemainderLen, data, len);

    // Parse RawFrames out of buffer
    size_t dataRemainderBegin = 0;
    if (bufferLen >= sizeof(RawFrame)) {
        size_t bufferPos = 0;
        while (bufferPos < (bufferLen - sizeof(RawFrame))) {
            RawFrame* currentFrame = reinterpret_cast<RawFrame*>(&buffer[bufferPos]);
            
            // Frame header will always be 0x54, and at the time of writing, the version will be 0x2C
            if (currentFrame->header == 0x54 && currentFrame->verLen == 0x2C) {
                // Check the CRC
                uint8_t calculatedCrc8 = calCRC8(&buffer[bufferPos], sizeof(RawFrame) - 1);
                if (calculatedCrc8 == currentFrame->crc8) {
                    // We have a good frame! Get the timestamp
                    const uint64_t timestamp = TimeStamp::get();
                    
                    // Sometimes we can get frames that wrap back around to 0 degrees, add 36000 to the end angle 
                    if (currentFrame->endAngle < currentFrame->startAngle) 
                        currentFrame->endAngle += 36000;                

                    float angleStep = (float)(currentFrame->endAngle - currentFrame->startAngle) / (float)(POINTS_PER_FRAME - 1);
                    for (uint8_t pointIndex = 0; pointIndex < POINTS_PER_FRAME; pointIndex++) {
                        LidarPoint point;
                        point.distance = currentFrame->points[pointIndex].distance;
                        point.intensity = currentFrame->points[pointIndex].intensity;

                        // Caclculate the angle for the point
                        point.angle = (currentFrame->startAngle + (uint16_t)(angleStep * pointIndex)) % 36000;

                        // Calculate the timestamp for the point (assume the last point is from the current timestamp)
                        double timestampOffsetSecs = ((double)(angleStep * ((POINTS_PER_FRAME - 1) - pointIndex)) / 100.0) / (double)(currentFrame->speed);
                        point.timestamp = timestamp - (uint64_t)(timestampOffsetSecs * 1000000000);

                        // Collect point
                        pointBuffer.push_back(point);
                    }
                    
                    // Advance bufferPos
                    bufferPos += sizeof(RawFrame);

                    // Check if we couldn't parse any bytes
                    int16_t unparsedBytes = (bufferPos - dataRemainderBegin) - sizeof(RawFrame);
                    if (unparsedBytes > 0) {
                        printf("Unable to parse %d bytes\n", unparsedBytes);
                    }
                    dataRemainderBegin = bufferPos;                

                    continue;
                } else {
                    printf("CRC fail: calculated 0x%02x, actual 0x%02x\n", calculatedCrc8, currentFrame->crc8);
                }
            }
            // If we couldn't parse a frame, advance until we see the start of a frame
            bufferPos++;        
        }
    }

    // Update new dataRemainder
    dataRemainderLen = bufferLen - dataRemainderBegin;
    if (dataRemainderLen > DATA_REMAINDER_SIZE) {
        int16_t unparsedBytes = dataRemainderLen - DATA_REMAINDER_SIZE;
        printf("Unable to parse %d bytes. Remainder too large.\n", unparsedBytes);
        dataRemainderLen = DATA_REMAINDER_SIZE;
    }
    if (dataRemainderLen > 0) {
        memcpy(dataRemainder, buffer + dataRemainderBegin, dataRemainderLen);
    }

    // Check pointBuffer for a full scan
    size_t lastScanStartIndex = 0;
    uint16_t lastAngle = 0;
    for (size_t i = 0; i < pointBuffer.size(); i++) {
        // Criteria for a scan (point angle goes from ~360 to 0)
        if (i > 0 && lastAngle > pointBuffer[i].angle) {
            // Call the full scan callback with the points vector
            std::vector<LidarPoint> scanPoints(pointBuffer.begin() + lastScanStartIndex, pointBuffer.begin() + i);
            fullScanCallback(scanPoints);

            lastScanStartIndex = i;
        }
        lastAngle = pointBuffer[i].angle;
    }
    if (lastScanStartIndex > 0) {
        pointBuffer.erase(pointBuffer.begin(), pointBuffer.begin() + lastScanStartIndex);
    }
}

void LD19::readLoop() {
    // Read forever
    if (uartFileStream != -1) {
        uint8_t uartBuffer[UART_BUFFER_SIZE];
        while (readingUart) {
            // Read and parse data
            int len = read(uartFileStream, uartBuffer, UART_BUFFER_SIZE);
            if (len > 0) {
                parse(uartBuffer, len);
            }
        }
    }

    // Close the UART
    close(uartFileStream);
}

bool LD19::startReading() {
    if (!readingUart) {
        // Open the UART
        uartFileStream = open(DEFAULT_SERIAL_FHL_LD19, O_RDONLY);
        if (uartFileStream == -1) {
            printf("Unable to open UART\n");
            return false;
        }

        // Configure the UART (Flags defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html)
        struct termios options;
        tcgetattr(uartFileStream, &options);
        options.c_cflag = B230400 | CS8 | CLOCAL | CREAD; // Set baud rate
        options.c_iflag = IGNPAR;
        options.c_oflag = 0;
        options.c_lflag = 0;
        tcflush(uartFileStream, TCIFLUSH);
        tcsetattr(uartFileStream, TCSANOW, &options);

        readingUart = true;
        readingThread = std::thread(&LD19::readLoop, this);

        return true;
    } else {
        printf("Cannot start reading. UART is already being read\n");
        return false;
    }
}

bool LD19::stopReading() {
    if (readingUart) {
        readingUart = false;
        readingThread.join();
        uartFileStream = -1;

        return true;
    }
    return false;
}
