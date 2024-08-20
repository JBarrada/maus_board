#ifndef __ESC_TELEMETRY_H__
#define __ESC_TELEMETRY_H__

// Simple class for reading KISS ESC type telemetry messages

#include <stddef.h>
#include <stdint.h>

#define ESC_TELEMETRY_BUF_SIZE 10

class ESCTelemetry {
private:
    // Circular buffer for reading in bytes from the serial interface
    uint8_t escTelemetryBuf[ESC_TELEMETRY_BUF_SIZE];
    uint8_t escTelemetryBufPos = 0;

    // Stream to use
    Stream &serialPort;

public:
    // Successfully parsed telemetry packets will get memcpy'd to here
    uint8_t buffer[ESC_TELEMETRY_BUF_SIZE];

    ESCTelemetry(Stream &serialPort) : serialPort(serialPort) {}

    // KISS ESC telemetry CRC calculation
    uint8_t getCRC8(uint8_t *data, uint8_t length) {
        uint8_t crc = 0;
        for (uint8_t i = 0; i < length; i++) {
            uint8_t crcU = data[i];
            crcU ^= crc;
            for (uint8_t j = 0; j < 8; j++)
                crcU = (crcU & 0x80) ? 0x7 ^ (crcU << 1) : (crcU << 1);
            crc = crcU;
        }
        return crc;
    }

    // Main update function. Run this as often as possible.
    // Returns true if a packet was successfully parsed. Packet will get memcpy'd to the public 'buffer'
    bool update() {
        bool packetReceived = false;

        while (serialPort.available()) {
            // Read in the next byte from the serial interface into the circular buffer
            escTelemetryBuf[escTelemetryBufPos] = serialPort.read();
            
            // Unwrap the circular buffer into tempBuffer so that byte 0 of the telemetry packet actually starts at intex 0
            uint8_t tempBuffer[ESC_TELEMETRY_BUF_SIZE];
            for (uint8_t i = 0; i < ESC_TELEMETRY_BUF_SIZE; i++)
              tempBuffer[i] = escTelemetryBuf[(escTelemetryBufPos + i + 1) % ESC_TELEMETRY_BUF_SIZE];

            // Check tempBuffer CRC
            const uint8_t calculatedCRC = getCRC8(tempBuffer, ESC_TELEMETRY_BUF_SIZE - 1);
            if (calculatedCRC == tempBuffer[ESC_TELEMETRY_BUF_SIZE - 1]) {
              // Copy the private buffer to a public buffer
              memcpy(buffer, tempBuffer, ESC_TELEMETRY_BUF_SIZE);
              packetReceived = true;
            }

            // Increment escTelemetryBufPos
            escTelemetryBufPos = (escTelemetryBufPos + 1) % ESC_TELEMETRY_BUF_SIZE;            
        }

        return packetReceived;
    }
};

#endif