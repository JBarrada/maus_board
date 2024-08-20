#ifndef __MAUS_BOARD_H__
#define __MAUS_BOARD_H__

// Basic driver for my custom Maus board
// Written by Justin Barrada

#include <stdint.h>
#include <stdio.h>
#include <vector>
#include <unistd.h>
#include <fcntl.h>
#include <fstream>
#include <termios.h>
#include <thread>
#include <string.h>
#include <cmath>

#include "timestamp.h"

#define DEFAULT_SERIAL_MAUS_BOARD "/dev/ttyAMA2"

class MausBoard {
public:
    struct __attribute__((__packed__)) ImuData {
        uint64_t timestamp; // Nanoseconds since epoch

        // Quaternion parameters
        float qX;
        float qY;
        float qZ;
        float qW;
        
        // Gyro 
        // TODO get to RADIANS/SEC units
        float gyroX;
        float gyroY;
        float gyroZ;

        // Accel
        // 8192 per G
        float accelX;
        float accelY;
        float accelZ;

        // Returns the yaw in radians from the quaternion
        float getYawRadians() const;

        // Builds an ImuData object from a FIFO packet
        static ImuData fromFifoPacket(const uint8_t* fifoPacket, const uint8_t fifoPacketSize);

        // Writes the ImuData object to a stream
        void writeBytes(std::ofstream& of) const;

        // Builds an ImuData object from raw bytes
        static ImuData fromBytes(const char* bytes);

        static size_t sizeBytes() { return 8 + 16 + 12 + 12; }
    }; // 48 bytes

    struct __attribute__((__packed__)) EscTelemetry {
        uint64_t timestamp; // Nanoseconds since epoch
        uint8_t temperature; // Celsius
        uint16_t voltage; // Volts * 100 (100 = 1V)
        uint16_t current; // Amps * 100 (100 = 1A)
        uint16_t consumption; // mAh
        uint16_t ERPM; // Electrical RPM / 100 (100 = 10000 ERPM)

        static EscTelemetry fromRawData(const uint8_t* rawData, const uint8_t rawDataSize);

        float getTemperature() const { return temperature; }
        float getVoltage() const { return voltage / 100.0f; }
		float getCurrent() const { return current / 100.0f; }
        float getConsumption() const { return consumption; }
		float getERPM() const { return ERPM; }
    }; // 18 bytes

private:
    static const uint8_t crcTable[256];

    uint8_t calCRC8(const uint8_t *p, const size_t len);

    const uint8_t magicBytesMessage[2] = {0x12, 0x34};

    enum CommandIds : uint8_t {
        CMD_ECHO_REQUEST = 0xFF,
        CMD_ECHO_RESPONSE = 0xFE,
        CMD_SET_RGB = 0x01,
        CMD_SET_SERVOS = 0x02,
        CMD_IMU_DUMP = 0x03,
        CMD_ENCODER_DUMP = 0x04, // UNIMPLEMENTED
        CMD_ESC_TELEMETRY_DUMP = 0x05
    };

    // Message buffer
    static const size_t MAX_MESSAGE_SIZE = 5 + 255;
    uint8_t messageBuffer[MAX_MESSAGE_SIZE];
    uint16_t messageBufferPos = 0;

    // Callbacks
    void (*imuDataCallback)(const ImuData&);
    void (*escTelemetryCallback)(const EscTelemetry&);

    // UART related members
    static const size_t UART_BUFFER_SIZE = 256;
    bool readingUart = false;
    int uartFileStream = -1;
    std::thread readingThread;

    // Parses a successfully received payload
    void parsePayload(const uint8_t* payload, const uint8_t payloadSize);

    // Try to parse a message from the message buffer
    void parseMessageBuffer();

    // Reads the UART continuously
    void readLoop();

    // Send a message internally
    void sendMessage(const uint8_t* payload, const uint8_t payloadSize);

public:
    // Public debug callback (DEPRECATED)
    void (*echoResponseCallback)(const uint8_t* payload, const uint8_t payloadSize) = nullptr;

    MausBoard(void (*imuDataCallback)(const ImuData&), void (*escTelemetryCallback)(const EscTelemetry&)) : imuDataCallback(imuDataCallback), escTelemetryCallback(escTelemetryCallback) {}
    ~MausBoard() { stopReading(); }

    bool startReading();
    bool stopReading();

    // Send servo and throttle values in servo pulse microseconds (1000 = -100%, 1500 = 0%, 2000 = 100%)
    void sendSetServos(const uint16_t steering, const uint16_t throttle);

    // Send a std::vector of uint32_t colors (max of 16) the PCB has 2 LEDs onboard
    void sentSetRGB(const std::vector<uint32_t>& colors);

    // Comms debug (DEPRECATED)
    void sendEcho(const uint8_t* data, const uint8_t dataSize);
};

#endif