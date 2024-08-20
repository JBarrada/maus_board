#include "maus_board.h"

const uint8_t MausBoard::crcTable[] = {
    0x00, 0x31, 0x62, 0x53, 0xc4, 0xf5, 0xa6, 0x97, 0xb9, 0x88, 0xdb, 0xea, 0x7d, 0x4c, 0x1f, 0x2e, 
    0x43, 0x72, 0x21, 0x10, 0x87, 0xb6, 0xe5, 0xd4, 0xfa, 0xcb, 0x98, 0xa9, 0x3e, 0x0f, 0x5c, 0x6d, 
    0x86, 0xb7, 0xe4, 0xd5, 0x42, 0x73, 0x20, 0x11, 0x3f, 0x0e, 0x5d, 0x6c, 0xfb, 0xca, 0x99, 0xa8, 
    0xc5, 0xf4, 0xa7, 0x96, 0x01, 0x30, 0x63, 0x52, 0x7c, 0x4d, 0x1e, 0x2f, 0xb8, 0x89, 0xda, 0xeb, 
    0x3d, 0x0c, 0x5f, 0x6e, 0xf9, 0xc8, 0x9b, 0xaa, 0x84, 0xb5, 0xe6, 0xd7, 0x40, 0x71, 0x22, 0x13, 
    0x7e, 0x4f, 0x1c, 0x2d, 0xba, 0x8b, 0xd8, 0xe9, 0xc7, 0xf6, 0xa5, 0x94, 0x03, 0x32, 0x61, 0x50, 
    0xbb, 0x8a, 0xd9, 0xe8, 0x7f, 0x4e, 0x1d, 0x2c, 0x02, 0x33, 0x60, 0x51, 0xc6, 0xf7, 0xa4, 0x95, 
    0xf8, 0xc9, 0x9a, 0xab, 0x3c, 0x0d, 0x5e, 0x6f, 0x41, 0x70, 0x23, 0x12, 0x85, 0xb4, 0xe7, 0xd6, 
    0x7a, 0x4b, 0x18, 0x29, 0xbe, 0x8f, 0xdc, 0xed, 0xc3, 0xf2, 0xa1, 0x90, 0x07, 0x36, 0x65, 0x54, 
    0x39, 0x08, 0x5b, 0x6a, 0xfd, 0xcc, 0x9f, 0xae, 0x80, 0xb1, 0xe2, 0xd3, 0x44, 0x75, 0x26, 0x17, 
    0xfc, 0xcd, 0x9e, 0xaf, 0x38, 0x09, 0x5a, 0x6b, 0x45, 0x74, 0x27, 0x16, 0x81, 0xb0, 0xe3, 0xd2, 
    0xbf, 0x8e, 0xdd, 0xec, 0x7b, 0x4a, 0x19, 0x28, 0x06, 0x37, 0x64, 0x55, 0xc2, 0xf3, 0xa0, 0x91, 
    0x47, 0x76, 0x25, 0x14, 0x83, 0xb2, 0xe1, 0xd0, 0xfe, 0xcf, 0x9c, 0xad, 0x3a, 0x0b, 0x58, 0x69, 
    0x04, 0x35, 0x66, 0x57, 0xc0, 0xf1, 0xa2, 0x93, 0xbd, 0x8c, 0xdf, 0xee, 0x79, 0x48, 0x1b, 0x2a, 
    0xc1, 0xf0, 0xa3, 0x92, 0x05, 0x34, 0x67, 0x56, 0x78, 0x49, 0x1a, 0x2b, 0xbc, 0x8d, 0xde, 0xef, 
    0x82, 0xb3, 0xe0, 0xd1, 0x46, 0x77, 0x24, 0x15, 0x3b, 0x0a, 0x59, 0x68, 0xff, 0xce, 0x9d, 0xac
};

uint8_t MausBoard::calCRC8(const uint8_t *p, const size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++)
        crc = crcTable[(crc ^ p[i]) & 0xff];
    return crc;
}

float MausBoard::ImuData::getYawRadians() const {
    const float siny_cosp = 2 * (qW * qZ + qX * qY);
    const float cosy_cosp = 1 - 2 * (qY * qY + qZ * qZ);
    return std::atan2(siny_cosp, cosy_cosp);
}

MausBoard::ImuData MausBoard::ImuData::fromFifoPacket(const uint8_t* fifoPacket, const uint8_t fifoPacketSize) {
    ImuData imuData;

    if (fifoPacketSize >= 42) {
        // Parse quaternion
        imuData.qW = (float)((fifoPacket[0] << 8) | fifoPacket[1]) / 16384.0f;
        imuData.qX = (float)((fifoPacket[4] << 8) | fifoPacket[5]) / 16384.0f;
        imuData.qY = (float)((fifoPacket[8] << 8) | fifoPacket[9]) / 16384.0f;
        imuData.qZ = (float)((fifoPacket[12] << 8) | fifoPacket[13]) / 16384.0f;

        // Parse gryo
        imuData.gyroX = (float)((fifoPacket[16] << 8) | fifoPacket[17]);
        imuData.gyroY = (float)((fifoPacket[20] << 8) | fifoPacket[21]);
        imuData.gyroZ = (float)((fifoPacket[24] << 8) | fifoPacket[25]);

        // Parse accel
        imuData.accelX = (float)((fifoPacket[28] << 8) | fifoPacket[29]);
        imuData.accelY = (float)((fifoPacket[32] << 8) | fifoPacket[33]);
        imuData.accelZ = (float)((fifoPacket[36] << 8) | fifoPacket[37]);
    }

    return imuData;
}

void MausBoard::ImuData::writeBytes(std::ofstream& of) const {
    of.write((char*)&timestamp, sizeof(uint64_t));
    of.write((char*)&qX, sizeof(float));
    of.write((char*)&qY, sizeof(float));
    of.write((char*)&qZ, sizeof(float));
    of.write((char*)&qW, sizeof(float));
    of.write((char*)&gyroX, sizeof(float));
    of.write((char*)&gyroY, sizeof(float));
    of.write((char*)&gyroZ, sizeof(float));
    of.write((char*)&accelX, sizeof(float));
    of.write((char*)&accelY, sizeof(float));
    of.write((char*)&accelZ, sizeof(float));
}

MausBoard::ImuData MausBoard::ImuData::fromBytes(const char* bytes) {
    ImuData parsedImuData;
    parsedImuData.timestamp = *(uint64_t*)&bytes[0];
    parsedImuData.qX = *(float*)&bytes[sizeof(uint64_t)];
    parsedImuData.qY = *(float*)&bytes[sizeof(uint64_t) + 4];
    parsedImuData.qZ = *(float*)&bytes[sizeof(uint64_t) + 8];
    parsedImuData.qW = *(float*)&bytes[sizeof(uint64_t) + 12];
    parsedImuData.gyroX = *(float*)&bytes[sizeof(uint64_t) + 16];
    parsedImuData.gyroY = *(float*)&bytes[sizeof(uint64_t) + 20];
    parsedImuData.gyroZ = *(float*)&bytes[sizeof(uint64_t) + 24];
    parsedImuData.accelX = *(float*)&bytes[sizeof(uint64_t) + 28];
    parsedImuData.accelY = *(float*)&bytes[sizeof(uint64_t) + 32];
    parsedImuData.accelZ = *(float*)&bytes[sizeof(uint64_t) + 36];
    return parsedImuData;
}

MausBoard::EscTelemetry MausBoard::EscTelemetry::fromRawData(const uint8_t* rawData, const uint8_t rawDataSize) {
    EscTelemetry escTelemetry;

    if (rawDataSize >= 10) {
        escTelemetry.temperature = rawData[0];
        escTelemetry.voltage = ((uint16_t)(rawData[1]) << 8) | rawData[2];
        escTelemetry.current = ((uint16_t)(rawData[3]) << 8) | rawData[4];
        escTelemetry.consumption = ((uint16_t)(rawData[5]) << 8) | rawData[6];
        escTelemetry.ERPM = ((uint16_t)(rawData[7]) << 8) | rawData[8];
        // CRC not parsed
    }

    return escTelemetry;
}

void MausBoard::parsePayload(const uint8_t* payload, const uint8_t payloadSize) {
    if (payloadSize >= 1) {
        const uint8_t commandId = payload[0];

        if (commandId == CommandIds::CMD_ECHO_REQUEST) {
            // Send the payload back
            uint8_t responsePayload[payloadSize];
            memcpy(responsePayload, payload, payloadSize);
            responsePayload[0] = CommandIds::CMD_ECHO_RESPONSE;
            sendMessage(responsePayload, payloadSize);
        }

        if (commandId == CommandIds::CMD_ECHO_RESPONSE) {
            if (echoResponseCallback)
                echoResponseCallback(payload, payloadSize);
        }

        if (commandId == CommandIds::CMD_IMU_DUMP) {
            // Parse the data and call the callback
            ImuData imuData = ImuData::fromFifoPacket(&payload[1], payloadSize - 1);
            imuData.timestamp = TimeStamp::get();
            imuDataCallback(imuData);
        }

        if (commandId == CommandIds::CMD_ESC_TELEMETRY_DUMP) {
            // Parse the data and call the callback
            EscTelemetry escTelemetry = EscTelemetry::fromRawData(&payload[1], payloadSize - 1);
            escTelemetry.timestamp = TimeStamp::get();
            escTelemetryCallback(escTelemetry);
        }
    }
}

void MausBoard::parseMessageBuffer() {
    // Offset relative to messageBufferPos
    uint16_t offset = 0;

    // Positions within magic bytes
    uint8_t magicPosMessage = 0;
    uint8_t magicPosAck = 0;

    while (offset < MAX_MESSAGE_SIZE) {
        // Get the actual possition in messageBufferPos
        const uint16_t bufferPos = (offset + messageBufferPos) % MAX_MESSAGE_SIZE;
        offset++;

        magicPosMessage = (magicBytesMessage[magicPosMessage] == messageBuffer[bufferPos]) ? magicPosMessage + 1 : 0;

        // Check if we have magic bytes for a regular message
        if (magicPosMessage == 2) {
            magicPosMessage = 0;
            // 2 byte magic number
            // 1 byte message ID
            // 1 byte payload size
            // 1 byte payload CRC8
            
            // Check if there is enough room to read up to the CRC
            if (offset + 3 <= MAX_MESSAGE_SIZE) {
                const uint8_t messageId = messageBuffer[(bufferPos + 1) % MAX_MESSAGE_SIZE];
                const uint8_t payloadSize = messageBuffer[(bufferPos + 2) % MAX_MESSAGE_SIZE];
                const uint8_t payloadCrc8 = messageBuffer[(bufferPos + 3) % MAX_MESSAGE_SIZE];

                // Check if there is enough room to read the whole message
                if ((offset + 3 + payloadSize) <= MAX_MESSAGE_SIZE) {
                    // Extract the payload
                    uint8_t payload[255];
                    for (uint16_t payloadIndex = 0; payloadIndex < payloadSize; payloadIndex++) {
                        const uint16_t payloadBufferPos = (bufferPos + 4 + payloadIndex) % MAX_MESSAGE_SIZE;
                        payload[payloadIndex] = messageBuffer[payloadBufferPos];
                    }

                    // Check the CRC8
                    const uint8_t payloadCrc8Calc = calCRC8(payload, payloadSize);
                    if (payloadCrc8Calc == payloadCrc8) {
                        // We have a message!
                        // TODO do this in a thread because parsePayload can block for a while (it will screw up timestamping)
                        parsePayload(payload, payloadSize);
                        
                        // TODO send an ack

                        // Zero out the message so we dont parse it again later
                        for (uint16_t messageIndex = 0; messageIndex < (payloadSize + 5); messageIndex++) {
                            const uint16_t zeroBufferPos = (bufferPos - 1 + messageIndex) % MAX_MESSAGE_SIZE;
                            messageBuffer[zeroBufferPos] = 0x00;
                        }

                        // Advance the offset
                        offset += (payloadSize + 4);
                    }
                }
            }
        }
    }
}

void MausBoard::readLoop() {
    // Read forever
    if (uartFileStream != -1) {
        uint8_t uartBuffer[UART_BUFFER_SIZE];
        while (readingUart) {
            // Read and parse data
            int len = read(uartFileStream, uartBuffer, UART_BUFFER_SIZE);
            if (len > 0) {
                for (int16_t i = 0; i < len; i++) {
                    messageBuffer[messageBufferPos] = uartBuffer[i];
                    messageBufferPos = (messageBufferPos + 1) % MAX_MESSAGE_SIZE;
                }

                parseMessageBuffer();
            }
        }
    }

    // Close the UART
    close(uartFileStream);
}

void MausBoard::sendMessage(const uint8_t* payload, const uint8_t payloadSize) {
    if (uartFileStream != -1) {
        // Build header
        uint8_t header[5];
        memcpy(header, magicBytesMessage, 2);
        header[2] = 0;
        header[3] = payloadSize;
        header[4] = calCRC8(payload, payloadSize);

        // Write the header and payload
        write(uartFileStream, header, 5);
        write(uartFileStream, payload, payloadSize);
    }
}

bool MausBoard::startReading() {
    if (!readingUart) {
        // Open the UART
        uartFileStream = open(DEFAULT_SERIAL_MAUS_BOARD, O_RDWR);
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
        readingThread = std::thread(&MausBoard::readLoop, this);

        return true;
    } else {
        printf("Cannot start reading. UART is already being read\n");
        return false;
    }
}

bool MausBoard::stopReading() {
    if (readingUart) {
        readingUart = false;
        readingThread.join();
        uartFileStream = -1;

        return true;
    }
    return false;
}

void MausBoard::sendSetServos(const uint16_t steering, const uint16_t throttle) {
    // Build the payload
    uint8_t payload[1 + 4];
    payload[0] = CommandIds::CMD_SET_SERVOS;
    memcpy(&payload[1], &steering, 2);
    memcpy(&payload[3], &throttle, 2);

    // Send it
    sendMessage(payload, 1 + 4);
}

void MausBoard::sentSetRGB(const std::vector<uint32_t>& colors) {
    // Build the payload 
    const uint8_t payloadSize = 1 + (colors.size() * 4);
    uint8_t payload[payloadSize];
    payload[0] = CommandIds::CMD_SET_RGB;
    for (size_t i = 0; i < colors.size(); i++) {
        const uint8_t payloadIndex = 1 + (i * 4);
        memcpy(&payload[payloadIndex], &colors[i], 4);
    }

    // Send it
    sendMessage(payload, payloadSize);
}

void MausBoard::sendEcho(const uint8_t* data, const uint8_t dataSize) {
    // Build the payload
    const uint8_t payloadSize = 1 + dataSize;
    uint8_t payload[payloadSize];
    payload[0] = CommandIds::CMD_ECHO_REQUEST;
    memcpy(&payload[1], data, dataSize);

    // Send it
    sendMessage(payload, payloadSize);
}