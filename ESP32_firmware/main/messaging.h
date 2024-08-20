#ifndef __MESSAGING_H__
#define __MESSAGING_H__

// Custom messaging protocol between the ESP32 and RPi. Allows for dynamic payload sizes and has a checksum for data security.

#include <stddef.h>
#include <stdint.h>

#define MAX_MESSAGE_SIZE (5 + 255)

// -- MESSAGING --
// Message format
// 0x12 - magic 
// 0x34 - magic
// 0x?? - message ID (wraps)
// 0x?? - payload size
// 0x?? - CRC8 of payload
// 0x?? - payload
// 0x?? - payload ...

// Acknowledge format (UNIMPLEMENTED. NOT NEEDED)
// 0x56 - magic
// 0x78 - magic
// 0x?? - message id of acknowledged message
// 0x?? - CRC8 of magic and message id

// CRC8 poly: 0x31
static const uint8_t crc8_table[] = {
  0x00, 0x31, 0x62, 0x53, 0xc4, 0xf5, 0xa6, 0x97, 0xb9, 0x88, 0xdb, 0xea, 0x7d,
  0x4c, 0x1f, 0x2e, 0x43, 0x72, 0x21, 0x10, 0x87, 0xb6, 0xe5, 0xd4, 0xfa, 0xcb,
  0x98, 0xa9, 0x3e, 0x0f, 0x5c, 0x6d, 0x86, 0xb7, 0xe4, 0xd5, 0x42, 0x73, 0x20,
  0x11, 0x3f, 0x0e, 0x5d, 0x6c, 0xfb, 0xca, 0x99, 0xa8, 0xc5, 0xf4, 0xa7, 0x96,
  0x01, 0x30, 0x63, 0x52, 0x7c, 0x4d, 0x1e, 0x2f, 0xb8, 0x89, 0xda, 0xeb, 0x3d,
  0x0c, 0x5f, 0x6e, 0xf9, 0xc8, 0x9b, 0xaa, 0x84, 0xb5, 0xe6, 0xd7, 0x40, 0x71,
  0x22, 0x13, 0x7e, 0x4f, 0x1c, 0x2d, 0xba, 0x8b, 0xd8, 0xe9, 0xc7, 0xf6, 0xa5,
  0x94, 0x03, 0x32, 0x61, 0x50, 0xbb, 0x8a, 0xd9, 0xe8, 0x7f, 0x4e, 0x1d, 0x2c,
  0x02, 0x33, 0x60, 0x51, 0xc6, 0xf7, 0xa4, 0x95, 0xf8, 0xc9, 0x9a, 0xab, 0x3c,
  0x0d, 0x5e, 0x6f, 0x41, 0x70, 0x23, 0x12, 0x85, 0xb4, 0xe7, 0xd6, 0x7a, 0x4b,
  0x18, 0x29, 0xbe, 0x8f, 0xdc, 0xed, 0xc3, 0xf2, 0xa1, 0x90, 0x07, 0x36, 0x65,
  0x54, 0x39, 0x08, 0x5b, 0x6a, 0xfd, 0xcc, 0x9f, 0xae, 0x80, 0xb1, 0xe2, 0xd3,
  0x44, 0x75, 0x26, 0x17, 0xfc, 0xcd, 0x9e, 0xaf, 0x38, 0x09, 0x5a, 0x6b, 0x45,
  0x74, 0x27, 0x16, 0x81, 0xb0, 0xe3, 0xd2, 0xbf, 0x8e, 0xdd, 0xec, 0x7b, 0x4a,
  0x19, 0x28, 0x06, 0x37, 0x64, 0x55, 0xc2, 0xf3, 0xa0, 0x91, 0x47, 0x76, 0x25,
  0x14, 0x83, 0xb2, 0xe1, 0xd0, 0xfe, 0xcf, 0x9c, 0xad, 0x3a, 0x0b, 0x58, 0x69,
  0x04, 0x35, 0x66, 0x57, 0xc0, 0xf1, 0xa2, 0x93, 0xbd, 0x8c, 0xdf, 0xee, 0x79,
  0x48, 0x1b, 0x2a, 0xc1, 0xf0, 0xa3, 0x92, 0x05, 0x34, 0x67, 0x56, 0x78, 0x49,
  0x1a, 0x2b, 0xbc, 0x8d, 0xde, 0xef, 0x82, 0xb3, 0xe0, 0xd1, 0x46, 0x77, 0x24,
  0x15, 0x3b, 0x0a, 0x59, 0x68, 0xff, 0xce, 0x9d, 0xac};

uint8_t crc8(uint8_t crc, const uint8_t *mem, size_t len) {
    const uint8_t *data = mem;
    if (data == NULL)
        return 0xff;
    crc &= 0xff;
    for (size_t i = 0; i < len; i++) 
        crc = crc8_table[crc ^ data[i]];
    return crc;
}

class MessageInterface {
private:
    const uint8_t magicBytesMessage[2] = {0x12, 0x34};
    const uint8_t magicBytesAck[2] = {0x56, 0x78};

    // Circular message buffer, after messages are parsed they are "blanked" out with zeros
    uint8_t messageBuffer[MAX_MESSAGE_SIZE];
    uint16_t messageBufferPos = 0;

    uint8_t currentMessageId = 0;

    // Stream to use
    Stream &serialPort;

    // Message received callback, passes the payload and payload size
    bool (*messageReceivedCallback)(const uint8_t*,const uint16_t);
public:
    MessageInterface(Stream &serialPort, bool (*messageReceivedCallback)(const uint8_t*, const uint16_t)) : serialPort(serialPort), messageReceivedCallback(messageReceivedCallback) {}

    void tryParseMessageBuffer() {
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
            magicPosAck = (magicBytesAck[magicPosAck] == messageBuffer[bufferPos]) ? magicPosAck + 1 : 0;

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
                        const uint8_t payloadCrc8Calc = crc8(0, payload, payloadSize);
                        if (payloadCrc8Calc == payloadCrc8) {
                            // We have a message!
                            const bool result = messageReceivedCallback(payload, payloadSize);                            
                            
                            // TODO send an ack

                            // Zero out the message so we dont parse it again later
                            for (uint16_t messageIndex = 0; messageIndex < (payloadSize + 5); messageIndex++) {
                                const uint16_t zeroBufferPos = (bufferPos + MAX_MESSAGE_SIZE - 1 + messageIndex) % MAX_MESSAGE_SIZE;
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

    void update() {
        if (serialPort.available()) {
            // Keep pushing data into the mesasage buffer
            while (serialPort.available()) {
                messageBuffer[messageBufferPos] = serialPort.read();
                messageBufferPos = (messageBufferPos + 1) % MAX_MESSAGE_SIZE;
            }

            // Check if we can parse out a message
            tryParseMessageBuffer();
        }
    }

    void sendMessage(const uint8_t* payload, const uint8_t payloadSize) {
        // Write the magic bytes
        serialPort.write(magicBytesMessage, 2);

        // Write the message ID
        serialPort.write(currentMessageId);
        currentMessageId = (currentMessageId + 1) % 256;

        // Write the payload size
        serialPort.write(payloadSize);

        // Write the CRC8
        const uint8_t payloadCrc8Calc = crc8(0, payload, payloadSize);
        serialPort.write(payloadCrc8Calc);

        // Write the payload
        serialPort.write(payload, payloadSize);
    }
};

#endif