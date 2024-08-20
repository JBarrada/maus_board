#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>
#include <HardwareSerial.h>
#include "MPU6050_6Axis_MotionApps20.h"

#include "messaging.h"
#include "esc_telemetry.h"

// -- HARDWARE --
#define PIN_MPU6050_INT 15
#define PIN_STEERING_SERVO 18
#define PIN_THROTTLE_SERVO 19
#define PIN_RGB 13
#define PIN_PI_UART_RX 16
#define PIN_PI_UART_TX 17
#define PIN_ESC_UART_RX 4
#define PIN_ESC_UART_TX 2

#define NUM_LEDS 2

// -- PAYLOAD --
// First byte is command ID
// Remaining bytes are command specific

// -- PAYLOAD COMMANDS --
#define CMD_ECHO_REQUEST 0xFF
#define CMD_ECHO_RESPONSE 0xFE

#define CMD_SET_RGB 0x01
// 0x01 - set rgb       - sent by the pi to set the RGB strip
//                      - payload uint32_t color x 16 leds

#define CMD_SET_SERVOS 0x02
// 0x02 - set servos    - sent by the pi to set the current servo positions
//                      - payload uint16_t steering servo, uint16_t throttle servo (microseconds pulse length)

#define CMD_IMU_DUMP 0x03
// 0x03 - imu dump      - sent to the pi with IMU data
//                      - payload DMP 2.0 default FIFO packet

#define CMD_ENCODER_DUMP 0x04 // UNIMPLEMENTED IN THIS VERSION
// 0x04 - encoder dump  - sent to the pi with encoder data
//                      - payload 2x AS5600 packet

#define CMD_ESC_TELEMETRY_DUMP 0x05
// 0x05 - telemetry dump  - sent to the pi with ESC telemetry data
//                        - payload 10 byte ESC telemetry buffer

// Debug switch
const bool debugMode = false;

// Callback declaration
bool piMessageReceived(const uint8_t* payload, const uint16_t payloadSize);

// UARTs
HardwareSerial escTelemetryUART(1);
HardwareSerial piUART(2);

// Message interface
MessageInterface piMessaging(piUART, piMessageReceived);

// RGB
Adafruit_NeoPixel rgb(NUM_LEDS, PIN_RGB, NEO_GRB + NEO_KHZ800);

// Servos
const uint16_t servoMicrosMin = 1000;
const uint16_t servoMicrosMax = 2000;
const uint16_t steeringServoDefaultMicros = 1500;
const uint16_t throttleServoDefaultMicros = 1500;
uint16_t steeringServoMicrosNew = 1500;
uint16_t throttleServoMicrosNew = 1500;
Servo steeringServo;
Servo throttleServo;
bool newServoData = false;
uint32_t lastSetServoMillis = 0;
const uint32_t maxSetServoIntervalMillis = 1000;

// ESC Telemetry
const bool ESC_TELEMETRY_ENABLED = true;
ESCTelemetry escTelemetry(escTelemetryUART);

// MPU6050
MPU6050 mpu;
bool dmpReady = false;
uint16_t packetSize = 0;
uint8_t fifoBuffer[64];

// To be completely honest, I don't know if this is required. I just saw this in the convoluted MPU6050 motionapps example
volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

void initMPU6050() {
    // Initialize MPU
    mpu.initialize();
    pinMode(PIN_MPU6050_INT, INPUT);

    uint8_t devStatus = mpu.dmpInitialize();

    // Supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    if (devStatus == 0) {
        // Calibration Time
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();

        // Enable DMP so the IMU fuses a quaternion for us
        mpu.setDMPEnabled(true);

        // Setup interrupt
        attachInterrupt(digitalPinToInterrupt(PIN_MPU6050_INT), dmpDataReady, RISING);
        const uint8_t mpuIntStatus = mpu.getIntStatus();

        dmpReady = true;

        // Get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
}

void initRGB() {
    rgb.begin();
    rgb.clear();

    // Cycle colors for startup
    for (const uint32_t color : {0x100000, 0x001000, 0x000010}) {
        rgb.setPixelColor(0, color);
        rgb.show();
        delay(500);
    }    
    
    rgb.clear();
    rgb.show();
}

void setup() {
    // Initialize debug serial
    Serial.begin(115200);

    // Initialize rpi UART
    piUART.begin(230400, SERIAL_8N1, PIN_PI_UART_RX, PIN_PI_UART_TX);

    // Initialize ESC telemetry UART
    if (ESC_TELEMETRY_ENABLED)
        escTelemetryUART.begin(115200, SERIAL_8N1, PIN_ESC_UART_RX, PIN_ESC_UART_TX); 

    // Initialize servos
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    steeringServo.setPeriodHertz(50);
    steeringServo.attach(PIN_STEERING_SERVO, servoMicrosMin, servoMicrosMax);
    steeringServo.write(steeringServoDefaultMicros);
    throttleServo.setPeriodHertz(50);
    throttleServo.attach(PIN_THROTTLE_SERVO, servoMicrosMin, servoMicrosMax);
    throttleServo.write(throttleServoDefaultMicros);

    // Initialize the RGB strip
    initRGB();

    // Initialize I2C
    Wire.begin();
    Wire.setClock(400000);

    // Initialize MPU
    initMPU6050();
}

bool piMessageReceived(const uint8_t* payload, const uint16_t payloadSize) {
    // Decode payload
    if (payloadSize >= 1) {
        const uint8_t commandId = payload[0];

        if (commandId == CMD_ECHO_REQUEST) {
            // Send the payload back
            uint8_t responsePayload[payloadSize];
            memcpy(responsePayload, payload, payloadSize);
            responsePayload[0] = CMD_ECHO_RESPONSE;
            piMessaging.sendMessage(responsePayload, payloadSize);
        }

        if (commandId == CMD_SET_RGB) {
            for (uint8_t ledIndex = 0; ledIndex < NUM_LEDS; ledIndex++) {
                const uint16_t payloadIndex = (ledIndex * 4) + 1;
                if ((payloadIndex + 4) <= payloadSize) {
                    const uint32_t color = *reinterpret_cast<const uint32_t*>(&payload[payloadIndex]);
                    rgb.setPixelColor(ledIndex, color);
                }
                rgb.show();
            }
        }
        if (commandId == CMD_SET_SERVOS) {
            if (payloadSize >= 5) {
                const uint16_t parsedSteeringServoMicros = *reinterpret_cast<const uint16_t*>(&payload[1]);
                const uint16_t parsedThrottleServoMicros = *reinterpret_cast<const uint16_t*>(&payload[3]);
                
                // Update the servos
                steeringServoMicrosNew = parsedSteeringServoMicros;
                throttleServoMicrosNew = parsedThrottleServoMicros;
                newServoData = true;

                // Keep track of when we last updated the servos
                lastSetServoMillis = millis();
            }
        }
    }

    return true;
}

void loop() {
    piMessaging.update();
    
    uint32_t currentMillis = millis();

    // ESC Telemetry
    if (ESC_TELEMETRY_ENABLED) {
        if (escTelemetry.update()) {
            uint8_t payload[1 + ESC_TELEMETRY_BUF_SIZE];
            payload[0] = CMD_ESC_TELEMETRY_DUMP;
            memcpy(&payload[1], escTelemetry.buffer, ESC_TELEMETRY_BUF_SIZE);
            piMessaging.sendMessage(payload, 1 + ESC_TELEMETRY_BUF_SIZE);
        }
    }

    // MPU6050
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        // Send the FIFO buffer
        uint8_t payload[1 + packetSize];
        payload[0] = CMD_IMU_DUMP;
        memcpy(&payload[1], fifoBuffer, packetSize);
        piMessaging.sendMessage(payload, 1 + packetSize);        
    }

    // If we havent received a servo update in maxSetServoIntervalMillis, set the servos back to their defaults (failsafe)
    if ((currentMillis - lastSetServoMillis) > maxSetServoIntervalMillis) {
        steeringServo.write(steeringServoDefaultMicros);
        throttleServo.write(throttleServoDefaultMicros);
    } else if (newServoData) {
        steeringServo.write(steeringServoMicrosNew);
        throttleServo.write(throttleServoMicrosNew);
    }
}
