#include "maus_board.h"
#include "fhl_ld19.h"
#include "controller.h"

void imuDataCallback(const MausBoard::ImuData& imuData) {
    // Every 10 milliseconds
    printf("IMU DATA, Yaw: %f radians\n", imuData.getYawRadians());

    // NOTE! Do not block here. Run longer tasks in a seperate thread. 
}

void escTelemetryCallback(const MausBoard::EscTelemetry& escTelemetry) {
    // Every 30 milliseconds
    printf("ESC DATA, Voltage: %f ERPM: %d\n", escTelemetry.getVoltage(), escTelemetry.getERPM());

    // NOTE!
    // Due to the nature of the KISS ESC telemetry protocol, ERPM is unsigned. 
    // This means that in order to determine which direction the motor is turning, you will need to compare this to your throttle input.
    // In addition to this, due to the limitations of the ESC, ERPM will be 0 unless throttle is applied. 
    // So ERPM will not be accurate when rolling under 0 throttle.
    // Example: 
    // float throttleValue = -0.8f;
    // const float signedERPM = copysignf(escTelemetry.getERPM(), throttleValue);

    // NOTE! Do not block here. Run longer tasks in a seperate thread.
}

void ld19FullScanCallback(std::vector<LD19::LidarPoint> points) {
    // Every 100 milliseconds
    printf("FULL SCAN: %d points\n", points.size());

    // NOTE! Do not block here. Run longer tasks in a seperate thread.
}

int main() {
    // Create an instance and set the callbacks
    MausBoard board(&imuDataCallback, &escTelemetryCallback);
    board.startReading(); // Read data in a separate thread until stopReading() 

    // Create an instance and set the callback
    LD19 ld19(&ld19FullScanCallback);
    ld19.startReading(); // Read data in a separate thread until stopReading()

    // Set the LED colors (blue, red)
    board.sentSetRGB( {0x000020, 0x200000} );

    // Send a "set servos" command to the neutral positions
    board.sendSetServos(1500, 1500);
    // Board will failsafe after 1 second if the SetServos command is not continuously recieved
    // This just means that the default servo positions will be set after 1 second (neutral)
    // In actual use, you should be sending this command at a set interval (I've used 10ms)

    // Create a controller object so we can control the car with a joystick
    Controller controller;
    controller.startPolling();

    // Set some scalers for throttle and steering
    const float throttleScaler = 0.5f; // Max throttle value from -0.5 to 0.5
    const float steeringScaler = -0.9f; // Max steering value from -0.9 to 0.9 (negative because its reversed on my vehicle)

    // Keep reading forever
    while (true) {
        // Grab the current joystick inputs and send them to the servos after scaling
        const float currentThrottleOutput = controller.throttlePos.load() * throttleScaler;
        const float currentSteeringOutput = controller.steeringPos.load() * steeringScaler;
        board.sendSetServos((currentSteeringOutput * 500.0f) + 1500, (currentThrottleOutput * 500.0f) + 1500);

        // Sleep for 10 milliseconds
        usleep(10000);
    }

    // Ideally if upon exit you should call board.stopReading() and ld19.stopReading()
    // But I haven't been doing that and I haven't seen any issues
}