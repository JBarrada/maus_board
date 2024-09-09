#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

// Hey, whats up. This code is pretty gross. Please don't look too hard.
// If you have your own joystick handling code, please just use that. It is probably better than this. 

#include <sys/stat.h>
#include <atomic>
#include <thread>
#include <unistd.h>

#include "joystick.h"
#include "timestamp.h"

class Controller {
private:
    const std::string DEFAULT_JOYSTICK = "/dev/input/js0";

    // Controls how often to check if the joystick is found/connected
    static const uint64_t MIN_CONNECTED_CHECK_NSECS = 5 * (uint64_t)NSECS_TO_SECS; // 5 seconds
    uint64_t lastConnectedCheckTimestamp = 0;

    // Minimum left trigger (braking) amount to cancel autonomous mode
    const float autonomousModeCancelLeftTriggerMinimum = 0.1f;

    // Actual joystick instance for polling
    Joystick joystick;

    static const uint32_t POLLING_LOOP_DELAY_USECS = 10000;
    bool isPolling = false;
    std::thread pollingThread;

    float map(const float in, const float inMin, const float inMax, const float outMin, const float outMax);
    bool isJoystickAvailable();
    bool tryConnect();
    void disconnected();
    void poll();
    void pollLoop();
public:
    std::atomic<float> rightTriggerPos;
    std::atomic<float> leftTriggerPos;
    std::atomic<float> throttlePos;
    std::atomic<float> steeringPos;

    std::atomic<bool> autonomousModeActive;
    std::atomic<bool> recordingModeActive;
    std::atomic<bool> isConnected;

    std::atomic<bool> aButtonPressed;
    std::atomic<bool> bButtonPressed;
    std::atomic<bool> xButtonPressed;
    std::atomic<bool> yButtonPressed;

    std::atomic<bool> dPadUpPressed;
    std::atomic<bool> dPadDownPressed;
    std::atomic<bool> dPadLeftPressed;
    std::atomic<bool> dPadRightPressed;

    Controller() : isConnected(false) { disconnected(); }
    ~Controller() { stopPolling(); }

    bool startPolling();
    bool stopPolling();
};

#endif