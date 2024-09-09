#include "controller.h"

float Controller::map(const float in, const float inMin, const float inMax, const float outMin, const float outMax) {
    return (in - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

bool Controller::isJoystickAvailable() {
    struct stat buffer;
    return (stat(DEFAULT_JOYSTICK.c_str(), &buffer) == 0); 
}

bool Controller::tryConnect() {
    // Reset to a safe state
    disconnected();

    printf("Attempting to connect controller...\n");

    // Re-open the joystick
    joystick.openPath(DEFAULT_JOYSTICK, true);

    return joystick.isFound();
}

void Controller::disconnected() {
    rightTriggerPos = 0.0f;
    leftTriggerPos = 0.0f;
    throttlePos = 0.0f;
    steeringPos = 0.0f;

    autonomousModeActive = false;
    recordingModeActive = false;
    isConnected = false;

    aButtonPressed = false;
    bButtonPressed = false;
    xButtonPressed = false;
    yButtonPressed = false;

    dPadUpPressed = false;
    dPadDownPressed = false;    
    dPadLeftPressed = false;
    dPadRightPressed = false;
}

void Controller::poll() {
    // Check if joystick is connected    
    uint64_t timestamp = TimeStamp::get();
    if ((timestamp - lastConnectedCheckTimestamp) > MIN_CONNECTED_CHECK_NSECS) {
        lastConnectedCheckTimestamp = timestamp;
        if (!isJoystickAvailable() || !joystick.isFound()) {
            // Try to connect the joystick if it isn't connected/found
            isConnected = tryConnect();
        }
    }

    if (joystick.isFound()) {
        if (!isConnected)
            isConnected = true;

        JoystickEvent event;
        if (joystick.sample(&event)) {
            if (event.isButton()) {
                bool aButtonPressedPrevious = aButtonPressed;
                bool bButtonPressedPrevious = bButtonPressed;
                bool xButtonPressedPrevious = xButtonPressed;
                bool yButtonPressedPrevious = yButtonPressed;

                if (event.number == 0)
                    aButtonPressed = (event.value > 0);
                if (event.number == 1)
                    bButtonPressed = (event.value > 0);
                if (event.number == 3)
                    xButtonPressed = (event.value > 0);
                if (event.number == 4)
                    yButtonPressed = (event.value > 0);

                // Toggle autonomous mode
                if (!aButtonPressedPrevious && aButtonPressed)
                    autonomousModeActive = !autonomousModeActive;

                // Toggle recording mode
                if (!yButtonPressedPrevious && yButtonPressed)
                    recordingModeActive = !recordingModeActive;
            }
            if (event.isAxis()) {
                // Left stick x axis
                if (event.number == 0) {
                    steeringPos = map(event.value, JoystickEvent::MIN_AXES_VALUE, JoystickEvent::MAX_AXES_VALUE, -1.0f, 1.0f);
                }
                
                // Right trigger
                if (event.number == 4) {
                    rightTriggerPos = map(event.value, JoystickEvent::MIN_AXES_VALUE, JoystickEvent::MAX_AXES_VALUE, 0.0f, 1.0f);
                }

                // Left trigger
                if (event.number == 5) {
                    leftTriggerPos = map(event.value, JoystickEvent::MIN_AXES_VALUE, JoystickEvent::MAX_AXES_VALUE, 0.0f, 1.0f);
                }

                // DPad L/R
                if (event.number == 6) {
                    dPadLeftPressed = (event.value < 0);
                    dPadRightPressed = (event.value > 0);
                }

                // DPad U/D
                if (event.number == 7) {
                    dPadUpPressed = (event.value < 0);
                    dPadDownPressed = (event.value > 0);
                }

                // Combine left and right trigger
                throttlePos = (rightTriggerPos - leftTriggerPos);

                // Cancel autonomous mode if the left trigger is pressed a certain amount
                if (leftTriggerPos >= autonomousModeCancelLeftTriggerMinimum) {
                    autonomousModeActive = false;
                }
            }
        }
    } else {
        disconnected();
    }
}

void Controller::pollLoop() {
    while (isPolling) {
        poll();
        usleep(POLLING_LOOP_DELAY_USECS);
    }
}

bool Controller::startPolling() {
    if (!isPolling) {
        isPolling = true;
        pollingThread = std::thread(&Controller::pollLoop, this);
        return true;
    } else {
        printf("Cannot start polling. Controller is already polling\n");
        return false;
    }
}

bool Controller::stopPolling() {
    if (isPolling) {
        isPolling = false;
        pollingThread.join();
        disconnected();

        return true;
    }
    return false;
}