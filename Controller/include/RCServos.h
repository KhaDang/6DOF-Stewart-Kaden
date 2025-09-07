#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

class RCServos {
public:
    enum Error {
        ERROR_NONE,
        ERROR_NOT_ATTACHED,
        ERROR_INVALID_ANGLE,
        ERROR_NOT_INITIALIZED
    };

    struct Config {

        bool enableSoftLimits = false;

        int minAngle = 0;           // Minimum angle (deg)
        int maxAngle = 180;         // Maximum angle (deg)
        float maxVelocity = 90.0f;  // Maximum speed (deg/sec)
        uint32_t updateInterval_ms = 20; // Servo refresh rate (~50 Hz)
    };
    // Constructor- initializes motor pins and default state
    RCServos(uint8_t channel, Adafruit_PWMServoDriver* driver):
        _channel(channel),
        _pwm(driver),
        _attached(false),
        _error(ERROR_NOT_INITIALIZED),
        _currentAngle(0.0f),
        _targetAngle(0.0f),
        _lastUpdate(0) {}

    // Initialize the servo
    bool begin(const Config& cfg) {
        _config = cfg;
        if (!_pwm) {  // Instance of Adafruit_PWMServo
            _error = ERROR_NOT_ATTACHED;
            return false;
        }
        _config = cfg;
        _pwm->begin();
        _pwm->setPWMFreq(50);

        _attached = true;
        _error = ERROR_NONE;
        return true;
    }

    // Set new target angle
    bool setTargetAngle(float angle) {
        if (!_attached) {
            _error = ERROR_NOT_ATTACHED;
            return false;
        }
        if (angle < _config.minAngle || angle > _config.maxAngle) {
            _error = ERROR_INVALID_ANGLE;
            return false;
        }
        _targetAngle = angle;
        return true;
    }

    // Update should be called frequently (non-blocking)
    bool update() {
        if (!_attached) {
            _error = ERROR_NOT_INITIALIZED;
            return false;
        }

        uint32_t now = millis();
        uint32_t dt = now - _lastUpdate;

        if (dt < _config.updateInterval_ms) {
            return true; // too soon, wait until refresh interval
        }
        _lastUpdate = now;

        float delta = _targetAngle - _currentAngle;
        if (fabs(delta) < 0.01f) {
            return true; // already at target
        }

        // Compute how much we can move in this interval
        float step = _config.maxVelocity * (dt / 1000.0f); // deg = deg/s * s
        if (fabs(delta) <= step) {
            _currentAngle = _targetAngle; // reach target
        } else {
            _currentAngle += (delta > 0 ? step : -step); // move toward target
        }

        // Convert angle to PCA9685 pulse
        uint16_t pulse = angleToPulse(_currentAngle);
        _pwm->setPWM(_channel, 0, pulse);
        return true;
    }

    // Accessors
    Error getLastError() const { return _error; }
    float getCurrentAngle() const { return _currentAngle; }
    float getTargetAngle() const { return _targetAngle; }

private:

    uint16_t angleToPulse(float angle) {
    // Map [0,180] deg to [150,600] ticks (approx for SG90, MG996R etc.)
    return map((int)angle, 0, 180, 150, 600);
    }

    uint8_t _channel;
    Adafruit_PWMServoDriver* _pwm;
    bool _attached;
    Error _error;
    Config _config;

    float _currentAngle;
    float _targetAngle;
    uint32_t _lastUpdate;
};
