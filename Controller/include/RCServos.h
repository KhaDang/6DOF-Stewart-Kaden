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
        float maxVelocity = 10.0f;  // Maximum speed (deg/sec)
        uint32_t updateInterval_ms = 20; // Servo refresh rate (~50 Hz)
    };
    // Constructor- initializes motor pins and default state
    RCServos(uint8_t channel, Adafruit_PWMServoDriver* driver):
        _channel(channel),
        _pwm(driver),
        _attached(false),
        _error(ERROR_NONE),
        _currentAngle(90.0f),
        _targetAngle(90.0f),
        _lastUpdate(0) {}

    // Initialize the servo
    bool begin(const Config& cfg) {
     
        if (!_pwm) {  // Instance of Adafruit_PWMServo
            _error = ERROR_NOT_ATTACHED;
            return false;
        }
        _config = cfg;
        _pwm->begin();
        _pwm->setPWMFreq(50);
        // _pwm->setOscillatorFrequency(2700000); //2.7 kHz
        _attached = true;
        _error = ERROR_NONE;
        return true;
    }

    // Set new target angle
    bool setTargetAngle(float angle) {

        Serial.print("Target angle: "); Serial.println(angle);

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

        // Serial.printf("Motor %d properties:\n",_channel);
        // Serial.printf("target angle: %.2f\n", _targetAngle);
        // Serial.printf("Current angle: %.2f\n", _currentAngle);
        // Serial.printf("deta value: %.2f\n", delta);

        // delay(5000);

        if (fabs(delta) < 0.01f) {
            return true; // already at target
        }

        // Compute how much we can move in this interval
       float maxStep = _config.maxVelocity * (_config.updateInterval_ms / 1000.0f);
        if (fabs(delta) > maxStep) {
            _currentAngle += (delta > 0 ? maxStep : -maxStep);
        } else {
            _currentAngle = _targetAngle;
        }

        // Convert angle to PCA9685 pulse, micro seconds
        double micro_second = map(_currentAngle, 0 ,179, 600, 2400);

        _pwm->writeMicroseconds(_channel, micro_second);
        // delay(1000);
        Serial.printf("MOTOR: %d POSITION GETTING UPDATED \n", _channel);
        Serial.printf("Microsecond written to motor %f \n", micro_second);

        return true;
    }

    // Accessors
    Error getLastError() const { return _error; }
    float getCurrentAngle() const { return _currentAngle; }
    float getTargetAngle() const { return _targetAngle; }

private:

    uint8_t _channel;
    Adafruit_PWMServoDriver* _pwm;
    bool _attached;
    Error _error;
    Config _config;

    float _currentAngle;
    float _targetAngle;
    uint32_t _lastUpdate;
};
