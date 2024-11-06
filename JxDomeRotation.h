
#ifndef JxDomeRotation_h
#define JxDomeRotation_h

#include "Arduino.h"

#if defined(ESP32)
#include <CytronMotorDriverEsp32.h>
#elif defined(ESP8266)
#include <CytronMotorDriver.h>
#endif

#include <Adafruit_PWMServoDriver.h>

class JxDomeRotation
{
public:

    enum MotorType {
        Direct, PWMServoDriver
    };
    
    int16_t currentSpeed() const { return _speed; };

    JxDomeRotation(int16_t inputMin, int16_t inputMax, int16_t inputCenter, uint32_t analogFrequenc = 18000);

    void setupMotor(MODE mode, uint8_t pin1, uint8_t pin2);
    void setupMotor(Adafruit_PWMServoDriver *pwm, uint8_t pinDir, uint8_t pinPwm);

    void updateMotorWith(int16_t value, uint16_t deadPoint, int16_t maxSpeed = 255);
    int16_t getCurrentSpeed();
    void stop();

private:
    bool _isReady = false;
    MotorType _currentMotorType;

    int16_t _speed = 0;

    int16_t _inputMin = -500;
    int16_t _inputMax = 500;
    int16_t _inputCenter = 0;
    

    CytronMD *_motor;

    Adafruit_PWMServoDriver *_pwm;
    uint8_t _pinDir;
    uint8_t _pinPwm;

    void updateSpeed();
};

#endif