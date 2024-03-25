
#ifndef JxDomeRotation_h
#define JxDomeRotation_h

#include "Arduino.h"
#include <CytronMotorDriver.h>

class JxDomeRotation
{
public:
    int16_t currentSpeed() const { return _speed; };

    JxDomeRotation(int16_t inputMin, int16_t inputMax, int16_t inputCenter, uint32_t analogFrequenc = 18000);

    void setupMotor(MODE mode, uint8_t pin1, uint8_t pin2);

    void updateMotorWith(int16_t value, uint16_t deadPoint, int16_t maxSpeed = 255);
    void stop();

private:
    int16_t _inputMin = -500;
    int16_t _inputMax = 500;
    int16_t _inputCenter = 0;
    int16_t _speed = 0;

    CytronMD *_motor;

    void updateSpeed();
};

#endif