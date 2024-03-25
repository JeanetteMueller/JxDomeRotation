#include "JxDomeRotation.h"

JxDomeRotation::JxDomeRotation(int16_t inputMin, int16_t inputMax, int16_t inputCenter, uint32_t analogFrequency)
{
    analogWriteFreq(analogFrequency);

    _inputMin = inputMin;
    _inputMax = inputMax;
    _inputCenter = inputCenter;
}

void JxDomeRotation::setupMotor(MODE mode, uint8_t pin1, uint8_t pin2)
{
    _motor = new CytronMD(mode, pin1, pin2);
    _speed = 0;
    _motor->setSpeed(_speed);
}

void JxDomeRotation::updateMotorWith(int16_t value, uint16_t deadPoint, int16_t maxSpeed)
{
    if (value >= _inputMin && value <= _inputMax)
    {
        int16_t rotationSpeed = map(value, _inputMin, _inputMax, maxSpeed, -(maxSpeed));

        _motor->setSpeed(rotationSpeed);
    }
    else
    {
        stop();
    }
}

void JxDomeRotation::stop()
{
    _speed = 0;
    updateSpeed();
}

void JxDomeRotation::updateSpeed()
{
    _motor->setSpeed(_speed);
}