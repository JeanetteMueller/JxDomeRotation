#include "JxDomeRotation.h"

JxDomeRotation::JxDomeRotation(int16_t inputMin, int16_t inputMax, int16_t inputCenter, uint32_t analogFrequency)
{
#ifdef ESP8266
    analogWriteFreq(analogFrequency);
#endif

    _inputMin = inputMin;
    _inputMax = inputMax;
    _inputCenter = inputCenter;
}

void JxDomeRotation::setupMotor(MODE mode, uint8_t pin1, uint8_t pin2, uint8_t channel, uint8_t resolution, uint32_t frequency)
{
    if (_isReady != true)
    {
#if defined(ESP32)
        _motor = new CytronMD(mode, pin1, pin2, channel, frequency, resolution);
#elif defined(ESP8266)
        _motor = new CytronMD(mode, pin1, pin2);
#endif

        _speed = 0;
        _motor->setSpeed(_speed);

        _currentMotorType = Direct;
        _isReady = true;
    }
}

void JxDomeRotation::setupMotor(Adafruit_PWMServoDriver *pwm, uint8_t pinDir, uint8_t pinPwm)
{
    if (_isReady != true)
    {
        _pwm = pwm;
        _pinDir = pinDir;
        _pinPwm = pinPwm;

        _currentMotorType = PWMServoDriver;
        _isReady = true;
    }
}

void JxDomeRotation::updateMotorWith(int16_t value, uint16_t deadPoint, int16_t maxSpeed)
{
    int16_t speed = constrain(value, _inputMin, _inputMax);
    if (value >= _inputCenter + deadPoint || value <= _inputCenter - deadPoint)
    {
        if (_currentMotorType == Direct)
        {
            // update via Motordriver pins
            int16_t rotationSpeed = 0;
            if (speed < _inputCenter - deadPoint || speed > _inputCenter + deadPoint)
            {
                if (speed < _inputCenter - deadPoint)
                {
                    rotationSpeed = map(speed, _inputMin, _inputCenter - deadPoint, -maxSpeed, 0);
                }
                else if (speed > _inputCenter + deadPoint)
                {
                    rotationSpeed = map(speed, _inputCenter + deadPoint, _inputMax, 0, maxSpeed);
                }

                rotationSpeed = constrain(rotationSpeed, -(maxSpeed), maxSpeed);
                _motor->setSpeed(rotationSpeed);
            }
            else
            {
                _motor->setSpeed(rotationSpeed);
            }
        }
        else if (_currentMotorType == PWMServoDriver)
        {
            // update via PWM
            uint16_t rotationSpeed = 0;
            if (value > _inputCenter)
            {
                rotationSpeed = map(value, _inputCenter, _inputMax, 0, 4095);

                _pwm->setPWM(_pinDir, 0, 4095);
                _pwm->setPWM(_pinPwm, 0, rotationSpeed);
            }
            else if (value < _inputCenter)
            {
                rotationSpeed = map(value, _inputMin, _inputCenter, 4095, 0);

                _pwm->setPWM(_pinDir, 0, 0);
                _pwm->setPWM(_pinPwm, 0, rotationSpeed);
            }
        }
    }
    else
    {
        if (_currentMotorType == Direct)
        {
            _motor->setSpeed(_speed);
        }
        else if (_currentMotorType == PWMServoDriver)
        {
            _pwm->setPWM(_pinDir, 0, 0);
            _pwm->setPWM(_pinPwm, 0, 0);
        }
    }
}

int16_t JxDomeRotation::getCurrentSpeed()
{
    return _speed;
}

void JxDomeRotation::stop()
{
    updateMotorWith(0, 0);
}