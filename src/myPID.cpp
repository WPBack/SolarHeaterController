#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "myPID.h"

myPID::myPID(double* input, double* output, double* setpoint, double kP, double kI, double kD, int direction) {
  _input = input;
  _output = output;
  _setpoint = setpoint;
  _kP = kP;
  _kI = kI;
  _kD = kD;
  _direction = direction;

  _upperLimit = 1023;
  _lowerLimit = 0;

  _sampleTime = 100;

  _deadbandMin = 0;
  _deadbandPlus = 0;

  _prevMillis = millis();
  _prevInput = *_input;
  _integralPart = 0;
}

void myPID::calculate() {
  if(millis() - _prevMillis >= _sampleTime) {
    // Update the time
    _prevMillis = millis();

    // Calculate the error
    if(_direction == NORMAL) {
      _error = *_setpoint - *_input;
      _inputChange = _prevInput - *_input;
    }
    else {
      _error = *_input - *_setpoint;
      _inputChange = *_input - _prevInput;
    }

    // Updates the last input
    _prevInput = *_input;

    // Update the integral part
    _integralPart += _error*_kI*_sampleTime;

    // Calculate the output
    *_output = _error*_kP + _integralPart + _inputChange*_kD/_sampleTime;

    // Checks if the output is outside of the bounds and prevents integral windup
    if(*_output < _lowerLimit) {
      *_output = _lowerLimit;
      _integralPart = *_output - _error*_kP - _inputChange*_kD/_sampleTime;
    }
    else if(*_output > _upperLimit) {
      *_output = _upperLimit;
      _integralPart = *_output - _error*_kP - _inputChange*_kD/_sampleTime;
    }
  }
}

void myPID::setOutputLimits(double lower, double upper) {
  _lowerLimit = lower;
  _upperLimit = upper;
}

void myPID::setParamters(double kP, double kI, double kD) {
  _kP = kP;
  _kI = kI;
  _kD = kD;
}

void myPID::setDirection(int direction) {
  _direction = direction;
}

void myPID::setSampleTime(int sampleTime) {
  _sampleTime = sampleTime;
}

void myPID::setDeadBand(double minus, double plus) {
  _deadbandMin = minus;
  _deadbandPlus = plus;
}
