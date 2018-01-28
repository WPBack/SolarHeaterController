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
    _prevMillis = millis();
    if(_direction == NORMAL) {
      _error = *_setpoint - *_input;
    }
    else {
      _error = *_input - *_setpoint;
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
