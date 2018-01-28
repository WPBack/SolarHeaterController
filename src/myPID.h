#ifndef myPID_h
#define myPID_h

class myPID {

  public:

    #define NORMAL 0
    #define REVERSE 1

    myPID(double*, double*, double*, double, double, double, int);

    void calculate();

    void setOutputLimits(double, double);
    void setParamters(double, double, double);
    void setDirection(int);
    void setSampleTime(int);
    void setDeadBand(double, double);

  private:
    double* _input;
    double* _output;
    double* _setpoint;

    double _kP;
    double _kI;
    double _kD;

    int _direction;

    double _upperLimit;
    double _lowerLimit;

    unsigned int _sampleTime;

    double _deadbandMin;
    double _deadbandPlus;

    unsigned int _prevMillis;
    double _prevInput;
    double _integralPart;
    double _error;
    double _inputChange;
};

#endif
