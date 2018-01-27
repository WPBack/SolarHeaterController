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
    double* input;
    double* output;
    double* setpoint;

    double kP;
    double kI;
    double kD;

    int direction;

    double upperLimit;
    double lowerLimit;

    int sampleTime;

    double deadbandMin;
    double deadbandMax;
};

#endif
