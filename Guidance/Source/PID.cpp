#include <cmath>
#include "PID.h"
#include <math.h>

using namespace UAV;

PID::PID(double max, double min):
        _max(max),
        _min(min)
{}

PID::PID(double dt, double max, double min, double Kp, double Kd, double Ki):
        _dT(dt),
        _max(max),
        _min(min),
        _Kp(Kp),
        _Kd(Kd),
        _Ki(Ki)
{}

double PID::calculate(double setpoint, double pv) {

    //Calculate error
    double error = setpoint - pv;
    if((abs(error)/error) != (abs(_prev_err)/_prev_err))
        _integral = 0;

    //Proportional term
    double Pout = _Kp * error;

    //Integral term
    if (fabs(error) < 0.001)
        _integral = 0;
    _integral += error * _dT;
    double Iout = _Ki * _integral;

    //Derivative term
    double derivative = (error - _prev_err) / _dT;
    double Dout = _Kd * derivative;

    //End result
    double output = Pout + Iout + Dout;

    //Constraint to max/min
    if (output > _max)
        output = _max;
    else if (output < _min)
        output = _min;

    _prev_err = error;

    return output;
}

