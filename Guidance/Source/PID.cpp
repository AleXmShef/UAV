#include <cmath>
#include "PID.h"
#include <math.h>

using namespace UAV;

PID::PID(double max, double min):
        _max(max),
        _min(min)
{Logger::GetInstance()->registerLoggable(this);}

PID::PID(std::string name, double max, double min, double Kp, double Kd, double Ki):
        Loggable(name),
        _max(max),
        _min(min),
        _Kp(Kp),
        _Kd(Kd),
        _Ki(Ki)
{Logger::GetInstance()->registerLoggable(this);}

double PID::calculate(double setpoint, double pv) {

    //Calculate error
    double error = setpoint - pv;
    _last_error = error;
    if((abs(error)/error) != (abs(_prev_err)/_prev_err))
        _integral = 0;

    //Proportional term
    double Pout = _Kp * error;
    _last_Kp_output = Pout;

    //Integral term
    if (fabs(error) < 0.001)
        _integral = 0;
    _integral += error * _dT;
    double Iout = _Ki * _integral;
    _last_Ki_output = Iout;

    //Derivative term
    double derivative = (error - _prev_err) / _dT;
    double Dout = _Kd * derivative;
    _last_Kd_output = Dout;
    //End result
    double output = Pout + Iout + Dout;

    //Constraint to max/min
    if (output > _max)
        output = _max;
    else if (output < _min)
        output = _min;

    _prev_err = error;
    _last_output = output;

    return output;
}

std::map<std::string, double>* PID::getLogInfo() {
    auto mMap = new std::map<std::string, double>;

    mMap->insert({"Last output", _last_output});
    mMap->insert({"Last error", _last_error});
    mMap->insert({"Last Kp output", _last_Kp_output});
    mMap->insert({"Last Ki output", _last_Ki_output});
    mMap->insert({"Last Kd output", _last_Kd_output});

    return mMap;
}

