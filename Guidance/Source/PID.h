#ifndef UAV_PIDCONTROLLER_H
#define UAV_PIDCONTROLLER_H

#include <vector>
#include <string>
#include "Loggable.h"

namespace UAV {
    class PID: public Loggable {
    public:
        PID(){};
        PID(double max, double min);
        PID(std::string name, double max, double min, double Kp, double Kd, double Ki);

        double calculate(double setpoint, double pv);
        std::map<std::string, double>* getLogInfo() override;

    public:
        double _dT = 0.1;
        double _max = 10;
        double _min = -10;
        double _Kp = 0.01;
        double _Ki = 0.001;
        double _Kd = 0.1;
    private:
        double _last_output = 0;
        double _last_error = 0;
        double _last_Kp_output = 0;
        double _last_Ki_output = 0;
        double _last_Kd_output = 0;
        double _prev_err = 0;
        double _integral = 0;
    };
};

#endif //UAV_PIDCONTROLLER_H