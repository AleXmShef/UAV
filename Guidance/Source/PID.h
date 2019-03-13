#ifndef UAV_PIDCONTROLLER_H
#define UAV_PIDCONTROLLER_H

namespace UAV {
    class PID {
    public:
        PID(){};
        PID(double max, double min);
        PID(double dt, double max, double min, double Kp, double Kd, double Ki);

        double calculate(double setpoint, double pv);
    public:
        double _dT = 0.1;
        double _max = 10;
        double _min = -10;
        double _Kp = 0.5;
        double _Ki = 0;
        double _Kd = 0;
    private:
        double _prev_err = 0;
        double _integral = 0;
    };
};

#endif //UAV_PIDCONTROLLER_H