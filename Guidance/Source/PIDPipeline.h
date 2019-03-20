#ifndef UAV_PIDPIPELINE_H
#define UAV_PIDPIPELINE_H

#include "PID.h"
#include <vector>

namespace UAV {
    class PIDPipeline {
    public:
        PIDPipeline();
        explicit PIDPipeline(int numPIDs);
        explicit PIDPipeline(std::vector<PID*>* PIDs);
        explicit PIDPipeline(PID* pid);

        double Calculate(double setpoint, std::vector<double> errs);
        double Calculate(double setpoint, double error);
        PID* GetPID(int numPID);

    private:
        std::vector<PID*>* _pids;
    };
}

#endif //UAV_PIDPIPELINE_H
