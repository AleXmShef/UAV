//
// Created by Sasha on 25.02.2019.
//

#include "PIDPipeline.h"

using namespace UAV;

PIDPipeline::PIDPipeline() {
    PID* pid = new PID();
    _pids = new std::vector<PID*>;
    _pids->push_back(pid);
}

PIDPipeline::PIDPipeline(int numPIDs) {
    _pids = new std::vector<PID*>;
    for(int i = 0; i < numPIDs; i++) {
        PID* pid = new PID();
        _pids->push_back(pid);
    }
}

PIDPipeline::PIDPipeline(std::vector<PID*>* PIDs) {
    _pids = PIDs;
}

double PIDPipeline::Calculate(double setpoint, std::vector<double> errs) {
    double result = setpoint;
    for(int i = 0; i < _pids->size(); i++) {
        result = (*_pids)[i]->calculate(result, errs[i]);
    }
    return result;
}

PID* PIDPipeline::GetPID(int numPID) {
    return (*_pids)[numPID];
}
