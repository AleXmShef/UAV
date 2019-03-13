#ifndef UAV_GUIDANCE_H
#define UAV_GUIDANCE_H

#include <map>
#include <windows.h>
#include <iostream>
#include "IPC.h"
#include "enums.h"
#include "PIDPipeline.h"

namespace UAV {
    enum Axes{X, Y};
    enum PipelinePIDs{AoAPID, AngVelPID, AngAccPID};
    enum DataMaps{SimData, ControlsData};

    class Guidance {
    public:
        static Guidance* GetInstance();
        void Update();
        void MasterSwitch(bool flag);     //FIXME: temporary
        void Log(double* p, int n, int c);
    private:
        void Init();
        Guidance();

        double GetCl(float Pitch);
        double GetPitch(float Cl);

        float phi = 0;
        time_t t = 0;
        time_t t2 = 0;
        static Guidance* mInstance;
        std::map<Axes, PIDPipeline*> mPIDpipelines;
        std::map<DataMaps, IPCns::IPCSharedMap*> mDataMaps;
        std::map<std::string, double> mValuesForCalculation;
    };
}

#endif //UAV_GUIDANCE_H
