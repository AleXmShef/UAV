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
    enum DataMaps{DerivedData, ControlsData};
    enum LNAVmodes{RouteL, HDGselect};
    enum VNAVmodes{RouteV, ALThold, LVLCHNG, Vspeed};

    struct AutopilotSettings {
        double HDG = 0;
        double ALT = 1000;
        double VSPD = 0;
        double SPD = 100;
    };

    class Guidance {
    public:
        static Guidance* GetInstance();
        void Update();
        void Log(double* p, int n, int c);
    private:
        Guidance();
        void Init();
        void UpdateGuidance();
        void UpdateControls(double PitchCorr, double RollCorr);

        double GetCl(double Pitch);
        double GetPitch(double Cl);

        float phi = 0;
        clock_t t = 0;
        time_t t2 = 0;

        LNAVmodes LNAVmode = HDGselect;
        VNAVmodes VNAVmode = ALThold;
        AutopilotSettings mAutopilotSettings;

        std::map<Axes, PIDPipeline*> mPIDpipelines;
        std::map<DataMaps, IPCns::IPCSharedMap*> mDataMaps;
        std::map<std::string, double> mValuesForCalculation;


        static Guidance* mInstance;
    };
}

#endif //UAV_GUIDANCE_H
