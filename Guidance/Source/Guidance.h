#ifndef UAV_GUIDANCE_H
#define UAV_GUIDANCE_H

#include <map>
#include <windows.h>
#include <iostream>
#include "IPC.h"
#include "enums.h"
#include "PIDPipeline.h"
#include "Logger.h"

namespace UAV {
    enum PIDdesc{PitchPIDpipe, RollPIDpipe, LVLchngPIDpipe, HDGselectPIDpipe};
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
        void SetRoute();

    public:
        LNAVmodes LNAVmode = HDGselect;
        VNAVmodes VNAVmode = Vspeed;
        bool MasterArm = 0;
        AutopilotSettings mAutopilotSettings;
        std::vector<std::vector<double>> mWaypoints;
    protected:
        Guidance();
        void Init();
        void UpdateGuidance();
        void UpdateControls(double PitchCorr, double RollCorr);

        double GetCl(double Pitch);
        double GetPitch(double Cl);

        float phi = 0;
        clock_t t = 0;
        time_t t2 = 0;



        std::map<PIDdesc, PIDPipeline*> mPIDpipelines;
        std::map<DataMaps, IPCns::IPCSharedMap*> mDataMaps;
        std::map<std::string, double> mValuesForCalculation;

        static Guidance* mInstance;
    };
}

#endif //UAV_GUIDANCE_H
