#ifndef UAV_GUIDANCE_H
#define UAV_GUIDANCE_H

#include <map>
#include <windows.h>
#include <iostream>
#include "IPC.h"
#include "enums.h"
#include "PIDPipeline.h"
#include "Logger.h"
#include "Waypoint.h"
#include "Route.h"

namespace UAV {
    enum LNAVmodes{RouteL, HDGselect};
    enum VNAVmodes{RouteV, ALThold, LVLCHNG, Vspeed};

    struct AutopilotSettings {
        LNAVmodes LNAVmode = HDGselect;
        VNAVmodes VNAVmode = Vspeed;

        double HDG = 0;
        double ALT = 1000;
        double VSPD = 0;
        double SPD = 100;
    };

    class Guidance: public Loggable {
        enum PIDdesc{PitchPIDpipe, RollPIDpipe, LVLchngPIDpipe, HDGselectPIDpipe};
        enum DataMaps{DerivedData, ControlsData};
        struct Telemetry {
            std::map<DataMaps, IPCns::IPCSharedMap*> mDataMaps;

            double latitude;
            double longitude;

            double pitch;
            double roll;
            double heading;

            double altitudeM;
            double altitudeF;
            double IAS;         //Indicated Airspeed

            double pitchAoA;
            double yawAoA;

            double verticalPath;
            double horizontalPath;
            double verticalVelocityFPM;
            double verticalVelocityMPS;
            double lateralVelocity;

            double IAsAcceleration;
            double verticalAccelereation = 0;
            double lateralAcceleration;

            double pitchAngularVelocity;
            double rollAngularVelocity;
            double yawAngularVelocity;

            double pitchAngularAcceleration;
            double rollAngularAcceleration;
            double yawAngularAcceleration;

            double massCurrent;
            double massFuel;
        };
        struct ControlCalculation {
            std::map<PIDdesc, PIDPipeline*> mPIDpipelines;
            std::map<PIDdesc, std::vector<double>> mPIDpipelinesErrors;

            double pitchCorr = 0;
            double rollCorr = 0;
            double desiredPitch = 0;
            double desiredRoll = 0;
        };
        struct Variables {
            bool MasterSwitch = 0;

            Telemetry telemetry;

            ControlCalculation controlCalculation;

            AutopilotSettings autopilotSettings;

            Route* mRoute = nullptr;

            std::vector<std::thread*> threadArray;

            float phiForControlsDebug = 0;
            clock_t clockPassed = 0;
            double timePassed = 0;
        };

    public:
        static Guidance* GetInstance();

        void Run();
        void Stop();

        void _debug_ChangeAutopilotLNAVMode(LNAVmodes mode, double value);
        void _debug_ChangeAutopilotVNAVMode(VNAVmodes mode, double value);
        void _debug_StartRouteGeneration();

        std::map<std::string, double>* getLogInfo() override;

    protected:
        void _th_UpdateGuidance();
        void _th_CalculateControls();
        void _th_UpdateControls();

        void UpdateTelemetry();
        void UpdateGuidance();
        void CalculateControls();
        void UpdateControls();


        Guidance();

        void Log(double* p, int n, int c);  ///Fair Enough
        void Init();


        void SetRoute();
        double GetCl(double Pitch);
        double GetPitch(double Cl);

    protected:
        Variables mVariables;
        clock_t t = 0;
        static Guidance* mInstance;
    };
}

#endif //UAV_GUIDANCE_H
