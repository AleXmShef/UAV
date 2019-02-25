#ifndef UAV_GUIDANCE_H
#define UAV_GUIDANCE_H

#include <map>
#include "IPC.h"
#include "enums.h"
#include "PIDPipeline.h"

namespace UAV {
    enum Axes{X, Y};
    enum PipelinePIDs{AOApid, AngVelPID, AngAccPID};
    enum DataMaps{SimData, ControlsData};

    class Guidance {
    public:
        static Guidance* GetInstance();
        static void Init();
        static void Update();
        static void MasterSwitch(bool flag);     //FIXME: temporary
    private:
        static Guidance* mInstance;
        static std::map<Axes, PIDPipeline*> mPIDpipelines;
        static std::map<DataMaps, IPCns::IPCSharedMap*> mDataMaps;
    };
}

#endif //UAV_GUIDANCE_H
