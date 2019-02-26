#ifndef UAV_GUIDANCE_H
#define UAV_GUIDANCE_H

#include <map>
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
    private:
        void Init();
        Guidance();
        static Guidance* mInstance;
        std::map<Axes, PIDPipeline*> mPIDpipelines;
        std::map<DataMaps, IPCns::IPCSharedMap*> mDataMaps;
    };
}

#endif //UAV_GUIDANCE_H
