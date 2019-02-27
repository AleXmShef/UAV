#ifndef UAV_CONTROLS_H
#define UAV_CONTROLS_H

#include <vector>
#include <string>
#include <map>
#include <thread>
#include "enums.h"
#include "XPLMDataAccess.h"
#include "IPC.h"

namespace XCOM {
    enum ControlsEnum{ControlPitch, ControlRoll, ControlYaw, ControlThrottle};
    class Guidance {
    public:
        static Guidance* GetInstance();

        void Update();

        ~Guidance();

    private:
        Guidance();

        std::map<SimDataEnum, XPLMDataRef>* mODataRefs;
        std::map<ControlsEnum, XPLMDataRef>* mIDataRefs;
        IPCns::IPCSharedMap* mOutputMap;
        IPCns::IPCSharedMap* mInputMap;

        static Guidance* mInstance;
    };
}

#endif //UAV_CONTROLS_H
