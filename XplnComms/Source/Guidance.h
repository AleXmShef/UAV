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

        std::thread* GetThread();

        ~Guidance();

        std::thread* mThread;

    private:
        Guidance();

        std::map<SimDataEnum, XPLMDataRef>* mODataRefs;
        std::map<ControlsEnum, XPLMDataRef>* mIDataRefs;
        std::vector<std::string> mODataRefsTypes;
        std::vector<std::string> mIDataRefsTypes;
        static Guidance* mInstance;
        IPCns::IPCSharedMap* mOutputMap;
        IPCns::IPCSharedMap* mInputMap;

    };
}

#endif //UAV_CONTROLS_H
