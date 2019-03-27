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
    //enum ControlsEnum{LeftElev, RightElev, LeftAil, RightAil, Rudder, Throttle};
    class Guidance {
    public:
        static Guidance* GetInstance();

        void Update();
        void SwitchControls();
        ~Guidance();
        float GetDataref(SimDataEnum at){return mOutputMap->at(at);};

    private:
        Guidance();

        std::map<SimDataEnum, XPLMDataRef>* mODataRefs;
        std::map<ControlsDataEnum, XPLMDataRef>* mIDataRefs;
        IPCns::IPCSharedMap* mOutputMap;
        IPCns::IPCSharedMap* mInputMap;

        static Guidance* mInstance;
    };
}

#endif //UAV_CONTROLS_H

/*TODO:
 * Data output into the window
 * Autopilot master switch via button
 * Graphical PFD Display with flight directors
 * */
