#ifndef UAV_CONTROLS_H
#define UAV_CONTROLS_H

#include "XPLMDataAccess.h"
#include "IPC.h"
#include <vector>
#include <string>
#include <map>

namespace UAV {
    class Guidance {
    public:
        static Guidance* GetInstance();

        void Update();

        ~Guidance();

    private:
        Guidance();
        std::map<std::string, XPLMDataRef>* mDataRefs;
        std::vector<std::string> mODataRefsTypes;
        std::vector<std::string> mIDataRefsTypes;
        static Guidance* mInstance;
        IPCns::IPCSharedMap* mOutputMap;
        IPCns::IPCSharedMap* mInputMap;
    };
}

#endif //UAV_CONTROLS_H
