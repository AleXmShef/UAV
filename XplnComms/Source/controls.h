#ifndef UAV_CONTROLS_H
#define UAV_CONTROLS_H

#include "XPLMDataAccess.h"
#include <vector>
#include <string>
#include <map>

namespace UAV {
    class Guidance {
    public:
        Guidance();
        static Guidance* GetInstance();

        void Update();

        ~Guidance();

    private:
        std::map<XPLMDataRef, std::string>* mDataRefs;
        static Guidance* mInstance;
    };
}

#endif //UAV_CONTROLS_H
