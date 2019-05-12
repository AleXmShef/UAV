#include <vector>
#include <windows.h>
#include <iostream>
#include "IPC.h"
#include "Guidance.h"
#include <thread>


using namespace UAV;

bool flag = true;

void _debug_sharedMemoryInit() {     //flag to override control surfaces

    //Initialize data maps
    shared_memory_object::remove(SHRDMEM_NAME);
    shared_memory_object::remove("mtx");
    IPCns::IPC::GetInstance();
    IPCns::IPCSharedMap* mOutputMap;
    IPCns::IPCSharedMap* mInputMap;
    mOutputMap = IPCns::IPC::registerData(mOutputMap, SHRDOUTPUT_NAME);
    mInputMap = IPCns::IPC::registerData(mInputMap, SHRDINPUT_NAME);

    //FUCKING ENUMS
    for(int i = 0; i < ControlSrfcOverride + 1; i++) {
        mOutputMap->insert(std::pair<const int, float>(i, 0));
    }

    for(int i = 0; i < Rudder + 1; i++) {
        mInputMap->insert(std::pair<const int, float>(i, 0));
    }
}

int main() {
    //DEBUG
    //_debug_sharedMemoryInit();
    //---
    Guidance* mGuidance = Guidance::GetInstance();
    mGuidance->Run();
    while(flag) {
        int a;
        std::cin >> a;
        int b;
        switch(a) {
            case 1:
                mGuidance->Stop();
                //flag = false;
                break;
            case 2:
                std::cin >> b;
                mGuidance->_debug_ChangeAutopilotVNAVMode(Vspeed, b);
                break;
            case 3:
                std::cin >> b;
                mGuidance->_debug_ChangeAutopilotLNAVMode(HDGselect, b);
                break;
            case 4:
                std::cin >> b;
                mGuidance->_debug_ChangeAutopilotVNAVMode(LVLCHNG, b);
                break;
            case 5:
                mGuidance->_debug_ToggleAutopilotATarm();
                break;
            case 6:
                mGuidance->_debug_StartRouteGeneration();
                break;
            case 7:
                mGuidance->Run();
                break;
            case 0:
                mGuidance->Stop();
                flag = false;
                break;
            default:
                break;
        }
    }

    return 0;
}

/*TODO:
 * PID properties loading from xml
 * Coordinate system conversion from spherical to planar
 * */
