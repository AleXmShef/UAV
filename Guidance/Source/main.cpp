#include <vector>
#include <windows.h>
#include <iostream>
#include "IPC.h"
#include "Guidance.h"
#include <thread>


using namespace UAV;

bool flag = true;

int main() {
    Guidance* mGuidance = Guidance::GetInstance();
    mGuidance->Run();
    while(flag) {
        int a;
        std::cin >> a;
        int b;
        switch(a) {
            case 1:
                mGuidance->Stop();
                flag = false;
                break;
            case 2: //Switch to VSPD
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
                mGuidance->_debug_StartRouteGeneration();
                break;
            default:
                break;
        }
    }

    return 0;
}

/*TODO:
 * PID properties loading from xml
 * Refactor PID Pipelines
 * Coordinate system
 * Waypoint guidance
 * */
