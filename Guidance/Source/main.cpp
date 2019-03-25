#include <vector>
#include <windows.h>
#include <iostream>
#include "IPC.h"
#include "Guidance.h"
#include <thread>


using namespace UAV;

bool flag = 1;

void loop2(Guidance* mGuidance) {

    while(flag) {
        mGuidance->CalculateControls();
    }
}

void loop1(Guidance* mGuidance) {
    while(flag) {
        mGuidance->Update();
    }
}

int main() {
    Guidance* mGuidance = Guidance::GetInstance();
    std::thread MyThread1(loop1, mGuidance);
    Sleep(3);
    std::thread MyThread2(loop2, mGuidance);
    while(flag) {
        int a;
        std::cin >> a;
        int b;
        switch(a) {
            case 1:
                flag = 0;
                break;
            case 2: //Switch to VSPD
                std::cin >> b;
                mGuidance->DebugChangeAutopilotVNAVMode(Vspeed, b);
                break;
            case 3:
                std::cin >> b;
                mGuidance->DebugChangeAutopilotLNAVMode(HDGselect, b);
                break;
            case 4:
                std::cin >> b;
                mGuidance->DebugChangeAutopilotVNAVMode(LVLCHNG, b);
                break;
            case 5:
                mGuidance->DebugStartRouteGeneration();
            default:
                break;
        }
    }
    MyThread1.join();
    MyThread2.join();

    return 0;
}

/*TODO:
 * PID properties loading from xml
 * Refactor PID Pipelines
 * Coordinate system
 * Waypoint guidance
 * */
