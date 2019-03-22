#include <vector>
#include <windows.h>
#include <iostream>
#include "IPC.h"
#include "Guidance.h"
#include <thread>


using namespace UAV;

bool flag = 1;

void loop() {
    Guidance* mGuidance = Guidance::GetInstance();

    while(flag) {
        mGuidance->Update();
    }
}

int main() {
    std::thread MyThread(loop);
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
                Guidance::GetInstance()->VNAVmode = Vspeed;
                Guidance::GetInstance()->mAutopilotSettings.VSPD = b;
                break;
            case 3:
                std::cin >> b;
                Guidance::GetInstance()->mAutopilotSettings.HDG = b;
                break;
            case 4:
                std::cin >> b;
                Guidance::GetInstance()->VNAVmode = LVLCHNG;
                Guidance::GetInstance()->mAutopilotSettings.ALT = b;
            case 5:
                Guidance::GetInstance()->SetRoute();
            default:
                break;
        }
    }
    MyThread.join();

    return 0;
}

/*TODO:
 * PID properties loading from xml
 * Refactor PID Pipelines
 * Coordinate system
 * Waypoint guidance
 * */
