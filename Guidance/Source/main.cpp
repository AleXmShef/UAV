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
        if(a) {
            flag = false;
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
