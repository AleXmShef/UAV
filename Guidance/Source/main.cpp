#include <vector>
#include <windows.h>
#include <iostream>
#include "IPC.h"
#include "Guidance.h"


using namespace UAV;

int main() {
    Guidance* mGuidance = Guidance::GetInstance();

    while(1) {
        mGuidance->Update();
    }

    return 0;
}

/*TODO:
 * PID properties loading from xml
 * Refactor PID Pipelines
 * Coordinate system
 * Waypoint guidance
 * */

