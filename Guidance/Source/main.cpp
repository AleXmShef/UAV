#include <vector>
#include <windows.h>
#include <iostream>
#include "IPC.h"
#include "Guidance.h"


using namespace UAV;

int main() {
    int a = 5;
    Guidance* mGuidance = Guidance::GetInstance();
    //mGuidance->MasterSwitch(1);
//    double* arr = new double(2);
//    arr[0] = 0;
//    arr[1] = 25;

    while(1) {
        mGuidance->Update();
    }

    return 0;
}

