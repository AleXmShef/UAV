#include "IPC.h"
#include <vector>
#include "windows.h"

int main() {
    std::vector<std::string> mIDataRefsTypes = {"YokePitch", "YokeRoll", "YokeYaw", "Throttle"};
    UAV::IPC::GetInstance();
    UAV::IPCSharedMap* imap;
    imap = UAV::IPC::findData(imap, SHRDINPUT_NAME);
    double phi = 0;
    float x;
    float y;

    while(1) {
        phi+= 0.01;
        x = cos(phi);
        y = sin(phi);
        imap->at(0) = x;
        imap->at(1) = y;
        Sleep(0.1);
    }



    return 0;
}

