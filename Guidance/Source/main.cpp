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
        x = cos(phi)*10;
        y = sin(phi)*10;
        imap->at(0) = x;
        imap->at(1) = y;
        //imap->at(2) = x;
        imap->at(3) = y;
        Sleep(1);
    }



    return 0;
}

