#include "IPC.h"
#include <vector>
#include "windows.h"

int main() {
    enum FlightControls {Throttle, LeftAil, RightAil, LeftElev, RightElev, Rudder};
    IPCns::IPC::GetInstance();
    IPCns::IPCSharedMap* imap;
    imap = IPCns::IPC::findData(imap, SHRDINPUT_NAME);
    double phi = 0;
    float x;
    float y;

    while(1) {
        phi+= 0.01;
        x = cos(phi)*50;
        y = sin(phi)*50;
        imap->at(LeftAil) = x;
        imap->at(RightAil) = -x;
        imap->at(LeftElev) = y;
        imap->at(RightElev) = y;
        imap->at(Rudder) = y;
        Sleep(1);
    }



    return 0;
}

