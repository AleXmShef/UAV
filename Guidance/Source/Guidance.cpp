#include "Guidance.h"

using namespace UAV;

Guidance* Guidance::mInstance = nullptr;

Guidance::Guidance() {
    Init();
}

Guidance* Guidance::GetInstance() {
    if(mInstance==nullptr) {
        mInstance = new Guidance();
        return mInstance;
    }
    return mInstance;
}

void Guidance::Init() {
    //Register PIDpipelines
    auto xPids = new std::vector<PID*>;
    auto yPids = new std::vector<PID*>;

    auto xaoapid = new PID(5, -5);
    auto xangvelpid = new PID(2, -2);
    auto xangaccpid = new PID(2, -2);
    auto xcorrpid = new PID(1, -1);

    xPids->push_back(xaoapid);
    xPids->push_back(xangvelpid);
    xPids->push_back(xangaccpid);
    xPids->push_back(xcorrpid);

    auto yyawangaccpid = new PID(2, -2);
    auto yrollangvelpid = new PID(9, -9);
    auto yrollangaccpid = new PID(2, -2);
    auto ycorrpid = new PID(1, -1);

    yPids->push_back(yyawangaccpid);
    yPids->push_back(yrollangvelpid);
    yPids->push_back(yrollangaccpid);
    yPids->push_back(ycorrpid);

    auto xpipeline = new PIDPipeline(xPids);
    auto ypipeline = new PIDPipeline(yPids);
    mPIDpipelines.insert({X, xpipeline});
    mPIDpipelines.insert({Y, ypipeline});

    IPCns::IPCSharedMap* ptr;
    IPCns::IPCSharedMap* ptr2;

    //Find DataMaps
    IPCns::IPC::GetInstance();
    mDataMaps.insert({SimData, IPCns::IPC::findData(ptr, SHRDOUTPUT_NAME)});     //Inverted
    mDataMaps.insert({ControlsData, IPCns::IPC::findData(ptr2, SHRDINPUT_NAME)});     //Inverted


    //Lock controls
    IPCns::IPC::lock();
    mDataMaps.at(SimData)->at(ControlOverride) = 1;
    mDataMaps.at(SimData)->at(ControlSrfcOverride) = 1;
    IPCns::IPC::unlock();
}

void Guidance::Update() {

    IPCns::IPC::lock();
    //update errors
    double verticalVelocity = -1*mDataMaps.at(SimData)->at(VerticalVelocity);

    std::vector<double> XaxisErrors;
    std::vector<double> YaxisErrors;

    XaxisErrors.push_back(verticalVelocity);
    XaxisErrors.push_back(mDataMaps.at(SimData)->at(PitchAoA));
    XaxisErrors.push_back(mDataMaps.at(SimData)->at(PitchAngVel));
    XaxisErrors.push_back(mDataMaps.at(SimData)->at(PitchAngAcc));
    double xarr[4] = {verticalVelocity, mDataMaps.at(SimData)->at(PitchAoA), mDataMaps.at(SimData)->at(PitchAngVel), mDataMaps.at(SimData)->at(PitchAngAcc)};
    //Guidance::Log(xarr, 4, 0);

    YaxisErrors.push_back(mDataMaps.at(SimData)->at(Roll));
    YaxisErrors.push_back(mDataMaps.at(SimData)->at(YawAngAcc));
    YaxisErrors.push_back(mDataMaps.at(SimData)->at(RollAngVel));
    YaxisErrors.push_back(mDataMaps.at(SimData)->at(RollAngAcc));

//    //calculate PIDs
    double Xcorr = mPIDpipelines.at(X)->Calculate(0, XaxisErrors)*4;
    double Ycorr = mPIDpipelines.at(Y)->Calculate(0, YaxisErrors)*4;
    //update flight controls

//    phi+=0.001;
//    double Xcorr = cos(phi);
//    double Ycorr = sin(phi);
    mDataMaps.at(ControlsData)->at(LeftAil) = Ycorr;
    mDataMaps.at(ControlsData)->at(RightAil) = -Ycorr;
    mDataMaps.at(ControlsData)->at(RightElev) = Xcorr;
    mDataMaps.at(ControlsData)->at(LeftElev) = Xcorr;
    double vec[2] = {Ycorr, Xcorr};
    Guidance::Log(vec, 2, 0);
    IPCns::IPC::unlock();
//
    Sleep(1);

}

void Guidance::MasterSwitch(bool flag) {
    if(flag) {
        mDataMaps.at(SimData)->at(ControlOverride) = 1;
        mDataMaps.at(SimData)->at(ControlSrfcOverride) = 1;
    }
    else {
        mDataMaps.at(SimData)->at(ControlOverride) = 0;
        mDataMaps.at(SimData)->at(ControlSrfcOverride) = 0;
    }
}

void Guidance::Log(double* p, int n, int c) {
    time_t temp;
    time(&temp);
    if (temp > t + 0.1) {
        COORD tl = {(short) c, 0};
        CONSOLE_SCREEN_BUFFER_INFO s;
        HANDLE console = GetStdHandle(STD_OUTPUT_HANDLE);
        GetConsoleScreenBufferInfo(console, &s);
        DWORD written, cells = s.dwSize.X * s.dwSize.Y;
        FillConsoleOutputCharacter(console, ' ', cells, tl, &written);
        FillConsoleOutputAttribute(console, s.wAttributes, cells, tl, &written);
        SetConsoleCursorPosition(console, tl);

        for (int i = 0; i < n; i++) {
            std::cout << p[i] << std::endl;
        }
        time(&t);
    }


}
