#include "Guidance.h"
#include <math.h>

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

    auto xangvelpid = new PID(0.1, 4, -4, 0.4, 0.05, 0.01);
    auto xangaccpid = new PID(0.1, 1, -1, 0.4, 0.2, 0);
    auto xcorrpid = new PID(0.1, 1, -1, 0.8, 0.5, 0.01);

    xPids->push_back(xangvelpid);
    xPids->push_back(xangaccpid);
    xPids->push_back(xcorrpid);

    auto yrollangvelpid = new PID(0.1, 6, -6, 0.4, 0.05, 0.01);
    auto yrollangaccpid = new PID(0.1, 3, -3, 0.5, 0.2, 0);
    auto ycorrpid = new PID(0.1, 1, -1, 1, 0.3, 0.01);

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
    mDataMaps.insert({DerivedData, IPCns::IPC::findData(ptr, SHRDOUTPUT_NAME)});     //Inverted
    mDataMaps.insert({ControlsData, IPCns::IPC::findData(ptr2, SHRDINPUT_NAME)});     //Inverted

    mValuesForCalculation.insert({"VerticalVelocity", 0});


    //Lock controls
    IPCns::IPC::lock();
    mDataMaps.at(DerivedData)->at(ControlOverride) = 1;
    mDataMaps.at(DerivedData)->at(ControlSrfcOverride) = 1;
    IPCns::IPC::unlock();
}

void Guidance::Update() {

    IPCns::IPC::lock();
    time_t CurTime;
    time(&CurTime);
    float dT = CurTime - t2;

    double mass = 910.8;
    double Vv = mDataMaps.at(DerivedData)->at(VerticalVelocity);
    double Vacc = (Vv - mValuesForCalculation.at("VerticalVelocity"))/dT;
    double Flift = mass*(9.81+Vacc);
    double qs = Flift/GetCl(mDataMaps.at(DerivedData)->at(PitchAoA));
    double DesiredPitch = GetPitch((mass*9.81)/qs);


    double LogArr[] = {mass, dT, Vv, mValuesForCalculation.at("VerticalVelocity"), Vacc, Flift, qs, DesiredPitch};
    //Guidance::Log(LogArr, 8, 0);
    mValuesForCalculation.at("VerticalVelocity") = Vv;

//    std::vector<double> XaxisErrors;
//    std::vector<double> YaxisErrors;
//
//    XaxisErrors.push_back(-1*mDataMaps.at(DerivedData)->at(Pitch));
//    XaxisErrors.push_back(mDataMaps.at(DerivedData)->at(PitchAngVel));
//    XaxisErrors.push_back(mDataMaps.at(DerivedData)->at(PitchAngAcc));
//    double xarr[4] = {mDataMaps.at(DerivedData)->at(Pitch), mDataMaps.at(DerivedData)->at(PitchAngVel), mDataMaps.at(DerivedData)->at(PitchAngAcc)};
//    Guidance::Log(xarr, 3, 0);
//
//    YaxisErrors.push_back(mDataMaps.at(DerivedData)->at(Roll));
//    YaxisErrors.push_back(mDataMaps.at(DerivedData)->at(RollAngVel));
//    YaxisErrors.push_back(mDataMaps.at(DerivedData)->at(RollAngAcc));
//
//    //calculate PIDs
//    double Xcorr = 0;
//    double Ycorr = 0;
//    Xcorr = mPIDpipelines.at(X)->Calculate(0, XaxisErrors)/1000;
//    Ycorr = mPIDpipelines.at(Y)->Calculate(0, YaxisErrors)*2;
    double Xtest = mPIDpipelines.at(X)->GetPID(1)->calculate(0, mDataMaps.at(DerivedData)->at(Pitch));
    double Ytest = mPIDpipelines.at(Y)->GetPID(1)->calculate(0, mDataMaps.at(DerivedData)->at(Roll));
    double Xcorr = mPIDpipelines.at(X)->GetPID(2)->calculate(Xtest, mDataMaps.at(DerivedData)->at(PitchAngVel))/500;
    double Ycorr = mPIDpipelines.at(Y)->GetPID(2)->calculate(Ytest, mDataMaps.at(DerivedData)->at(RollAngVel))/500;
//    if (mDataMaps.at(DerivedData)->at(PitchAngVel) < 0) {
//        Xcorr*=-1;
//    }
    double arr[4] = {Ycorr*500, mDataMaps.at(DerivedData)->at(RollAngVel)};
    Guidance::Log(arr, 2, 0);
    //update flight controls

//    phi+=0.001;
//    double Xcorr = cos(phi);
//    double Ycorr = sin(phi);
//    mDataMaps.at(ControlsData)->at(LeftAil) = Ycorr*10;
//    mDataMaps.at(ControlsData)->at(RightAil) = -Ycorr*10;
    mDataMaps.at(ControlsData)->at(RightAil) += -1*Ycorr;
    if(mDataMaps.at(ControlsData)->at(RightAil) > 10) {
        mDataMaps.at(ControlsData)->at(RightAil) = 10;
    }
    else if (mDataMaps.at(ControlsData)->at(RightAil) < -10) {
        mDataMaps.at(ControlsData)->at(RightAil) = -10;
    }
    mDataMaps.at(ControlsData)->at(LeftAil) = -1 * mDataMaps.at(ControlsData)->at(RightAil);

    mDataMaps.at(ControlsData)->at(RightElev) += -1*(float)Xcorr;
    if(mDataMaps.at(ControlsData)->at(RightElev) > 4) {
        mDataMaps.at(ControlsData)->at(RightElev) = 4;
    }
    else if (mDataMaps.at(ControlsData)->at(RightElev) < -4) {
        mDataMaps.at(ControlsData)->at(RightElev) = -4;
    }
    mDataMaps.at(ControlsData)->at(LeftElev) = mDataMaps.at(ControlsData)->at(RightElev);
    double vec[6] = {Ycorr, Xcorr, DesiredPitch, mDataMaps.at(DerivedData)->at(Pitch), mDataMaps.at(DerivedData)->at(PitchAngVel), mDataMaps.at(DerivedData)->at(PitchAngAcc)};
    t2 = CurTime;
    //Guidance::Log(vec, 6, 0);
    IPCns::IPC::unlock();
//
    Sleep(1);

}

void Guidance::MasterSwitch(bool flag) {
    if(flag) {
        mDataMaps.at(DerivedData)->at(ControlOverride) = 1;
        mDataMaps.at(DerivedData)->at(ControlSrfcOverride) = 1;
    }
    else {
        mDataMaps.at(DerivedData)->at(ControlOverride) = 0;
        mDataMaps.at(DerivedData)->at(ControlSrfcOverride) = 0;
    }
}

void Guidance::Log(double* p, int n, int c) {
    clock_t temp;
    temp = clock();
    if (temp > t + 5) {
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
        t = temp;
    }


}

double Guidance::GetCl(float Pitch) {
    double c = (1.6 + 0.2)/(16.5 +5);
    return (Pitch + 5)*c - 0.2;
}

double Guidance::GetPitch(float Cl) {
    double c = (16.5 + 5)/(1.6+0.2);
    return (Cl + 0.2)*c - 5;
}
