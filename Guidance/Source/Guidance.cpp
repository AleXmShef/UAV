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
    //Register PID Pipelines
    auto xPids = new std::vector<PID*>;
    auto yPids = new std::vector<PID*>;

    auto xangvelpid = new PID(0.1, 1, -1, 0.4, 0.5, 0);
    auto xcorrpid = new PID(0.1, 1, -1, 0.7, 0.5, 0.01);
    xPids->push_back(xangvelpid);
    xPids->push_back(xcorrpid);

    auto yangvelpid = new PID(0.1, 2, -2, 0.7, 0.6, 0);
    auto ycorrpid = new PID(0.1, 1.5, -1.5, 0.8, 0.6, 0.01);
    yPids->push_back(yangvelpid);
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

    double mass = mDataMaps.at(DerivedData)->at(Mass);
    double Vv = mDataMaps.at(DerivedData)->at(VerticalVelocity);
    double Vacc = (Vv - mValuesForCalculation.at("VerticalVelocity"))/dT;
    double Flift = mass*(9.81+Vacc);
    double R = 4000;
    double AccCenter = (mDataMaps.at(DerivedData)->at(Velocity)*mDataMaps.at(DerivedData)->at(Velocity))/(R*1.944);
    AccCenter = 0;
    double Fcenter = mass*AccCenter;
    double Flifttarget = mass*9.81;
    double Ftarget = sqrt(Fcenter*Fcenter+Flifttarget*Flifttarget);
    double qs = Flift/GetCl(mDataMaps.at(DerivedData)->at(PitchAoA));
    double DesiredAoA = 0.0001;

    DesiredAoA += GetPitch((float)(Ftarget/qs));
    double DesiredRoll = asin(Fcenter/Ftarget)*180/M_PI;

    double DesiredPitch = 0.0001;
    DesiredPitch += DesiredAoA + (asin(0/(mDataMaps.at(DerivedData)->at(Velocity)*101.269)))*180/M_PI;

    mValuesForCalculation.at("VerticalVelocity") = Vv;

    std::vector<double> XaxisErrors;
    std::vector<double> YaxisErrors;

    XaxisErrors.push_back(mDataMaps.at(DerivedData)->at(Pitch));
    XaxisErrors.push_back(mDataMaps.at(DerivedData)->at(PitchAngVel)-mDataMaps.at(DerivedData)->at(YawAngVel)*sin(mDataMaps.at(DerivedData)->at(Roll)*M_PI/180));

    YaxisErrors.push_back(mDataMaps.at(DerivedData)->at(Roll));
    YaxisErrors.push_back(mDataMaps.at(DerivedData)->at(RollAngVel));

    //calculate PIDs
    double Xcorr = 0;
    double Ycorr = 0;
    Xcorr = mPIDpipelines.at(X)->Calculate(DesiredPitch, XaxisErrors)/500;
    Ycorr = mPIDpipelines.at(Y)->Calculate(DesiredRoll, YaxisErrors)/500;

    double arr[6] = {DesiredRoll, mDataMaps.at(DerivedData)->at(Roll), DesiredPitch, mDataMaps.at(DerivedData)->at(PitchAngVel)-mDataMaps.at(DerivedData)->at(YawAngVel)*sin(mDataMaps.at(DerivedData)->at(Roll)*M_PI/180), mDataMaps.at(DerivedData)->at(Pitch),
                     mDataMaps.at(DerivedData)->at(YawAngVel)};
    Guidance::Log(arr, 6, 0);

    //For testing purposes
//    phi+=0.001;
//    Xcorr = cos(phi);
//    Ycorr = sin(phi);
//    mDataMaps.at(ControlsData)->at(LeftAil) = Ycorr;      //Down
//    mDataMaps.at(ControlsData)->at(RightAil) = Ycorr;     //Up
//    mDataMaps.at(ControlsData)->at(LeftElev) = Xcorr;     //Up
//    mDataMaps.at(ControlsData)->at(RightElev) = Xcorr;    //Up

    //Update flight controls
    mDataMaps.at(ControlsData)->at(LeftAil) += Ycorr;
    if(mDataMaps.at(ControlsData)->at(LeftAil) > 10) {
        mDataMaps.at(ControlsData)->at(LeftAil) = 10;
    }
    else if (mDataMaps.at(ControlsData)->at(LeftAil) < -10) {
        mDataMaps.at(ControlsData)->at(LeftAil) = -10;
    }
    mDataMaps.at(ControlsData)->at(RightAil) = mDataMaps.at(ControlsData)->at(LeftAil);

    mDataMaps.at(ControlsData)->at(RightElev) += -1*(float)Xcorr;
    if(mDataMaps.at(ControlsData)->at(RightElev) > 4) {
        mDataMaps.at(ControlsData)->at(RightElev) = 4;
    }
    else if (mDataMaps.at(ControlsData)->at(RightElev) < -4) {
        mDataMaps.at(ControlsData)->at(RightElev) = -4;
    }
    mDataMaps.at(ControlsData)->at(LeftElev) = mDataMaps.at(ControlsData)->at(RightElev);

    IPCns::IPC::unlock();
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
