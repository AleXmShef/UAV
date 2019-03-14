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
    auto xangaccpid = new PID(0.1, 2, -2, 0.3, 0.05, 0.01);
    auto xcorrpid = new PID(0.1, 0.1, -0.1, 0.3, 0.1, 0.01);

    xPids->push_back(xangvelpid);
    xPids->push_back(xangaccpid);
    xPids->push_back(xcorrpid);

    auto yrollangvelpid = new PID(0.1, 6, -6, 0.4, 0.05, 0.01);
    auto yrollangaccpid = new PID(0.1, 2, -2, 0.3, 0.05, 0.01);
    auto ycorrpid = new PID(0.1, 0.5, -0.5, 0.5, 0.1, 0.01);

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

    mValuesForCalculation.insert({"VerticalVelocity", 0});


    //Lock controls
    IPCns::IPC::lock();
    mDataMaps.at(SimData)->at(ControlOverride) = 1;
    mDataMaps.at(SimData)->at(ControlSrfcOverride) = 1;
    IPCns::IPC::unlock();
}

void Guidance::Update() {

    IPCns::IPC::lock();
    time_t CurTime;
    time(&CurTime);
    float dT = CurTime - t2;

    double mass = 910.8;
    double Vv = mDataMaps.at(SimData)->at(VerticalVelocity);
    double Vacc = (Vv - mValuesForCalculation.at("VerticalVelocity"))/dT;
    double Flift = mass*(9.81+Vacc);
    double qs = Flift/GetCl(mDataMaps.at(SimData)->at(PitchAoA));
    double DesiredPitch = GetPitch((mass*9.81)/qs);


    double LogArr[] = {mass, dT, Vv, mValuesForCalculation.at("VerticalVelocity"), Vacc, Flift, qs, DesiredPitch};
    //Guidance::Log(LogArr, 8, 0);
    mValuesForCalculation.at("VerticalVelocity") = Vv;





    //update errors
    double verticalVelocity = -1*mDataMaps.at(SimData)->at(VerticalVelocity);

    std::vector<double> XaxisErrors;
    std::vector<double> YaxisErrors;

    XaxisErrors.push_back(-1*mDataMaps.at(SimData)->at(Pitch));
    //XaxisErrors.push_back(mDataMaps.at(SimData)->at(PitchAoA));
    XaxisErrors.push_back(mDataMaps.at(SimData)->at(PitchAngVel));
    XaxisErrors.push_back(mDataMaps.at(SimData)->at(PitchAngAcc));
    double xarr[4] = {mDataMaps.at(SimData)->at(Pitch), mDataMaps.at(SimData)->at(PitchAngVel), mDataMaps.at(SimData)->at(PitchAngAcc)};
    Guidance::Log(xarr, 3, 0);

    YaxisErrors.push_back(mDataMaps.at(SimData)->at(Roll));
    //YaxisErrors.push_back(mDataMaps.at(SimData)->at(YawAngAcc));
    YaxisErrors.push_back(mDataMaps.at(SimData)->at(RollAngVel));
    YaxisErrors.push_back(mDataMaps.at(SimData)->at(RollAngAcc));

//    //calculate PIDs
    double Xcorr = 0;
    double Ycorr = 0;
    Xcorr = mPIDpipelines.at(X)->Calculate(0, XaxisErrors)/1000;
    Ycorr = mPIDpipelines.at(Y)->Calculate(0, YaxisErrors)*2;
    //update flight controls

//    phi+=0.001;
//    double Xcorr = cos(phi);
//    double Ycorr = sin(phi);
//    mDataMaps.at(ControlsData)->at(LeftAil) = Ycorr*10;
//    mDataMaps.at(ControlsData)->at(RightAil) = -Ycorr*10;
    mDataMaps.at(ControlsData)->at(RightAil) += -1*Ycorr;
    if(mDataMaps.at(ControlsData)->at(RightAil) > 20) {
        mDataMaps.at(ControlsData)->at(RightAil) = 20;
    }
    else if (mDataMaps.at(ControlsData)->at(RightAil) < -20) {
        mDataMaps.at(ControlsData)->at(RightAil) = -20;
    }

    mDataMaps.at(ControlsData)->at(LeftAil) = -1 * mDataMaps.at(ControlsData)->at(RightAil);

    mDataMaps.at(ControlsData)->at(RightElev) += Xcorr;
    if(mDataMaps.at(ControlsData)->at(RightElev) > 4) {
        mDataMaps.at(ControlsData)->at(RightElev) = 4;
    }
    else if (mDataMaps.at(ControlsData)->at(RightElev) < -4) {
        mDataMaps.at(ControlsData)->at(RightElev) = -4;
    }
    mDataMaps.at(ControlsData)->at(LeftElev) = mDataMaps.at(ControlsData)->at(RightElev);
    double vec[6] = {Ycorr, Xcorr, DesiredPitch, mDataMaps.at(SimData)->at(Pitch), mDataMaps.at(SimData)->at(PitchAngVel), mDataMaps.at(SimData)->at(PitchAngAcc)};
    t2 = CurTime;
    //Guidance::Log(vec, 6, 0);
    IPCns::IPC::unlock();
//
    //Sleep(1);

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

double Guidance::GetCl(float Pitch) {
    double c = (1.6 + 0.2)/(16.5 +5);
    return (Pitch + 5)*c - 0.2;
}

double Guidance::GetPitch(float Cl) {
    double c = (16.5 + 5)/(1.6+0.2);
    return (Cl + 0.2)*c - 5;
}
