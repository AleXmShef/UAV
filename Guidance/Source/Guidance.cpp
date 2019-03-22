#include "Guidance.h"
#include <math.h>
#include <thread>

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
    auto pitchPids = new std::vector<PID*>;
    auto rollPids = new std::vector<PID*>;

    auto pitchAngVelPID = new PID("Pitch Angular Velocity PID", 1, -1, 0.4, 0.5, 0);
    auto pitchCorrPID = new PID("Pitch Control Surfaces PID", 1, -1, 0.7, 0.5, 0.01);
    pitchPids->push_back(pitchAngVelPID);
    pitchPids->push_back(pitchCorrPID);

    auto rollAngVelPID = new PID("Roll Angular Velocity PID", 4, -4, 0.4, 0.5, 0);
    auto rollCorrPID = new PID("Roll Control Surfaces PID", 4, -4, 1.5, 0.3, 0);
    rollPids->push_back(rollAngVelPID);
    rollPids->push_back(rollCorrPID);

    auto HDGselectPID = new PID("Heading Angular Velocity PID", 3, -3, 0.1, 0.2, 0.0001);
    auto LVLchngPID = new PID("Vertical Speed PID", 800, -800, 4, 1.5, 0);

    auto pitchPIDpipeline = new PIDPipeline(pitchPids);
    auto rollPIDpipeline = new PIDPipeline(rollPids);
    mPIDpipelines.insert({PitchPIDpipe, pitchPIDpipeline});
    mPIDpipelines.insert({RollPIDpipe, rollPIDpipeline});

    auto HDGselectPIDpipeline = new PIDPipeline(HDGselectPID);
    mPIDpipelines.insert({HDGselectPIDpipe, HDGselectPIDpipeline});

    auto LVLchngPIDpipeline = new PIDPipeline(LVLchngPID);
    mPIDpipelines.insert({LVLchngPIDpipe, LVLchngPIDpipeline});

    //Find DataMaps
    IPCns::IPCSharedMap* ptr;
    IPCns::IPCSharedMap* ptr2;

    IPCns::IPC::GetInstance();
    mDataMaps.insert({DerivedData, IPCns::IPC::findData(ptr, SHRDOUTPUT_NAME)});     //Inverted
    mDataMaps.insert({ControlsData, IPCns::IPC::findData(ptr2, SHRDINPUT_NAME)});     //Inverted

    mValuesForCalculation.insert({"VerticalVelocity", 0});      ///Temporary

    //Lock controls
    IPCns::IPC::lock();
    mDataMaps.at(DerivedData)->at(ControlOverride) = 1;
    mDataMaps.at(DerivedData)->at(ControlSrfcOverride) = 1;

//    mAutopilotSettings.HDG = mDataMaps.at(DerivedData)->at(Heading);
//    mAutopilotSettings.VSPD = mDataMaps.at(DerivedData)->at(VerticalVelocity);

    IPCns::IPC::unlock();
}

void Guidance::Update() {
    UpdateGuidance();
}

void Guidance::UpdateGuidance() {
    IPCns::IPC::lock();

    //Get elapsed time since last iteration
    time_t CurTime;
    time(&CurTime);
    float dT = CurTime - t2;

    //Get necessary current values
    double mass = mDataMaps.at(DerivedData)->at(Mass);
    double Vv = mDataMaps.at(DerivedData)->at(VerticalVelocity);
    double Vacc = (Vv - mValuesForCalculation.at("VerticalVelocity"))/dT;
    double Flift = mass*(9.81+Vacc);

    //Init target values
    double DesiredVerticalVelocity = 0;
    double DesiredAccCenter = 0;

    //See what do we need to do depending on an autopilot setup
    switch(LNAVmode) {
        case RouteL: {
            double activeWaypointLAT = mWaypoints[0][0];
            double activeWaypointLONG = mWaypoints[1][0];
            double myLongitude = mDataMaps.at(DerivedData)->at(Longitude);
            double myLatitude = mDataMaps.at(DerivedData)->at(Latitude);
            double dLambda = (activeWaypointLONG - myLongitude)*M_PI/180;
            double dPhi = (activeWaypointLAT - myLatitude)*M_PI/180;
            double Azimuth = atan2((sin(dLambda)*cos(activeWaypointLAT)), (cos(myLatitude)*sin(activeWaypointLAT)-sin(myLatitude)*cos(activeWaypointLAT)*cos(dLambda)))*180/M_PI;
            if(Azimuth < 0) {
                Azimuth+=360;
            }
            mAutopilotSettings.HDG = Azimuth;
            mDataMaps.at(DerivedData)->at(WaypointLONG) = activeWaypointLONG;
            mDataMaps.at(DerivedData)->at(WaypointLAT) = activeWaypointLAT;
        }
        case HDGselect: {
            //Get desired angular velocity -> calculate desired AccCenter
            double HDGerror = mAutopilotSettings.HDG;
            if(mAutopilotSettings.HDG - mDataMaps.at(DerivedData)->at(Heading) > 180) {
                HDGerror = mDataMaps.at(DerivedData)->at(Heading) - (360 - (mAutopilotSettings.HDG- mDataMaps.at(DerivedData)->at(Heading)));
            }
            else if(mDataMaps.at(DerivedData)->at(Heading) - mAutopilotSettings.HDG > 180) {
                HDGerror = mDataMaps.at(DerivedData)->at(Heading) + (360 - (mDataMaps.at(DerivedData)->at(Heading) - mAutopilotSettings.HDG));
            }
            double DesiredYawAngVel = mPIDpipelines.at(HDGselectPIDpipe)->Calculate(HDGerror, mDataMaps.at(DerivedData)->at(Heading))*M_PI/180;
            double DesiredCurveR = (mDataMaps.at(DerivedData)->at(Velocity)/1.944)/DesiredYawAngVel;
            DesiredAccCenter = ((mDataMaps.at(DerivedData)->at(Velocity)/1.944)*(mDataMaps.at(DerivedData)->at(Velocity)/1.944))/DesiredCurveR;
            break;
        }
        default:
            break;
    }
    switch(VNAVmode) {
        case ALThold:
            DesiredVerticalVelocity = 0;
            break;
        case LVLCHNG:
            DesiredVerticalVelocity = mPIDpipelines.at(LVLchngPIDpipe)->Calculate(mAutopilotSettings.ALT, mDataMaps.at(DerivedData)->at(Altitude)*3.281);
            //Claculate desired vertical velocity for level change
            break;
        case Vspeed:
            DesiredVerticalVelocity = mAutopilotSettings.VSPD;
            break;
        case RouteV:
            //climb&descent profiles
            break;
        default:
            break;

    }

    double DesiredFcenter = mass*DesiredAccCenter;
    double DesiredFlift = mass*9.81;

    double Ftarget = sqrt(DesiredFcenter*DesiredFcenter+DesiredFlift*DesiredFlift);
    double qs = Flift/GetCl(mDataMaps.at(DerivedData)->at(PitchAoA));

    double DesiredAoA = 0.0001;
    DesiredAoA += GetPitch(Ftarget/qs);

    double DesiredRoll = asin(DesiredFcenter/Ftarget)*180/M_PI;

    double DesiredPitch = 0.0001;
    DesiredPitch += DesiredAoA + (asin(DesiredVerticalVelocity/(mDataMaps.at(DerivedData)->at(Velocity)*101.269)))*180/M_PI;

    mValuesForCalculation.at("VerticalVelocity") = Vv;

    //CalculateControls
    std::vector<double> PitchAxisErrors;
    std::vector<double> RollAxisErrors;

    PitchAxisErrors.push_back(mDataMaps.at(DerivedData)->at(Pitch));
    PitchAxisErrors.push_back(mDataMaps.at(DerivedData)->at(PitchAngVel)-mDataMaps.at(DerivedData)->at(YawAngVel)*sin(mDataMaps.at(DerivedData)->at(Roll)*M_PI/180));

    RollAxisErrors.push_back(mDataMaps.at(DerivedData)->at(Roll));
    RollAxisErrors.push_back(mDataMaps.at(DerivedData)->at(RollAngVel));

    //calculate PIDs
    double PitchCorr = 0;
    double RollCorr = 0;
    PitchCorr = mPIDpipelines.at(PitchPIDpipe)->Calculate(DesiredPitch, PitchAxisErrors)/500;
    RollCorr = mPIDpipelines.at(RollPIDpipe)->Calculate(DesiredRoll, RollAxisErrors)/500;

    double arr[4] = {mDataMaps.at(DerivedData)->at(Heading), mDataMaps.at(DerivedData)->at(YawAngVel)*cos(mDataMaps.at(DerivedData)->at(Roll)), DesiredRoll, mDataMaps.at(DerivedData)->at(Roll)};
    //Guidance::Log(arr, 4, 0);
    Logger::GetInstance()->logConsole();
    UpdateControls(PitchCorr, RollCorr);

    IPCns::IPC::unlock();

    Sleep(1);
}

void Guidance::UpdateControls(double PitchCorr, double RollCorr) {

    //For testing purposes
//    phi+=0.001;
//    Xcorr = cos(phi);
//    Ycorr = sin(phi);
//    mDataMaps.at(ControlsData)->at(LeftAil) = RollCorr;      //Down
//    mDataMaps.at(ControlsData)->at(RightAil) = RollCorr;     //Up
//    mDataMaps.at(ControlsData)->at(LeftElev) = PitchCorr;     //Up
//    mDataMaps.at(ControlsData)->at(RightElev) = PitchCorr;    //Up

    mDataMaps.at(ControlsData)->at(LeftAil) += RollCorr;
    if(mDataMaps.at(ControlsData)->at(LeftAil) > 10) {
        mDataMaps.at(ControlsData)->at(LeftAil) = 10;
    }
    else if (mDataMaps.at(ControlsData)->at(LeftAil) < -10) {
        mDataMaps.at(ControlsData)->at(LeftAil) = -10;
    }
    mDataMaps.at(ControlsData)->at(RightAil) = mDataMaps.at(ControlsData)->at(LeftAil);

    mDataMaps.at(ControlsData)->at(RightElev) += -1*PitchCorr;
    if(mDataMaps.at(ControlsData)->at(RightElev) > 8) {
        mDataMaps.at(ControlsData)->at(RightElev) = 8;
    }
    else if (mDataMaps.at(ControlsData)->at(RightElev) < -8) {
        mDataMaps.at(ControlsData)->at(RightElev) = -8;
    }
    mDataMaps.at(ControlsData)->at(LeftElev) = mDataMaps.at(ControlsData)->at(RightElev);
}

void Guidance::Log(double* p, int n, int c) {
    clock_t temp;
    temp = clock();
    if (temp > t + 5) {
        //clean console
        COORD tl = {(short) c, 0};
        CONSOLE_SCREEN_BUFFER_INFO s;
        HANDLE console = GetStdHandle(STD_OUTPUT_HANDLE);
        GetConsoleScreenBufferInfo(console, &s);
        DWORD written, cells = s.dwSize.X * s.dwSize.Y;
        //FillConsoleOutputCharacter(console, ' ', cells, tl, &written);
        FillConsoleOutputAttribute(console, s.wAttributes, cells, tl, &written);

        //get base

        SetConsoleCursorPosition(console, tl);

        for (int i = 0; i < n; i++) {
            std::cout << p[i] << std::endl;
        }
        t = temp;
    }


}

double Guidance::GetCl(double Pitch) {       ///Fixme
    double c = (1.6 + 0.2)/(16.5 +5);
    return (Pitch + 5)*c - 0.2;
}

double Guidance::GetPitch(double Cl) {
    double c = (16.5 + 5)/(1.6+0.2);
    return (Cl + 0.2)*c - 5;
}

void Guidance::SetRoute() {
    if (mWaypoints.empty()) {
        std::vector<double> lats;
        std::vector<double> longs;
        IPCns::IPC::lock();
        double lat = mDataMaps.at(DerivedData)->at(Latitude);
        double lng = mDataMaps.at(DerivedData)->at(Longitude);
        IPCns::IPC::unlock();

        srand(time(NULL));

        double lat1 = lat + (double)((double)(rand() % 3-3)/10);
        lats.push_back(lat1);
        double lng1 = lng + (double)((double)(rand() % 3-3)/10);
        longs.push_back(lng1);

        double lat2 = lat1 + (double)((double)(rand() % 3-3)/10);
        lats.push_back(lat2);
        double lng2 = lng1 + (double)((double)(rand() % 3-3)/10);
        longs.push_back(lng2);

        double lat3 = lat2 + (double)((double)(rand() % 3-3)/10);
        lats.push_back(lat3);
        double lng3 = lng2 + (double)((double)(rand() % 3-3)/10);
        longs.push_back(lng3);

        mWaypoints.push_back(lats);
        mWaypoints.push_back(longs);

        LNAVmode = RouteL;
    }
}
