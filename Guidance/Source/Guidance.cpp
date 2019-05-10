#include "Guidance.h"
#include <math.h>
#include <thread>

using namespace UAV;

Guidance* Guidance::mInstance = nullptr;

Guidance::Guidance() {
}

Guidance* Guidance::GetInstance() {
    if(mInstance==nullptr) {
        mInstance = new Guidance();
        mInstance->mName = "Core Guidance";
        Logger::GetInstance()->registerLoggable(mInstance);
        mInstance->Init();
    }
    return mInstance;
}

void Guidance::Init() {
    //Register PID Pipelines
    Logger::GetInstance()->logIntoMainLogFile(this, "Test");
    auto pitchPids = new std::vector<PID*>;
    auto rollPids = new std::vector<PID*>;
    auto throttlePids = new std::vector<PID*>;

    auto pitchAngVelPID = new PID("Pitch Angular Velocity PID", 1, -1, 0.4, 0.5, 0);
    auto pitchCorrPID = new PID("Pitch Control Surfaces PID", 1, -1, 0.7, 0.5, 0.01);
    pitchPids->push_back(pitchAngVelPID);
    pitchPids->push_back(pitchCorrPID);

    auto rollAngVelPID = new PID("Roll Angular Velocity PID", 4, -4, 0.4, 0.5, 0);
    auto rollCorrPID = new PID("Roll Control Surfaces PID", 4, -4, 1.5, 0.3, 0);
    rollPids->push_back(rollAngVelPID);
    rollPids->push_back(rollCorrPID);

    auto throttleVelPID = new PID("Throttle Acceleration PID", 10, -10, 0.4, 0.5, 0);
    auto throttleCorrPID = new PID("Throttle Control Lever PID", 0.5, -0.5, 0.4, 0.5, 0);
    throttlePids->push_back(throttleVelPID);
    throttlePids->push_back(throttleCorrPID);

    auto HDGselectPID = new PID("Heading Angular Velocity PID", 3, -3, 0.1, 0.2, 0);
    auto LVLchngPID = new PID("Vertical Speed PID", 800, -800, 4, 1.5, 0);

    auto pitchPIDpipeline = new PIDPipeline(pitchPids);
    auto rollPIDpipeline = new PIDPipeline(rollPids);
    auto throttlePIDpipeline = new PIDPipeline(throttlePids);
    mVariables.controlCalculation.mPIDpipelines.insert({PitchPIDpipe, pitchPIDpipeline});
    mVariables.controlCalculation.mPIDpipelines.insert({RollPIDpipe, rollPIDpipeline});
    mVariables.controlCalculation.mPIDpipelines.insert({ThrottlePIDpipe, throttlePIDpipeline});

    auto HDGselectPIDpipeline = new PIDPipeline(HDGselectPID);
    mVariables.controlCalculation.mPIDpipelines.insert({HDGselectPIDpipe, HDGselectPIDpipeline});

    auto LVLchngPIDpipeline = new PIDPipeline(LVLchngPID);
    mVariables.controlCalculation.mPIDpipelines.insert({LVLchngPIDpipe, LVLchngPIDpipeline});

    std::vector<double> PitchAxisErrors;
    std::vector<double> RollAxisErrors;

    mVariables.controlCalculation.mPIDpipelinesErrors.insert({PitchPIDpipe, PitchAxisErrors});
    mVariables.controlCalculation.mPIDpipelinesErrors.insert({RollPIDpipe, RollAxisErrors});

    //Find DataMaps
    IPCns::IPCSharedMap* ptr;
    IPCns::IPCSharedMap* ptr2;

    IPCns::IPC::GetInstance();
    mVariables.telemetry.mDataMaps.insert({DerivedData, IPCns::IPC::findData(ptr, SHRDOUTPUT_NAME)});     //Inverted
    mVariables.telemetry.mDataMaps.insert({ControlsData, IPCns::IPC::findData(ptr2, SHRDINPUT_NAME)});     //Inverted

    mVariables.mRoute = Route::GetInstance();

    mVariables.controlCalculation.mPIDpipelinesErrors.at(PitchPIDpipe).resize(2);
    mVariables.controlCalculation.mPIDpipelinesErrors.at(RollPIDpipe).resize(2);

    //Lock controls
    IPCns::IPC::lock();
    mVariables.telemetry.mDataMaps.at(DerivedData)->at(ControlOverride) = 1;
    mVariables.telemetry.mDataMaps.at(DerivedData)->at(ControlSrfcOverride) = 1;
    IPCns::IPC::unlock();
}

void Guidance::UpdateTelemetry() {
    IPCns::IPC::lock();

    clock_t temp = clock();
    mVariables.timePassed = (double)(temp - mVariables.clockPassed)/CLOCKS_PER_SEC;
    mVariables.clockPassed = temp;

    mVariables.telemetry.latitude =  mVariables.telemetry.mDataMaps.at(DerivedData)->at(Latitude);
    mVariables.telemetry.longitude =  mVariables.telemetry.mDataMaps.at(DerivedData)->at(Longitude);

    mVariables.telemetry.pitch =  mVariables.telemetry.mDataMaps.at(DerivedData)->at(Pitch);
    mVariables.telemetry.roll =  mVariables.telemetry.mDataMaps.at(DerivedData)->at(Roll);
    mVariables.telemetry.heading =  mVariables.telemetry.mDataMaps.at(DerivedData)->at(Heading);

    mVariables.telemetry.altitudeM =  mVariables.telemetry.mDataMaps.at(DerivedData)->at(Altitude);
    mVariables.telemetry.altitudeF =  mVariables.telemetry.mDataMaps.at(DerivedData)->at(Altitude)*3.281;
    mVariables.telemetry.IAS =  mVariables.telemetry.mDataMaps.at(DerivedData)->at(Velocity);

    mVariables.telemetry.pitchAoA =  mVariables.telemetry.mDataMaps.at(DerivedData)->at(PitchAoA);
    mVariables.telemetry.yawAoA =  mVariables.telemetry.mDataMaps.at(DerivedData)->at(YawAoA);

    //mVariables.telemetry.IAsAcceleration =  mVariables.telemetry.mDataMaps.at(DerivedData)->at(Latitude);
    mVariables.telemetry.verticalAccelereation =  (mVariables.telemetry.mDataMaps.at(DerivedData)->at(VerticalVelocity)/196.85 - mVariables.telemetry.verticalVelocityMPS)/mVariables.timePassed;
    //mVariables.telemetry.lateralAcceleration =  mVariables.telemetry.mDataMaps.at(DerivedData)->at(Latitude);

    mVariables.telemetry.verticalPath =  mVariables.telemetry.mDataMaps.at(DerivedData)->at(VerticalPath);
    mVariables.telemetry.horizontalPath =  mVariables.telemetry.mDataMaps.at(DerivedData)->at(HorizontalPath);
    mVariables.telemetry.verticalVelocityFPM =  mVariables.telemetry.mDataMaps.at(DerivedData)->at(VerticalVelocity);
    mVariables.telemetry.verticalVelocityMPS =  mVariables.telemetry.mDataMaps.at(DerivedData)->at(VerticalVelocity)/196.85;
    //mVariables.telemetry.lateralVelocity =  mVariables.telemetry.mDataMaps.at(DerivedData)->at(Latitude);

    mVariables.telemetry.pitchAngularVelocity =  mVariables.telemetry.mDataMaps.at(DerivedData)->at(PitchAngVel);
    mVariables.telemetry.rollAngularVelocity =  mVariables.telemetry.mDataMaps.at(DerivedData)->at(RollAngVel);
    mVariables.telemetry.yawAngularVelocity =  mVariables.telemetry.mDataMaps.at(DerivedData)->at(YawAngVel);

    mVariables.telemetry.pitchAngularAcceleration =  mVariables.telemetry.mDataMaps.at(DerivedData)->at(PitchAngAcc);
    mVariables.telemetry.rollAngularAcceleration =  mVariables.telemetry.mDataMaps.at(DerivedData)->at(RollAngAcc);
    mVariables.telemetry.yawAngularAcceleration =  mVariables.telemetry.mDataMaps.at(DerivedData)->at(YawAngAcc);

    mVariables.telemetry.massCurrent =  mVariables.telemetry.mDataMaps.at(DerivedData)->at(Mass);
    //mVariables.telemetry.massFuel =  mVariables.telemetry.mDataMaps.at(DerivedData)->at(Latitude);

    IPCns::IPC::unlock();

    Sleep(1);
}

void Guidance::UpdateGuidance() {

    UpdateTelemetry();

    double Flift = mVariables.telemetry.massCurrent*(9.81+mVariables.telemetry.verticalAccelereation);

    //Init target values
    double DesiredVerticalVelocity = 0;
    double DesiredAccCenter = 0;

    //See what do we need to do depending on an autopilot setup
    switch(mVariables.autopilotSettings.LNAVmode) {
        case RouteL: {
            if(mVariables.mRoute->isEmpty()) {
                SetRoute();
            }
            double activeWaypointLAT = mVariables.mRoute->getActiveWaypoint().getLocation()[0]*M_PI/180;
            double activeWaypointLONG = mVariables.mRoute->getActiveWaypoint().getLocation()[1]*M_PI/180;
            double myLongitude = mVariables.telemetry.longitude*M_PI/180;
            double myLatitude = mVariables.telemetry.latitude*M_PI/180;
            if(abs(activeWaypointLAT - myLatitude) < 0.005*M_PI/180 && abs(activeWaypointLONG - myLongitude) < 0.005*M_PI/180){
                mVariables.mRoute->toNextWaypoint();
                if(mVariables.mRoute->isEmpty()) {
                    SetRoute();
                }
                activeWaypointLAT = mVariables.mRoute->getActiveWaypoint().getLocation()[0]*M_PI/180;
                activeWaypointLONG = mVariables.mRoute->getActiveWaypoint().getLocation()[1]*M_PI/180;
            }
            double dLambda = (activeWaypointLONG - myLongitude);
            double dPhi = (activeWaypointLAT - myLatitude);
            double Azimuth = atan2((sin(dLambda)*cos(activeWaypointLAT)), (cos(myLatitude)*sin(activeWaypointLAT)-sin(myLatitude)*cos(activeWaypointLAT)*cos(dLambda)))*180/M_PI;
            if(Azimuth < 0) {
                Azimuth+=360;
            }
            mVariables.autopilotSettings.HDG = Azimuth;
            mVariables.telemetry.mDataMaps.at(DerivedData)->at(WaypointLONG) = mVariables.mRoute->getActiveWaypoint().getLocation()[1];
            mVariables.telemetry.mDataMaps.at(DerivedData)->at(WaypointLAT) = mVariables.mRoute->getActiveWaypoint().getLocation()[0];
        }
        case HDGselect: {
            //Get desired angular velocity -> calculate desired AccCenter
            double HDGerror = mVariables.autopilotSettings.HDG;
            if( mVariables.autopilotSettings.HDG - mVariables.telemetry.heading > 180) {
                HDGerror = mVariables.telemetry.heading - (360 - ( mVariables.autopilotSettings.HDG- mVariables.telemetry.heading));
            }
            else if(mVariables.telemetry.heading - mVariables.autopilotSettings.HDG > 180) {
                HDGerror = mVariables.telemetry.heading + (360 - (mVariables.telemetry.heading -  mVariables.autopilotSettings.HDG));
            }
            double DesiredYawAngVel = mVariables.controlCalculation.mPIDpipelines.at(HDGselectPIDpipe)->Calculate(HDGerror, mVariables.telemetry.heading)*M_PI/180;
            double DesiredCurveR = (mVariables.telemetry.IAS/1.944)/DesiredYawAngVel;
            DesiredAccCenter = ((mVariables.telemetry.IAS/1.944)*(mVariables.telemetry.IAS/1.944))/DesiredCurveR;
            break;
        }
        default:
            break;
    }
    switch(mVariables.autopilotSettings.VNAVmode) {
        case ALThold:
            DesiredVerticalVelocity = 0;
            break;
        case LVLCHNG:
            DesiredVerticalVelocity = mVariables.controlCalculation.mPIDpipelines.at(LVLchngPIDpipe)->Calculate(mVariables.autopilotSettings.ALT, mVariables.telemetry.altitudeF);
            //Claculate desired vertical velocity for level change
            break;
        case Vspeed:
            DesiredVerticalVelocity = mVariables.autopilotSettings.VSPD;
            break;
        case RouteV:
            //climb&descent profiles
            break;
        default:
            break;

    }

    double DesiredFcenter = mVariables.telemetry.massCurrent*DesiredAccCenter;
    double DesiredFlift = mVariables.telemetry.massCurrent*9.81;

    double Ftarget = sqrt(DesiredFcenter*DesiredFcenter+DesiredFlift*DesiredFlift);
    double qs = Flift/GetCl(mVariables.telemetry.pitchAoA);

    double DesiredAoA = 0.0001;
    DesiredAoA += GetPitch(Ftarget/qs);

    mVariables.controlCalculation.desiredRoll = asin(DesiredFcenter/Ftarget)*180/M_PI;

    mVariables.controlCalculation.desiredPitch = 0.0001;
    mVariables.controlCalculation.desiredPitch += DesiredAoA + (asin(DesiredVerticalVelocity/(mVariables.telemetry.IAS*101.269)))*180/M_PI;

    mVariables.controlCalculation.mPIDpipelinesErrors.at(PitchPIDpipe)[0] = (mVariables.telemetry.pitch);
    mVariables.controlCalculation.mPIDpipelinesErrors.at(PitchPIDpipe)[1] = (mVariables.telemetry.pitchAngularVelocity-mVariables.telemetry.yawAngularVelocity
            *sin(mVariables.telemetry.roll*M_PI/180));

    mVariables.controlCalculation.mPIDpipelinesErrors.at(RollPIDpipe)[0] = (mVariables.telemetry.roll);
    mVariables.controlCalculation.mPIDpipelinesErrors.at(RollPIDpipe)[1] = (mVariables.telemetry.rollAngularVelocity);

    Logger::GetInstance()->logConsole();
}

void Guidance::CalculateControls() {

    mVariables.controlCalculation.pitchCorr = mVariables.controlCalculation.mPIDpipelines.at(PitchPIDpipe)->Calculate( mVariables.controlCalculation.desiredPitch, mVariables.controlCalculation.mPIDpipelinesErrors.at(PitchPIDpipe))/500;
    mVariables.controlCalculation.rollCorr = mVariables.controlCalculation.mPIDpipelines.at(RollPIDpipe)->Calculate( mVariables.controlCalculation.desiredRoll, mVariables.controlCalculation.mPIDpipelinesErrors.at(RollPIDpipe))/500;
    mVariables.controlCalculation.throttleCorr = mVariables.controlCalculation.mPIDpipelines.at(ThrottlePIDpipe)->Calculate(mVariables.autopilotSettings.SPD, mVariables.telemetry.IAS)/1000;
    //Sleep(1);
}

void Guidance::UpdateControls() {

    //For testing purposes
//    phi+=0.001;
//    Xcorr = cos(phi);
//    Ycorr = sin(phi);
//    mDataMaps.at(ControlsData)->at(LeftAil) = RollCorr;      //Down
//    mDataMaps.at(ControlsData)->at(RightAil) = RollCorr;     //Up
//    mDataMaps.at(ControlsData)->at(LeftElev) = PitchCorr;     //Up
//    mDataMaps.at(ControlsData)->at(RightElev) = PitchCorr;    //Up

    double RollCorr = mVariables.controlCalculation.rollCorr;
    double PitchCorr = mVariables.controlCalculation.pitchCorr;

    IPCns::IPC::lock();

    mVariables.telemetry.mDataMaps.at(ControlsData)->at(LeftAil) += RollCorr;
    if( mVariables.telemetry.mDataMaps.at(ControlsData)->at(LeftAil) > 10) {
        mVariables.telemetry.mDataMaps.at(ControlsData)->at(LeftAil) = 10;
    }
    else if ( mVariables.telemetry.mDataMaps.at(ControlsData)->at(LeftAil) < -10) {
        mVariables.telemetry.mDataMaps.at(ControlsData)->at(LeftAil) = -10;
    }
    mVariables.telemetry.mDataMaps.at(ControlsData)->at(RightAil) =  mVariables.telemetry.mDataMaps.at(ControlsData)->at(LeftAil);

    mVariables.telemetry.mDataMaps.at(ControlsData)->at(RightElev) += -1*PitchCorr;
    if( mVariables.telemetry.mDataMaps.at(ControlsData)->at(RightElev) > 8) {
        mVariables.telemetry.mDataMaps.at(ControlsData)->at(RightElev) = 8;
    }
    else if ( mVariables.telemetry.mDataMaps.at(ControlsData)->at(RightElev) < -8) {
        mVariables.telemetry.mDataMaps.at(ControlsData)->at(RightElev) = -8;
    }
    mVariables.telemetry.mDataMaps.at(ControlsData)->at(LeftElev) = mVariables.telemetry.mDataMaps.at(ControlsData)->at(RightElev);

    if(mVariables.autopilotSettings.ATarm)
        mVariables.telemetry.mDataMaps.at(ControlsData)->at(Throttle) += mVariables.controlCalculation.throttleCorr;

    IPCns::IPC::unlock();

    Sleep(1);
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

        std::vector<double> lats;
        std::vector<double> longs;
        double lat =  mVariables.telemetry.latitude;
        double lng =  mVariables.telemetry.longitude;

        srand(time(NULL));

        double lat1 = lat -0.1 + ((double)(rand() % 2)/10);
        double lng1 = lng -0.1 +((double)(rand() % 2)/10);
        Waypoint w1(lat1, lng1);
        mVariables.mRoute->addWaypoint(w1);

        double lat2 = lat1 -0.1 +((double)(rand() % 2)/10);
        double lng2 = lng1 -0.1 +((double)(rand() % 2)/10);
        Waypoint w2(lat2, lng2);
        mVariables.mRoute->addWaypoint(w2);

        double lat3 = lat2 -0.1 +((double)(rand() % 2)/10);
        double lng3 = lng2 -0.1 +((double)(rand() % 2)/10);
        Waypoint w3(lat3, lng3);
        mVariables.mRoute->addWaypoint(w3);

        mVariables.autopilotSettings.LNAVmode = RouteL;
}

void Guidance::_debug_ChangeAutopilotLNAVMode(UAV::LNAVmodes mode, double value) {
    switch(mode) {
        case HDGselect:
            mVariables.autopilotSettings.LNAVmode = mode;
            mVariables.autopilotSettings.HDG = value;
            break;
        default:
            break;
    }
}

void Guidance::_debug_ChangeAutopilotVNAVMode(UAV::VNAVmodes mode, double value) {
    switch(mode) {
        case Vspeed:
            mVariables.autopilotSettings.VNAVmode = mode;
            mVariables.autopilotSettings.VSPD = value;
            break;
        case ALThold:
            mVariables.autopilotSettings.VNAVmode = mode;
            break;
        case LVLCHNG:
            mVariables.autopilotSettings.VNAVmode = mode;
            mVariables.autopilotSettings.ALT = value;
            break;
        default:
            break;
    }
}

void Guidance::_debug_ToggleAutopilotATarm() {
    if (mVariables.autopilotSettings.ATarm) {
        mVariables.autopilotSettings.ATarm = false;
    } else
        mVariables.autopilotSettings.ATarm = true;
}

void Guidance::_debug_StartRouteGeneration() {
    SetRoute();
}

std::map<std::string, double>* Guidance::getLogInfo() {
    auto tmap = new std::map<std::string, double>;
    tmap->insert({"Current autopilot LNAV mode", mVariables.autopilotSettings.LNAVmode});
    tmap->insert({"Current autopilot VNAV mode", mVariables.autopilotSettings.VNAVmode});
    tmap->insert({"Heading", mVariables.telemetry.heading});
    tmap->insert({"Altitude", mVariables.telemetry.altitudeM});
    tmap->insert({"Speed", mVariables.telemetry.IAS});
    tmap->insert({"Pitch", mVariables.telemetry.pitch});
    tmap->insert({"Roll", mVariables.telemetry.roll});
    tmap->insert({"Mass", mVariables.telemetry.massCurrent});
    tmap->insert({"Time per cycle", mVariables.timePassed});
    return tmap;
}

void Guidance::Run() {
    mVariables.MasterSwitch = true;
    auto updateThread = new std::thread(&Guidance::_th_UpdateGuidance, this);
    auto calculateControlsThread = new std::thread(&Guidance::_th_CalculateControls, this);
    auto updateControlsThread = new std::thread(&Guidance::_th_UpdateControls, this);
    mVariables.threadArray.push_back(updateThread);
    mVariables.threadArray.push_back(calculateControlsThread);
    mVariables.threadArray.push_back(updateControlsThread);
}

void Guidance::Stop() {
    mVariables.MasterSwitch = false;
    Logger::GetInstance()->logIntoMainLogFile(NULL, "Terminate");
    if(!mVariables.threadArray.empty()) {
        for (int i = 0; i < mVariables.threadArray.size(); i++) {
            mVariables.threadArray[i]->join();
        }
        mVariables.threadArray.clear();
    }

}

void Guidance::_th_UpdateControls() {
    while(mVariables.MasterSwitch) {
        UpdateControls();
    }
}

void Guidance::_th_CalculateControls() {
    while(mVariables.MasterSwitch) {
        CalculateControls();
    }
}

void Guidance::_th_UpdateGuidance() {
    while(mVariables.MasterSwitch) {
        UpdateGuidance();
    }
}
