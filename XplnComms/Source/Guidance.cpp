#include "Guidance.h"

using namespace XCOM;
using namespace IPCns;

Guidance* Guidance::mInstance = nullptr;

Guidance::Guidance() {
    //find datarefs
    mIDataRefs = new std::map<ControlsEnum, XPLMDataRef>;
    mODataRefs = new std::map<SimDataEnum, XPLMDataRef>;

//    mIDataRefs->insert({ControlPitch, XPLMFindDataRef("sim/flightmodel2/wing/elevator1_deg")});     //elevator
//    mIDataRefs->insert({ControlRoll, XPLMFindDataRef("sim/flightmodel2/wing/aileron1_deg")});     //ailerons
    mIDataRefs->insert({ControlPitch, XPLMFindDataRef("sim/flightmodel/controls/elv1_def")});     //elevator
    mIDataRefs->insert({ControlRoll, XPLMFindDataRef("sim/flightmodel/controls/ail1_def")});     //ailerons
    mIDataRefs->insert({ControlYaw, XPLMFindDataRef("sim/flightmodel2/wing/rudder1_deg")});       //rudderw
    mIDataRefs->insert({ControlThrottle, XPLMFindDataRef("sim/flightmodel/engine/ENGN_thro_use")});          //engine throttle

    mODataRefs->insert({Latitude, XPLMFindDataRef("sim/flightmodel/position/latitude")});          //latitude
    mODataRefs->insert({Longitude, XPLMFindDataRef("sim/flightmodel/position/longitude")});        //longitude
    mODataRefs->insert({Pitch, XPLMFindDataRef("sim/flightmodel/position/true_theta")});           //pitch
    mODataRefs->insert({Roll, XPLMFindDataRef("sim/flightmodel/position/true_phi")});              //roll
    mODataRefs->insert({Heading, XPLMFindDataRef("sim/flightmodel/position/true_phi")});              //heading
    mODataRefs->insert({Altitude, XPLMFindDataRef("sim/flightmodel/position/elevation")});            //heading
    mODataRefs->insert({Velocity, XPLMFindDataRef("sim/flightmodel/position/indicated_airspeed")});            //heading
    mODataRefs->insert({PitchAoA, XPLMFindDataRef("sim/flightmodel/position/alpha")});            //heading
    mODataRefs->insert({YawAoA, XPLMFindDataRef("sim/flightmodel/position/beta")});            //heading
    mODataRefs->insert({VerticalPath, XPLMFindDataRef("sim/flightmodel/position/vpath")});            //heading
    mODataRefs->insert({HorizontalPath, XPLMFindDataRef("sim/flightmodel/position/hpath")});            //heading
    mODataRefs->insert({PitchAngVel, XPLMFindDataRef("sim/flightmodel/position/Q")});            //heading
    mODataRefs->insert({RollAngVel, XPLMFindDataRef("sim/flightmodel/position/P")});            //heading
    mODataRefs->insert({YawAngVel, XPLMFindDataRef("sim/flightmodel/position/R")});            //heading
    mODataRefs->insert({PitchAngAcc, XPLMFindDataRef("sim/flightmodel/position/Q_dot")});            //heading
    mODataRefs->insert({RollAngAcc, XPLMFindDataRef("sim/flightmodel/position/P_dot")});            //heading
    mODataRefs->insert({YawAngAcc, XPLMFindDataRef("sim/flightmodel/position/R_dot")});
//    mODataRefs->insert({Heading, XPLMFindDataRef("sim/flightmodel/position/mag_psi")});            //heading
//    mODataRefs->insert({Heading, XPLMFindDataRef("sim/flightmodel/position/mag_psi")});            //heading
//    mODataRefs->insert({Heading, XPLMFindDataRef("sim/flightmodel/position/mag_psi")});            //heading



    mODataRefs->insert({ControlOverride, XPLMFindDataRef("sim/operation/override/override_flightcontrol")});       //flag to override flight controls
    mODataRefs->insert({ControlSrfcOverride, XPLMFindDataRef("sim/operation/override/override_control_surfaces")});       //flag to override control surfaces

    //Initialize data maps
    shared_memory_object::remove(SHRDMEM_NAME);
    shared_memory_object::remove("mtx");
    IPC::GetInstance();
    mOutputMap = IPC::registerData(mOutputMap, SHRDOUTPUT_NAME);
    mInputMap = IPC::registerData(mInputMap, SHRDINPUT_NAME);

    //FUCKING ENUMS
    for(int i = 0; i < ControlSrfcOverride + 1; i++) {
        mOutputMap->insert(std::pair<const int, float>(i, 0));
    }

    for(int i = 0; i < Rudder + 1; i++) {
        mInputMap->insert(std::pair<const int, float>(i, 0));
    }

}

Guidance* Guidance::GetInstance() {
    if(mInstance == nullptr) {
        mInstance = new Guidance();
        return mInstance;
    }
    return mInstance;
}

void Guidance::Update() {
    IPC::lock();

    //Update flight controls
    //See mIDatarefsTypes for switch reference
    auto arr = new float[15];
    for (int j = 0; j < 6; j++) {
        switch (j) {
            case Throttle:
                XPLMSetDataf(mIDataRefs->at(ControlThrottle), mInputMap->at(Throttle));
                break;
            case LeftAil:
                arr[0] = mInputMap->at(LeftAil);
                XPLMSetDatavf(mIDataRefs->at(ControlRoll), arr, 0, 1);
                break;
            case RightAil:
                arr[1] = mInputMap->at(RightAil);
                XPLMSetDatavf(mIDataRefs->at(ControlRoll), arr, 1, 2);
                break;
            case LeftElev:
                arr[8] = mInputMap->at(LeftElev);
                XPLMSetDatavf(mIDataRefs->at(ControlPitch), arr, 8, 9);
                break;
            case RightElev:
                arr[9] = mInputMap->at(RightElev);
                XPLMSetDatavf(mIDataRefs->at(ControlPitch), arr, 9, 10);
                break;
            case Rudder:
                arr[10] = mInputMap->at(Rudder);
                arr[11] = mInputMap->at(Rudder);
                XPLMSetDatavf(mIDataRefs->at(ControlYaw), arr, 10, 11);
                XPLMSetDatavf(mIDataRefs->at(ControlYaw), arr, 11, 12);
                break;
            default:
                break;
        }

    }

    //Update flags
    //FUCKING GRAVE, IM OUT
    XPLMSetDatai(mODataRefs->at(ControlOverride), mOutputMap->at(ControlOverride));
    XPLMSetDatai(mODataRefs->at(ControlSrfcOverride), mOutputMap->at(ControlSrfcOverride));
    //Update flight params
    for (int i = 0; i < 15; i++) {
        mOutputMap->at(i) = XPLMGetDataf(mODataRefs->at((SimDataEnum) i));
    }

    IPC::unlock();
}

Guidance::~Guidance() {
    //destroy pids
}

