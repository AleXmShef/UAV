#include "Guidance.h"

using namespace XCOM;
using namespace IPCns;

Guidance* Guidance::mInstance = nullptr;

Guidance::Guidance() {
    //find datarefs
    mIDataRefs = new std::map<ControlsDataEnum, XPLMDataRef>;
    mODataRefs = new std::map<SimDataEnum, XPLMDataRef>;

    mIDataRefs->insert({LeftElev, XPLMFindDataRef("sim/flightmodel/controls/hstab1_elv1def")});     //elevators
    mIDataRefs->insert({RightElev, XPLMFindDataRef("sim/flightmodel/controls/hstab1_elv2def")});
    mIDataRefs->insert({LeftAil, XPLMFindDataRef("sim/flightmodel/controls/wing2l_ail1def")});     //ailerons
    mIDataRefs->insert({RightAil, XPLMFindDataRef("sim/flightmodel/controls/wing1l_ail1def")});
    mIDataRefs->insert({Rudder, XPLMFindDataRef("sim/flightmodel/controls/vstab1_rud1def")});       //rudderw
    mIDataRefs->insert({Throttle, XPLMFindDataRef("sim/flightmodel/engine/ENGN_thro")});          //engine throttle

    mODataRefs->insert({Latitude, XPLMFindDataRef("sim/flightmodel/position/latitude")});          //latitude
    mODataRefs->insert({Longitude, XPLMFindDataRef("sim/flightmodel/position/longitude")});        //longitude
    mODataRefs->insert({Pitch, XPLMFindDataRef("sim/flightmodel/position/true_theta")});           //pitch
    mODataRefs->insert({Roll, XPLMFindDataRef("sim/flightmodel/position/true_phi")});              //roll
    mODataRefs->insert({Heading, XPLMFindDataRef("sim/flightmodel/position/true_psi")});              //heading
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
    mODataRefs->insert({VerticalVelocity, XPLMFindDataRef("sim/flightmodel/position/vh_ind_fpm")});
    mODataRefs->insert({Mass, XPLMFindDataRef("sim/flightmodel/weight/m_total")});
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
    if(mOutputMap->at(ControlSrfcOverride) == 1 && mOutputMap->at(ControlOverride) == 1) {
        auto arr = new float[56];
        for (int i = 0; i < 56; i++) {
            arr[i] = 0;
        }
        for (int j = 0; j < 6; j++) {
            switch (j) {
                case Throttle: {
                    float *arr2 = new float[8];
                    arr2[0] = mInputMap->at(Throttle);
                    XPLMSetDatavf(mIDataRefs->at(Throttle), arr2, 0, 1);
                    break;
                }
                case LeftAil:
                    arr[0] = mInputMap->at(LeftAil);
                    XPLMSetDataf(mIDataRefs->at(LeftAil), arr[0]);
                    break;
                case RightAil:
                    arr[0] = mInputMap->at(RightAil);
                    XPLMSetDataf(mIDataRefs->at(RightAil), arr[0]);
                    break;
                case LeftElev:
                    arr[0] = mInputMap->at(LeftElev);
                    XPLMSetDataf(mIDataRefs->at(LeftElev), arr[0]);
                    break;
                case RightElev:
                    arr[0] = mInputMap->at(RightElev);
                    XPLMSetDataf(mIDataRefs->at(RightAil), arr[0]);
                    break;
                case Rudder:
                    arr[0] = mInputMap->at(Rudder);
                    XPLMSetDataf(mIDataRefs->at(Rudder), arr[0]);
                    break;
                default:
                    break;
            }

        }
    }

    //Update flags
    //FUCKING GRAVE, IM OUT
    XPLMSetDatai(mODataRefs->at(ControlOverride), mOutputMap->at(ControlOverride));
    XPLMSetDatai(mODataRefs->at(ControlSrfcOverride), mOutputMap->at(ControlSrfcOverride));
    //Update flight params
    for (int i = 0; i < ControlSrfcOverride - 3; i++) {
        mOutputMap->at(i) = XPLMGetDataf(mODataRefs->at((SimDataEnum)i));
    }

    IPC::unlock();
}

void Guidance::SwitchControls() {
    IPC::lock();
    mInputMap->at(LeftAil) = 0;
    mInputMap->at(RightAil) = 0;
    mInputMap->at(LeftElev) = 0;
    mInputMap->at(RightElev) = 0;
    if (mOutputMap->at(ControlOverride) == 1) {
        mOutputMap->at(ControlOverride) = 0;
        mOutputMap->at(ControlSrfcOverride) = 0;
    }
    else {
        mOutputMap->at(ControlOverride) = 1;
        mOutputMap->at(ControlSrfcOverride) = 1;
    }
    IPC::unlock();
}

Guidance::~Guidance() {

}

