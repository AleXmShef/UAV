#include "Guidance.h"

using namespace UAV;

Guidance* Guidance::mInstance = nullptr;

Guidance::Guidance() {
    //find datarefs
    mODataRefsTypes = {"Latitude", "Longitude", "Pitch", "Roll", "Heading"};
    mIDataRefsTypes = {"YokePitch", "YokeRoll", "YokeYaw", "Throttle"};
    mDataRefs = new std::map<std::string, XPLMDataRef>;

    //IMPORTANT: change these to individual cntrl srfces
    mDataRefs->insert({"YokePitch", XPLMFindDataRef("sim/flightmodel/controls/elv1_def")});     //left aileron
    mDataRefs->insert({"YokeRoll", XPLMFindDataRef("sim/flightmodel/controls/ail1_def")});     //right aileron
    mDataRefs->insert({"YokeYaw", XPLMFindDataRef("sim/flightmodel/controls/rudd_def")});       //engine throttle


//    mDataRefs->insert({"YokePitch", XPLMFindDataRef("sim/cockpit2/controls/yoke_pitch_ratio")});          //yoke pitch
//    mDataRefs->insert({"YokeRoll", XPLMFindDataRef("sim/cockpit2/controls/yoke_roll_ratio")});          //yoke roll
//    mDataRefs->insert({"YokeYaw", XPLMFindDataRef("sim/cockpit2/controls/yoke_yaw_ratio")});          //yoke yaw
//    mDataRefs->insert({"YokePitch", XPLMFindDataRef("sim/flightmodel/controls/elv1_def")});          //yoke pitch
//    mDataRefs->insert({"YokeRoll", XPLMFindDataRef("sim/flightmodel/controls/ail1_def")});          //yoke roll
//    mDataRefs->insert({"YokeYaw", XPLMFindDataRef("sim/joystick/yoke_yaw_ratio")});          //yoke yaw
    mDataRefs->insert({"Throttle", XPLMFindDataRef("sim/flightmodel/engine/ENGN_thro_use")});          //engine throttle
    mDataRefs->insert({"Latitude", XPLMFindDataRef("sim/flightmodel/position/latitude")});          //latitude
    mDataRefs->insert({"Longitude", XPLMFindDataRef("sim/flightmodel/position/longitude")});        //longitude
    mDataRefs->insert({"Pitch", XPLMFindDataRef("sim/flightmodel/position/true_theta")});           //pitch
    mDataRefs->insert({"Roll", XPLMFindDataRef("sim/flightmodel/position/true_phi")});              //roll
    mDataRefs->insert({"Heading", XPLMFindDataRef("sim/flightmodel/position/mag_psi")});            //heading
    mDataRefs->insert({"fltcntrl_override_flag", XPLMFindDataRef("sim/operation/override/override_flightcontrol")});       //flag to override flight controls
    mDataRefs->insert({"fltcntrloverride_flag", XPLMFindDataRef("sim/operation/override/override_control_surfaces")});       //flag to override flight controls

    XPLMSetDatai(mDataRefs->at("fltcntrl_override_flag"), 1);
    XPLMSetDatai(mDataRefs->at("fltcntrloverride_flag"), 1);

    //Initialize data maps
    IPC::GetInstance();
    mOutputMap = IPC::registerData(mOutputMap, SHRDOUTPUT_NAME);
    mInputMap = IPC::registerData(mInputMap, SHRDINPUT_NAME);

    for(int i = 0; i < mODataRefsTypes.size(); i++) {
        mOutputMap->insert(std::pair<const int, float>(i, 0));
    }

    for(int i = 0; i < mIDataRefsTypes.size(); i++) {
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
    //Update flight params
    for(int i = 0; i < mODataRefsTypes.size(); i++) {
        mOutputMap->at(i) = XPLMGetDataf(mDataRefs->at(mODataRefsTypes[i]));
    }
    //Update flight controls
    float* arr = new float[56];
    for(int j = 0; j < 3; j++) {
        for (int i = 0; i < 56; i++) {
            arr[i] = mInputMap->at(j);
        }
        XPLMSetDatavf(mDataRefs->at(mIDataRefsTypes[j]), arr, 0, 55);
    }
}

Guidance::~Guidance() {
    //destroy pids
}

