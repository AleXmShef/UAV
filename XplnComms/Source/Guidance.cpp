#include "Guidance.h"

using namespace UAV;

Guidance* Guidance::mInstance = nullptr;

Guidance::Guidance() {
    //find datarefs
    mODataRefsTypes = {"Latitude", "Longitude", "Pitch", "Roll", "Heading"};
    mIDataRefsTypes = {"YokePitch", "YokeRoll", "YokeYaw", "Throttle"};

    //IMPORTANT: change these to individual cntrl srfces
//    mDataRefs->insert({XPLMFindDataRef("sim/flightmodel/controls/lail1def"), "left_aileron"});     //left aileron
//    mDataRefs->insert({XPLMFindDataRef("sim/flightmodel/controls/rail1def"), "right_aileron"});     //right aileron
//    mDataRefs->insert({XPLMFindDataRef("sim/flightmodel/engine/ENGN_thro"), "engine_throttle"});       //engine throttle

    mDataRefs->insert({"YokePitch", XPLMFindDataRef("sim/cockpit2/controls/yoke_pitch_ratio")});          //yoke pitch
    mDataRefs->insert({"YokeRoll", XPLMFindDataRef("sim/cockpit2/controls/yoke_roll_ratio")});          //yoke roll
    mDataRefs->insert({"YokeYaw", XPLMFindDataRef("sim/cockpit2/controls/yoke_yaw_ratio")});          //yoke yaw
    mDataRefs->insert({"Throttle", XPLMFindDataRef("sim/cockpit2/engine/actuators/throttle_ratio_all")});          //engine throttle
    mDataRefs->insert({"Latitude", XPLMFindDataRef("sim/flightmodel/position/latitude")});          //latitude
    mDataRefs->insert({"Longitude", XPLMFindDataRef("sim/flightmodel/position/longitude")});        //longitude
    mDataRefs->insert({"Pitch", XPLMFindDataRef("sim/flightmodel/position/true_theta")});           //pitch
    mDataRefs->insert({"Roll", XPLMFindDataRef("sim/flightmodel/position/true_phi")});              //roll
    mDataRefs->insert({"Heading", XPLMFindDataRef("sim/flightmodel/position/mag_psi")});            //heading
    mDataRefs->insert({"fltcntrl_override_flag", XPLMFindDataRef("sim/operation/override/override_flight_control")});       //flag to override flight controls

    //Initialize data maps
    IPC::GetInstance();
    mOutputMap = IPC::registerData(mOutputMap, SHRDOUTPUT_NAME);
   // mInputMap = IPC::registerData(mInputMap, SHRDINPUT_NAME);

    for(int i = 0; i < mODataRefsTypes.size(); i++) {
        mOutputMap->insert(std::pair<const int, float>(i, 0));
    }

//    for(int i = 0; i < mIDataRefsTypes.size(); i++) {
//        mInputMap->insert(std::pair<const std::string, float>(mIDataRefsTypes[i], 0));
//    }


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
    for(int i = 0; i < mIDataRefsTypes.size(); i++) {
        XPLMSetDataf(mDataRefs->at(mIDataRefsTypes[i]), mInputMap->at(i));
    }
}

Guidance::~Guidance() {
    //destroy pids
}

