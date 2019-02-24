#include "controls.h"

using namespace UAV;

Guidance* Guidance::mInstance = nullptr;

Guidance::Guidance() {
    //find datarefs

    //IMPORTANT: change these to individual cntrl srfces
//    mDataRefs->insert({XPLMFindDataRef("sim/flightmodel/controls/lail1def"), "left_aileron"});     //left aileron
//    mDataRefs->insert({XPLMFindDataRef("sim/flightmodel/controls/rail1def"), "right_aileron"});     //right aileron
//    mDataRefs->insert({XPLMFindDataRef("sim/flightmodel/engine/ENGN_thro"), "engine_throttle"});       //engine throttle

    mDataRefs->insert({XPLMFindDataRef("sim/cockpit2/controls/yoke_pitch_ratio"), "yoke_pitch"});          //yoke pitch
    mDataRefs->insert({XPLMFindDataRef("sim/cockpit2/controls/yoke_roll_ratio"), "yoke_roll"});          //yoke roll
    mDataRefs->insert({XPLMFindDataRef("sim/cockpit2/controls/yoke_yaw_ratio"), "yoke_yaw"});          //yoke yaw
    mDataRefs->insert({XPLMFindDataRef("sim/cockpit2/engine/actuators/throttle_ratio_all"), "throttle"});          //engine throttle
    mDataRefs->insert({XPLMFindDataRef("sim/flightmodel/position/latitude"), "latitude"});          //latitude
    mDataRefs->insert({XPLMFindDataRef("sim/flightmodel/position/latitude"), "latitude"});          //latitude
    mDataRefs->insert({XPLMFindDataRef("sim/flightmodel/position/longitude"), "longitude"});        //longitude
    mDataRefs->insert({XPLMFindDataRef("sim/flightmodel/position/true_theta"), "pitch"});           //pitch
    mDataRefs->insert({XPLMFindDataRef("sim/flightmodel/position/true_phi"), "roll"});              //roll
    mDataRefs->insert({XPLMFindDataRef("sim/flightmodel/position/mag_psi"), "heading"});            //heading
    mDataRefs->insert({XPLMFindDataRef("sim/operation/override/override_flight_control"), "fltcntrl_override_flag"});       //flag to override flight controls



    //setup pids
    //couple pids
}

Guidance* Guidance::GetInstance() {return mInstance;}

void Guidance::Update() {
    //get current data
    //call pids
    //update data
}

Guidance::~Guidance() {
    //destroy pids
}

