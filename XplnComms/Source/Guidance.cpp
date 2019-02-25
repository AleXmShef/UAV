#include "Guidance.h"

using namespace UAV;
using namespace IPCns;

Guidance* Guidance::mInstance = nullptr;

Guidance::Guidance() {
    //find datarefs
    mODataRefsTypes = {"Latitude", "Longitude", "Pitch", "Roll", "Heading"};
    mIDataRefsTypes = {"Throttle", "LeftAileron", "RightAileron", "LeftElevator", "RightElevator", "Rudder"};
    mDataRefs = new std::map<std::string, XPLMDataRef>;

    //IMPORTANT: change these to individual cntrl srfces
    mDataRefs->insert({"Pitch", XPLMFindDataRef("sim/flightmodel2/wing/elevator1_deg")});     //elevator
    mDataRefs->insert({"Roll", XPLMFindDataRef("sim/flightmodel2/wing/aileron1_deg")});     //ailerons
    mDataRefs->insert({"Yaw", XPLMFindDataRef("sim/flightmodel2/wing/rudder1_deg")});       //rudder
    mDataRefs->insert({"Throttle", XPLMFindDataRef("sim/flightmodel/engine/ENGN_thro_use")});          //engine throttle

    mDataRefs->insert({"Latitude", XPLMFindDataRef("sim/flightmodel/position/latitude")});          //latitude
    mDataRefs->insert({"Longitude", XPLMFindDataRef("sim/flightmodel/position/longitude")});        //longitude
    mDataRefs->insert({"Pitch", XPLMFindDataRef("sim/flightmodel/position/true_theta")});           //pitch
    mDataRefs->insert({"Roll", XPLMFindDataRef("sim/flightmodel/position/true_phi")});              //roll
    mDataRefs->insert({"Heading", XPLMFindDataRef("sim/flightmodel/position/mag_psi")});            //heading
    mDataRefs->insert({"fltcntrl_override_flag", XPLMFindDataRef("sim/operation/override/override_flightcontrol")});       //flag to override flight controls
    mDataRefs->insert({"fltcntrloverride_flag", XPLMFindDataRef("sim/operation/override/override_control_surfaces")});       //flag to override control surfaces

    XPLMSetDatai(mDataRefs->at("fltcntrl_override_flag"), 1);
    XPLMSetDatai(mDataRefs->at("fltcntrloverride_flag"), 1);

    //Initialize data maps
    shared_memory_object::remove(SHRDMEM_NAME);
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
    //See mIDatarefsTypes for switch reference
    float* arr = new float[23];
    for(int j = 0; j < mIDataRefsTypes.size(); j++) {
        switch(j) {
            case 0:
                XPLMSetDataf(mDataRefs->at("Throttle"), mInputMap->at(j));
                break;
            case 1:
                for (int i = 0; i < 23; i++) {
                    arr[i] = mInputMap->at(j);
                }
                XPLMSetDatavf(mDataRefs->at("Roll"), arr, 0, 1);
                break;
            case 2:
                for (int i = 0; i < 23; i++) {
                    arr[i] = mInputMap->at(j);
                }
                XPLMSetDatavf(mDataRefs->at("Roll"), arr, 1, 2);
                break;
            case 3:
                for (int i = 0; i < 23; i++) {
                    arr[i] = mInputMap->at(j);
                }
                XPLMSetDatavf(mDataRefs->at("Pitch"), arr, 0, 1);
                break;
            case 4:
                for (int i = 0; i < 23; i++) {
                    arr[i] = mInputMap->at(j);
                }
                XPLMSetDatavf(mDataRefs->at("Pitch"), arr, 1, 2);
                break;
            case 5:
                for (int i = 0; i < 23; i++) {
                    arr[i] = mInputMap->at(j);
                }
                XPLMSetDatavf(mDataRefs->at("Yaw"), arr, 0, 1);
                XPLMSetDatavf(mDataRefs->at("Yaw"), arr, 1, 2);
                break;
            default:
                break;
        }

    }
}

Guidance::~Guidance() {
    //destroy pids
}

