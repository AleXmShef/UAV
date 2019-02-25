#include "Guidance.h"

using namespace UAV;

Guidance* Guidance::mInstance = nullptr;
std::map<Axes, PIDPipeline*> Guidance::mPIDpipelines;
std::map<DataMaps, IPCns::IPCSharedMap*> Guidance::mDataMaps;

Guidance* Guidance::GetInstance() {
    if(mInstance==nullptr) {
        mInstance = new Guidance();
        return mInstance;
    }
    return mInstance;
}

void Guidance::Init() {
    //Register PIDpipelines
    auto xpipeline = new PIDPipeline(4);
    auto ypipeline = new PIDPipeline(4);
    mPIDpipelines.insert({X, xpipeline});
    mPIDpipelines.insert({Y, ypipeline});

    //Find DataMaps
    IPCns::IPC::GetInstance();
    IPCns::IPC::findData(mDataMaps.at(ControlsData), SHRDINPUT_NAME);     //Inverted
    IPCns::IPC::findData(mDataMaps.at(SimData), SHRDOUTPUT_NAME);     //Inverted

    //Lock controls
}

void Guidance::Update() {

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
