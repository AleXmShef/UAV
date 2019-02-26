#include "Guidance.h"

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
    auto xpipeline = new PIDPipeline(4);
    auto ypipeline = new PIDPipeline(4);
    mPIDpipelines.insert({X, xpipeline});
    mPIDpipelines.insert({Y, ypipeline});

    //Find DataMaps
    IPCns::IPC::GetInstance();
    IPCns::IPC::findData(mDataMaps.at(ControlsData), SHRDINPUT_NAME);     //Inverted
    IPCns::IPC::findData(mDataMaps.at(SimData), SHRDOUTPUT_NAME);     //Inverted

    //Lock controls
    mDataMaps.at(SimData)->at(ControlOverride) = 1;
    mDataMaps.at(SimData)->at(ControlSrfcOverride) = 1;
}

void Guidance::Update() {
    //update errors
    double verticalVelocity = mDataMaps.at(SimData)->at(VerticalPath)*cos(mDataMaps.at(SimData)->at(Velocity));

    std::vector<double> XaxisErrors;
    std::vector<double> YaxisErrors;

    XaxisErrors.push_back(verticalVelocity);
    XaxisErrors.push_back(mDataMaps.at(SimData)->at(PitchAoA));
    XaxisErrors.push_back(mDataMaps.at(SimData)->at(PitchAngVel));
    XaxisErrors.push_back(mDataMaps.at(SimData)->at(PitchAngAcc));

    YaxisErrors.push_back(mDataMaps.at(SimData)->at(YawAngVel));
    YaxisErrors.push_back(mDataMaps.at(SimData)->at(YawAngAcc));
    YaxisErrors.push_back(mDataMaps.at(SimData)->at(RollAngVel));
    YaxisErrors.push_back(mDataMaps.at(SimData)->at(RollAngAcc));

    //calculate PIDs
    double Xcorr = mPIDpipelines.at(X)->Calculate(0, XaxisErrors);
    double Ycorr = mPIDpipelines.at(Y)->Calculate(0, YaxisErrors);
    //update flight controls

    mDataMaps.at(ControlsData)->at(LeftAil) = Ycorr;
    mDataMaps.at(ControlsData)->at(RightAil) = -Ycorr;
    mDataMaps.at(ControlsData)->at(RightElev) = Xcorr;
    mDataMaps.at(ControlsData)->at(LeftElev) = Xcorr;

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
