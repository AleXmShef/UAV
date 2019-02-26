#ifndef UAV_ENUMS_H
#define UAV_ENUMS_H

enum SimDataEnum{Latitude,
        Longitude,
        Pitch,
        Roll,
        Heading,
        Altitude,
        Velocity,
        PitchAoA,
        YawAoA,
        VerticalPath,
        HorizontalPath,
        PitchAngVel,
        RollAngVel,
        YawAngVel,
        PitchAngAcc,
        RollAngAcc,
        YawAngAcc,
        //These must be at the end
        ControlOverride,
        ControlSrfcOverride
};
enum ControlsDataEnum{
        Throttle,
        LeftAil,
        RightAil,
        LeftElev,
        RightElev,

        //This guy must be at the end
        Rudder
};


#endif //UAV_ENUMS_H
