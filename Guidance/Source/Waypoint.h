#ifndef UAV_WAYPOINT_H
#define UAV_WAYPOINT_H

#include "Loggable.h"

namespace UAV {
    class Waypoint: public Loggable {
    public:
        Waypoint(double lat, double lng):_longitude(lng), _latitude(lat){};
        double* getLocation();
        const bool operator==(const Waypoint& waypoint);
        std::map<std::string, double>* getLogInfo() override;
    public:
        unsigned int _type = 0;         //change to some enum later
    protected:
        double _longitude;
        double _latitude;
    };

}

#endif //UAV_WAYPOINT_H
