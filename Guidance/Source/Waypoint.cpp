//
// Created by Sasha on 25.03.2019.
//

#include "Waypoint.h"

using namespace UAV;

double* Waypoint::getLocation() {
    auto arr = new double(2);
    arr[0] = _latitude;
    arr[1] = _longitude;

    return arr;
}

const bool Waypoint::operator==(const UAV::Waypoint &waypoint) {
    if (this->_latitude == waypoint._latitude && this->_longitude == waypoint._longitude)
        return true;
    else
        return false;
}

std::map<std::string, double>* Waypoint::getLogInfo() {

}
