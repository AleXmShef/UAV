//
// Created by Sasha on 25.03.2019.
//

#include "Route.h"
#include <math.h>

using namespace UAV;

Route* Route::mInstance = nullptr;

Route* Route::GetInstance() {
    if (mInstance == nullptr) {
        mInstance = new Route();
        mInstance->mName = "Route Info";
        Logger::GetInstance()->registerLoggable(mInstance);
    }
    return mInstance;
}

bool Route::isEmpty() {
    return (mWaypoints.empty());
}

void Route::addWaypoint(UAV::Waypoint waypoint) {
    if (mWaypoints.empty())
        mWaypoints.push_back(waypoint);
    else if(!(mWaypoints[(mWaypoints.size()-1)] == waypoint))
        mWaypoints.push_back(waypoint);
    calculateDistances();
}

Waypoint Route::getActiveWaypoint() {
    if (!(mWaypoints.empty()))
        return mWaypoints[0];
    else
        return Waypoint(0, 0);
}

void Route::swapWaypoint(UAV::Waypoint swapToWaypoint, int waypointIndex) {
    mWaypoints[waypointIndex] = swapToWaypoint;
    calculateDistances();
}

void Route::insertWaypoint(UAV::Waypoint waypoint, int afterIndex) {
    mWaypoints.insert(mWaypoints.begin() + afterIndex + 1, waypoint);
    calculateDistances();
}

void Route::removeWaypoint(int removeAtIndex) {
    if(mWaypoints.size() >= removeAtIndex + 1)
        mWaypoints.erase(mWaypoints.begin() + removeAtIndex);
    calculateDistances();
}

void Route::toNextWaypoint() {
    if(mWaypoints.size() >= 1)
        mWaypoints.erase(mWaypoints.begin());
    calculateDistances();
}

void Route::directTo(int toAtIndex) {
    if(mWaypoints.size() >= toAtIndex)
        mWaypoints.erase(mWaypoints.begin(), mWaypoints.begin() + toAtIndex);
}

double Route::calculateDistanceBetweenTwoWaypoints(UAV::Waypoint waypoint1, UAV::Waypoint waypoint2) {
    double phi1 = waypoint1.getLocation()[0]*M_PI/180;
    double phi2 = waypoint2.getLocation()[0]*M_PI/180;
    double dLambda = (waypoint2.getLocation()[1] - waypoint1.getLocation()[1])*M_PI/180;
    double dPhi = phi2 - phi1;
    double a = sin(dPhi/2)*sin(dPhi/2) + cos(phi1)*cos(phi2)*sin(dLambda/2)*sin(dLambda/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return c * 6371000;
}

void Route::calculateDistances() {
    totalDistance = 0;
    if (mWaypoints.size() >= 2) {
        for (int i = 0; i < mWaypoints.size() - 1; i++) {
            if(pairedDistance.empty() || pairedDistance.size() == i + 1)
                pairedDistance.push_back(0);
            pairedDistance[i] = calculateDistanceBetweenTwoWaypoints(mWaypoints[i], mWaypoints[i+1]);
            totalDistance += pairedDistance[i];
        }
    }
}

std::map<std::string, double>* Route::getLogInfo() {
    auto tmap = new std::map<std::string, double>;
    tmap->insert({"Total distance", totalDistance});
    tmap->insert({"Total waypoints", mWaypoints.size()});
    return tmap;
}

