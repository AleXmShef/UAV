#ifndef UAV_ROUTE_H
#define UAV_ROUTE_H

#include "Waypoint.h"
#include <vector>

namespace UAV {
    class Route: public Loggable {
    public:
        static Route* GetInstance();

        void addWaypoint(Waypoint waypoint);
        Waypoint getActiveWaypoint();
        void swapWaypoint(Waypoint swapToWaypoint, int waypointIndex);
        void insertWaypoint(Waypoint waypoint, int afterIndex);
        void removeWaypoint(int removeAtIndex);
        void toNextWaypoint();
        void directTo(int toAtIndex);
        double calculateDistanceBetweenTwoWaypoints(Waypoint waypoint1, Waypoint waypoint2);
        bool isEmpty();

    public:
        std::map<std::string, double>* getLogInfo() override;

    protected:
        void calculateDistances();

    protected:
        std::vector<Waypoint> mWaypoints;
        double totalDistance = 0;
        std::vector<double> pairedDistance;

        static Route* mInstance;
    };
}

#endif //UAV_ROUTE_H
