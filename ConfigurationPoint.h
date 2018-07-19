#ifndef MRPE_CONFIGURATIONPOINT_H
#define MRPE_CONFIGURATIONPOINT_H

#include <vector>
#include "CGAL_defines.h"
using namespace std;


class ConfigurationPoint
{
    public:
        ConfigurationPoint(vector<Point_2> robots);
        ConfigurationPoint(ConfigurationPoint* old, int robotChangedIndex, Point_2 robotChangedNewPosition);

        bool operator<(const ConfigurationPoint &rhs) const;
        bool operator!=(const ConfigurationPoint &rhs) const;

        double distanceToConfiguration(ConfigurationPoint *dest);
        bool isConfigurationLegal();

        int numberOfRobots;
        vector<Point_2> robots;
        ConfigurationPoint* last = nullptr;
        bool visited = false;
        double heuristic=-1;
        double distance = 0;
};

#endif //MRPE_CONFIGURATIONPOINT_H
