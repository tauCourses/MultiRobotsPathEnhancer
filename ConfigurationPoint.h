#ifndef MRPE_CONFIGURATIONPOINT_H
#define MRPE_CONFIGURATIONPOINT_H

#include <vector>
#include "CGAL_defines.h"

#define CPoint shared_ptr<ConfigurationPoint>

using namespace std;


class ConfigurationPoint
{
private:
    static long objectCounter;
public:
    ConfigurationPoint(vector<Point_2>& robots);
    ConfigurationPoint(CPoint old, int robotChangedIndex, Point_2& robotChangedNewPosition);

    bool operator<(const ConfigurationPoint &rhs) const;
    bool operator!=(const ConfigurationPoint &rhs) const;
    double distanceToConfiguration(CPoint dest);
    bool isConfigurationLegal();

    int numberOfRobots;
    vector<Point_2> robots;
    CPoint last;
    bool visited = false;
    long index;
    double heuristic=-1;
    double distance = 0;
};

ostream &operator<<(std::ostream &os, const ConfigurationPoint &cp);


#endif //MRPE_CONFIGURATIONPOINT_H
