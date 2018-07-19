#ifndef MRPE_PATHFINDER_H
#define MRPE_PATHFINDER_H

#include <vector>
#include <list>
#include <math.h>
#include <set>
#include <queue>

#include "CGAL_defines.h"
#include "ConfigurationPoint.h"
#include "WorkingSpace.h"

#define RADIUS 1

using namespace std;

class Edge{
public:
    ConfigurationPoint *from, *to;
    double distance;
    int robotMovedIndex = -1;

    bool operator<(const Edge &rhs) const;
};

class PathFinder
{
private:
    int numberOfRobots;
    WorkingSpace& workingSpace;

    ConfigurationPoint *startCPoint, *endCPoint;
    set<ConfigurationPoint> cMap;
    priority_queue<Edge> queue;

    void addEdge(ConfigurationPoint *current, int robotMovedIndex, Point_2 robotMovedNewPosition);
    void addNeighbors(ConfigurationPoint *current);

    bool isEdgeLegal(Edge edge);

public:
    PathFinder(WorkingSpace& workingSpace);
    bool findPath(vector<Point_2>& start, vector<Point_2>& end);
    list<ConfigurationPoint> fetchPath();
};

#endif //MRPE_PATHFINDER_H
