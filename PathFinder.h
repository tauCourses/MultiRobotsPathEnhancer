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

#define RADIUS 3

using namespace std;

class Edge{
public:
    CPoint from, to;
    double distance;
    int robotMovedIndex = -1;

    bool operator<(const Edge &rhs) const;
};

struct CompareCPoint
{
    bool operator()(const CPoint& a, const CPoint& b) const;
};

class PathFinder
{
private:
    int numberOfRobots;
    WorkingSpace& workingSpace;

    CPoint startCPoint, endCPoint;
    set<CPoint, CompareCPoint> cSet;
    priority_queue<Edge> queue;

    void addEdge(CPoint current, int robotMovedIndex, Point_2& robotMovedNewPosition);
    void addNeighbors(CPoint current);

    bool isEdgeLegal(Edge& edge);

public:
    PathFinder(WorkingSpace& workingSpace);
    bool findPath(vector<Point_2>& start, vector<Point_2>& end);
    list<ConfigurationPoint> fetchPath();
};


#endif //MRPE_PATHFINDER_H
