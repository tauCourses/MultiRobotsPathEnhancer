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
#include "Path.h"

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
    long numberOfEdges=0, numberOfCpoints=0, processedEdges=0, deprecatedEdges=0, maxQueueSize=0;
    double searchRadius;
    WorkingSpace& workingSpace;

    CPoint startCPoint, endCPoint;
    set<CPoint, CompareCPoint> cSet;
    priority_queue<Edge> queue;

    void addEdge(CPoint current, int robotMovedIndex, Point_2& robotMovedNewPosition);
    void addNeighbors(CPoint current);

    bool isEdgeLegal(Edge& edge);

public:
    PathFinder(WorkingSpace& workingSpace, double searchRadius);
    Path findPath(vector<Point_2>& start, vector<Point_2>& end);
    void printStatistics(bool print);
};


#endif //MRPE_PATHFINDER_H
