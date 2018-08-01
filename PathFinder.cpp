#include "PathFinder.h"

PathFinder::PathFinder(WorkingSpace& workingSpace, vector<Point_2>& start, vector<Point_2>& end, bool insertPoints)
        : workingSpace(workingSpace) {
    if(start.size() != end.size())
        throw "inconsistent number of robots";
    this->numberOfRobots = static_cast<int>(start.size());

    if(insertPoints) {
        workingSpace.insertPoints(start, START_POINT);
        workingSpace.insertPoints(end, END_POINT);
    }
    this->startCPoint = make_shared<ConfigurationPoint>(start);
    cSet.insert(this->startCPoint);
    startCPoint->visited = true;

    this->endCPoint = make_shared<ConfigurationPoint>(end);
    cSet.insert(this->endCPoint);
}

bool Edge::operator<(const Edge &rhs) const {
    if(this->distance != rhs.distance)
        return this->distance > rhs.distance;

    if(this->from != rhs.from)
        return this->from > rhs.from;

    return this->to > rhs.to;
}

bool CompareCPoint::operator()(const CPoint &a, const CPoint &b) const {
    return *a < *b;
}

void PathFinder::addEdge(CPoint current, int robotMovedIndex, Point_2& robotMovedNewPosition) {
    CPoint temp = make_shared<ConfigurationPoint>(ConfigurationPoint(current, robotMovedIndex, robotMovedNewPosition));
    auto it=cSet.find(temp);
    if(it != cSet.end())
    {
        temp = *it;
        if(temp->visited)
            return;
    } else
    {
        if(!temp->isConfigurationLegal())
            return;

        cSet.insert(temp);
        this->numberOfCpoints++;
    }
    this->numberOfEdges++;
    workingSpace.updatePointMap(temp->robots[robotMovedIndex], POINT_IN_CONFIGURATION);
    if(temp->heuristic < 0)
        temp->heuristic = temp->distanceToConfiguration(this->endCPoint) * heuristicMultiplier;
    double newDistance = current->distance + temp->distanceToConfiguration(current) + temp->heuristic;

    this->queue.push({current, temp, newDistance, robotMovedIndex});
}

void PathFinder::addNeighbors(CPoint current, bool robotPointsEnforcement) {
    if(robotPointsEnforcement)
    {
        for (int i = 0; i < numberOfRobots; i++) {
            vector<Point_2> neighbors = workingSpace.getNeighbors(current->robots[i], searchRadius, i);
            for (Point_2 &neighbor:neighbors) {
                if (neighbor != current->robots[i])
                    addEdge(current, i, neighbor);
            }
        }
    } else {
        for (int i = 0; i < numberOfRobots; i++) {
            vector<Point_2> neighbors = workingSpace.getNeighbors(current->robots[i], searchRadius);
            for (Point_2 &neighbor:neighbors) {
                if (neighbor != current->robots[i])
                    addEdge(current, i, neighbor);
            }
        }
    }
}

Path PathFinder::findPath(double searchRadius, double heuristicMultiplier, bool cutPath, bool robotPointsEnforcement)
{
    this->searchRadius = searchRadius;
    this->heuristicMultiplier = heuristicMultiplier;
    addNeighbors(startCPoint, robotPointsEnforcement);
    while (!queue.empty()) {
        if(queue.size() > maxQueueSize)
            maxQueueSize = queue.size();
        Edge currentEdge = queue.top();
        queue.pop();

        if (currentEdge.to->visited || !isEdgeLegal(currentEdge)) {
            this->deprecatedEdges++;
            continue;
        }
        this->processedEdges++;

        CPoint currentCpoint = currentEdge.to;


        currentCpoint->last = currentEdge.from;
        currentCpoint->robotChangedIndex = currentEdge.robotMovedIndex;
        currentCpoint->distance = currentEdge.distance - currentCpoint->heuristic;
        currentCpoint->visited = true;

        workingSpace.updatePointMap(currentCpoint->robots[currentCpoint->robotChangedIndex], POINT_PROCESSED);

        if(currentCpoint == endCPoint) {
            if(cutPath)
                this->cutPath();
            return Path(this->startCPoint, this->endCPoint);
        }
        addNeighbors(currentCpoint, robotPointsEnforcement);

    }
    return Path();
}

bool PathFinder::isEdgeLegal(Edge& edge) {
    Point_2& startPoint = edge.from->robots[edge.robotMovedIndex];
    Point_2& endPoint = edge.to->robots[edge.robotMovedIndex];
    Segment_2 qry(startPoint, endPoint);

    if(!this->workingSpace.segmentQuery(qry))
        return false;


    for(int i=0; i<this->numberOfRobots;i++)
    {
        if(edge.robotMovedIndex == i)
            continue;
        Point_2& stadyPoint = edge.to->robots[i];

        Point_2 p1(stadyPoint.x()-1, stadyPoint.y()-1);
        Point_2 p2(stadyPoint.x()-1, stadyPoint.y()+1);
        Point_2 p3(stadyPoint.x()+1, stadyPoint.y()+1);
        Point_2 p4(stadyPoint.x()+1, stadyPoint.y()-1);

        Segment_2 s1(p1,p2), s2(p2,p3), s3(p3,p4), s4(p4,p1);
        if(CGAL::do_intersect(s1, qry) || CGAL::do_intersect(s2, qry) ||
           CGAL::do_intersect(s3, qry) || CGAL::do_intersect(s4, qry))
            return false;
    }

    return true;
}

void PathFinder::printStatistics(bool print) {
    if(!print)
        return;

    cout << "PATH FINDER STATISTICS:\n";
    cout << "max queue size  " << maxQueueSize << endl;
    cout << "number of edges " << numberOfEdges << endl;
    cout << "number of configurations " << numberOfCpoints << endl;
    cout << "number of edges processed " << processedEdges << endl;
    cout << "number of deprecated edges  " << deprecatedEdges << endl;
    cout << "number of points cut out " << cuts << endl;
}

void PathFinder::cutPath() {
    CPoint CPointIt = this->endCPoint;
    while(CPointIt != this->startCPoint && CPointIt->last != this->startCPoint)
    {
        while(CPointIt->robotChangedIndex == CPointIt->last->robotChangedIndex)
        {
            Edge tempEdge({CPointIt->last->last, CPointIt, 0, CPointIt->robotChangedIndex});
            if(isEdgeLegal(tempEdge))
            {
                cuts++;
                workingSpace.updatePointMap(CPointIt->last->robots[CPointIt->robotChangedIndex], POINT_CUTED);
                CPointIt->last = CPointIt->last->last;
            } else
                break;
        }
        CPointIt = CPointIt->last;
    }
}
