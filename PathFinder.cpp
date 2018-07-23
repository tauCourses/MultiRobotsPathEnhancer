#include "PathFinder.h"

PathFinder::PathFinder(WorkingSpace& workingSpace, double searchRadius)
        : workingSpace(workingSpace), searchRadius(searchRadius) {}

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
        temp->heuristic = temp->distanceToConfiguration(this->endCPoint);
    double newDistance = current->distance + temp->distanceToConfiguration(current) + temp->heuristic;

    this->queue.push({current, temp, newDistance, robotMovedIndex});
}

void PathFinder::addNeighbors(CPoint current) {
    for(int i=0; i<numberOfRobots; i++)
    {
        vector<Point_2> neighbors = workingSpace.getNeighbors(current->robots[i], searchRadius);
        for(Point_2& neighbor:neighbors) {
            if (neighbor != current->robots[i])
                addEdge(current, i, neighbor);
        }
    }
}

Path PathFinder::findPath(vector<Point_2>& start, vector<Point_2>& end)
{
    if(start.size() != end.size())
        throw "inconsistent number of robots";
    this->numberOfRobots = static_cast<int>(start.size());

    workingSpace.insertPoints(start, START_POINT);
    workingSpace.insertPoints(end, END_POINT);

    this->startCPoint = make_shared<ConfigurationPoint>(start);
    cSet.insert(this->startCPoint);
    startCPoint->visited = true;

    this->endCPoint = make_shared<ConfigurationPoint>(end);
    cSet.insert(this->endCPoint);

    addNeighbors(startCPoint);

    while (!queue.empty()) {
        if(queue.size() > maxQueueSize)
            maxQueueSize = queue.size();
        Edge currentEdge = queue.top();
        queue.pop();

        if (!isEdgeLegal(currentEdge)) {
            this->deprecatedEdges++;
            continue;
        }
        this->processedEdges++;
        CPoint currentCpoint = currentEdge.to;
        if(currentCpoint->robotChangedIndex >=0)
            workingSpace.updatePointMap(currentCpoint->robots[currentCpoint->robotChangedIndex], POINT_PROCESSED);
        currentCpoint->last = currentEdge.from;
        currentCpoint->distance = currentEdge.distance - currentCpoint->heuristic;
        currentCpoint->visited = true;

        if(currentCpoint == endCPoint)
            return Path(this->startCPoint, this->endCPoint, workingSpace);

        addNeighbors(currentCpoint);

    }
    return Path();
}

bool PathFinder::isEdgeLegal(Edge& edge) {
    if(edge.to->visited)
        return false;

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
    cout << "number of configuration points " << numberOfCpoints << endl;
    cout << "number of edges processed " << processedEdges << endl;
    cout << "number of deprecated edges  " << deprecatedEdges << endl;
}
