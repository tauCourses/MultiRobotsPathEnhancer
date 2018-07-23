#ifndef MRPE_WORKINGSPACE_H
#define MRPE_WORKINGSPACE_H

#include <vector>
#include <random>
#include <map>

#include "CGAL_defines.h"

enum POINT_STATES {POINT_CREATED, POINT_IN_CONFIGURATION, POINT_PROCESSED ,POINT_IN_PATH, START_POINT, END_POINT};

using namespace std;

class polygon_split_observer : public CGAL::Arr_observer<Arrangement_2> {
    void after_split_face(Face_handle f1, Face_handle f2, bool) override;
};

class WorkingSpace {
private:

    Polygon_2& outer_poly;
    vector<Polygon_2>& obstacles;
    Arrangement_2 arr;
    Landmarks_pl pl;
    Tree tree;
    map<Point_2, POINT_STATES> pointsStateMap;
    double pointsPerSquare;
    long numberOfFaces=0, numberOfConatinedFaces=0, numberOfPointsChecked=0, numberOfLegalPoints=0;
    bool exportPointsEnabled;

    void verticalDecomposition(Kernel &ker);
    void addVerticalSegment(Vertex_handle v, CGAL::Object obj, Kernel &ker);
    void createArrangment();
    void setRandomPoints();
    void setFaceRandomPoints(Face_handle face);

    void printArr();


public:
    WorkingSpace(Polygon_2& outer_poly, vector<Polygon_2>& obstacles, bool exportPointsEnabled, double pointsPerSquare);

    void insertPoints(vector<Point_2> points, POINT_STATES state);
    bool inLegalFace(const Point_2 &p);
    bool segmentQuery(Segment_2 seg);
    void updatePointMap(Point_2& p, POINT_STATES state);
    void exportPoints();
    vector<Point_2> getNeighbors(Point_2& p, double radius);

    void printStatistics(bool print);
};


#endif //MRPE_WORKINGSPACE_H
