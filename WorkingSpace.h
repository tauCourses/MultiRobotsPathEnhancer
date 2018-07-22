#ifndef MRPE_WORKINGSPACE_H
#define MRPE_WORKINGSPACE_H

#include <vector>
#include <random>

#include "CGAL_defines.h"
#define NUM_OF_POINTS_PER_SQUARE 2

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
    int numberOfPoints = 0;

    void verticalDecomposition(Kernel &ker);
    void addVerticalSegment(Vertex_handle v, CGAL::Object obj, Kernel &ker);
    void createArrangment();
    void setRandomPoints();
    void setFaceRandomPoints(Face_handle face);

    void printArr();


public:
    WorkingSpace(Polygon_2& outer_poly, vector<Polygon_2>& obstacles);

    void insertPoints(vector<Point_2> points);
    bool inLegalFace(const Point_2 &p);
    bool segmentQuery(Segment_2 seg);
    vector<Point_2> getNeighbors(Point_2 p, FT radius);

};


#endif //MRPE_WORKINGSPACE_H
