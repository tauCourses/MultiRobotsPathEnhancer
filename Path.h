#ifndef MRPE_PATH_H
#define MRPE_PATH_H

#include "ConfigurationPoint.h"
#include "WorkingSpace.h"

class Path {
public:
    Path(CPoint start, CPoint end, WorkingSpace& ws);
    Path();

    CPoint start, end;
    list<CPoint> cPoints;
    double length = 0;
    int numberOfPoints = 0;
    bool legal=true;

    void printStatistics(bool print);
    void exportPath(char* file);
};

#endif //MRPE_PATH_H
