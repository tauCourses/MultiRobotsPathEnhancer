#ifndef MRPE_PATH_H
#define MRPE_PATH_H

#include "ConfigurationPoint.h"

class Path {
public:
    Path(CPoint start, CPoint end);
    Path();

    CPoint start, end;
    list<CPoint> cPoints;
    double length = 0;
    int numberOfPoints = 0;
    bool legal=true;

    void printStatistics(bool print);
    void exportPath(string file);
};

#endif //MRPE_PATH_H
