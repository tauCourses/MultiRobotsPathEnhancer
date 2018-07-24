#ifndef MRPE_CMDMANAGER_H
#define MRPE_CMDMANAGER_H

#include <iostream>
#include <cstdlib>

#define PATH_ENHANCE_STR "-enhance"
#define EXPORT_POINTS_STR "-export"
#define PRINT_STATISTICS_STR "-stats"
#define POINTS_PER_SQUARE_STR "-pps"
#define SEARCH_RADIUS_STR "-radius"
#define HEURISTIC_RATIO_STR "-er"
#define NUMBER_STR "number"

#define DEFAULT_NUM_OF_POINTS_PER_SQUARE 2
#define DEFAULT_SEARCH_RADIUS 3
#define DEFAULT_HEURISTIC_RATIO 2


using namespace std;

class CMDManager {
public:
    CMDManager(int argc, char *argv[]);
    bool exportPoints=false, enhancePath=false, printStatistics=false;
    double pointsPerSquare = DEFAULT_NUM_OF_POINTS_PER_SQUARE;
    double searchRadius = DEFAULT_SEARCH_RADIUS;
    double heuristicRatio = DEFAULT_HEURISTIC_RATIO;
    char *robotFile, *obstecalesFile, *outputFile;
};


#endif //MRPE_CMDMANAGER_H
