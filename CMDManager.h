#ifndef MRPE_CMDMANAGER_H
#define MRPE_CMDMANAGER_H

#include <iostream>
#include <cstdlib>

#define PATH_CUT_STR "-cut"
#define PATH_ENHANCE_STR "-enhance"
#define EXPORT_POINTS_STR "-export"
#define PRINT_STATISTICS_STR "-stats"
#define POINTS_PER_SQUARE_STR "-pps"
#define SEARCH_RADIUS_STR "-sr"
#define HEURISTIC_MULTIPLIER_STR "-hm"

#define ENHANCER_ROBOTS_POINT_ENFORCEMENT_STR "-erpe"
#define ENHANCER_WIDTH_STR "-ew"
#define ENHANCER_SEARCH_RADIUS_STR "-esr"
#define ENHANCER_HEURISTIC_MULTIPLIER_STR "-ehm"
#define ENHANCER_POINTS_PER_SQUARE_STR "-epps"

#define NUMBER_STR "number"

#define DEFAULT_NUM_OF_POINTS_PER_SQUARE 2
#define DEFAULT_SEARCH_RADIUS 3
#define DEFAULT_HEURISTIC_MULTIPLIER 2

#define DEFAULT_ENHANCER_WIDTH 2
#define DEFAULT_ENHANCER_SEARCH_RADIUS 3
#define DEFAULT_ENHANCER_HEURISTIC_MULTIPLIER 1
#define DEFAULT_ENHANCER_NUM_OF_POINTS_PER_SQUARE 2



using namespace std;

class CMDManager {
public:
    CMDManager(int argc, char *argv[]);
    bool exportPoints=false, enhancePath=false, printStatistics=false, cutPath=false, pointEnforcement=false;
    double pointsPerSquare = DEFAULT_NUM_OF_POINTS_PER_SQUARE;
    double searchRadius = DEFAULT_SEARCH_RADIUS;
    double heuristicMultiplier = DEFAULT_HEURISTIC_MULTIPLIER;

    double enhancerPointsPerSquare = DEFAULT_ENHANCER_NUM_OF_POINTS_PER_SQUARE;
    double enhancerWidth = DEFAULT_ENHANCER_WIDTH;
    double enhancerSearchRadius = DEFAULT_ENHANCER_SEARCH_RADIUS;
    double enhancerHeuristicMultiplier = DEFAULT_ENHANCER_HEURISTIC_MULTIPLIER;
    char *robotFile, *obstecalesFile;
};


#endif //MRPE_CMDMANAGER_H
