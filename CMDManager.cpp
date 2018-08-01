#include "CMDManager.h"

#define CHECK_VALUE_MACRO(VALUE_VARIABLE, VALUE_STR) if(string(VALUE_STR) == argv[i]) \
    {\
    if(++i == argc) \
        throw "no number specified after " VALUE_STR; \
    (VALUE_VARIABLE) = atof(argv[i]); \
    if((VALUE_VARIABLE) < 0) \
        throw "can't parse " VALUE_STR; \
    continue; \
    }

#define CHECK_EXIST_MACRO(VALUE_VARIABLE, VALUE_STR) if(string(VALUE_STR) == argv[i]) \
    {\
        (VALUE_VARIABLE) = true;\
        continue; \
    }

CMDManager::CMDManager(int argc, char **argv) {
    if (argc < 4) {
        char str[1024];
        sprintf(str, "[USAGE]: inputRobots inputObstacles [%s] [%s] [%s] [%s] [%s %s] [%s %s] [%s %s]\n"
                    "ENHANCE: [%s] [%s %s] [%s %s] [%s %s] [%s %s]",

                PATH_ENHANCE_STR,
                EXPORT_POINTS_STR,
                PRINT_STATISTICS_STR,
                PATH_CUT_STR,
                POINTS_PER_SQUARE_STR, NUMBER_STR,
                SEARCH_RADIUS_STR, NUMBER_STR,
                HEURISTIC_MULTIPLIER_STR, NUMBER_STR,
                ENHANCER_ROBOTS_POINT_ENFORCEMENT_STR,
                ENHANCER_WIDTH_STR, NUMBER_STR,
                ENHANCER_SEARCH_RADIUS_STR, NUMBER_STR,
                ENHANCER_HEURISTIC_MULTIPLIER_STR, NUMBER_STR,
                ENHANCER_POINTS_PER_SQUARE_STR, NUMBER_STR);

        cerr << str << endl;

        throw "not enough args";
    }
    robotFile = argv[1];
    obstecalesFile = argv[2];
    for(int i=3; i < argc; i++)
    {
        CHECK_EXIST_MACRO(enhancePath, PATH_ENHANCE_STR)
        CHECK_EXIST_MACRO(exportPoints, EXPORT_POINTS_STR)
        CHECK_EXIST_MACRO(printStatistics, PRINT_STATISTICS_STR)
        CHECK_EXIST_MACRO(cutPath, PATH_CUT_STR)
        CHECK_EXIST_MACRO(pointEnforcement, ENHANCER_ROBOTS_POINT_ENFORCEMENT_STR)

        CHECK_VALUE_MACRO(pointsPerSquare, POINTS_PER_SQUARE_STR)
        CHECK_VALUE_MACRO(searchRadius, SEARCH_RADIUS_STR)
        CHECK_VALUE_MACRO(heuristicMultiplier, HEURISTIC_MULTIPLIER_STR)
        CHECK_VALUE_MACRO(enhancerWidth, ENHANCER_WIDTH_STR)
        CHECK_VALUE_MACRO(enhancerSearchRadius, ENHANCER_SEARCH_RADIUS_STR)
        CHECK_VALUE_MACRO(enhancerHeuristicMultiplier, ENHANCER_HEURISTIC_MULTIPLIER_STR)
        CHECK_VALUE_MACRO(enhancerPointsPerSquare, ENHANCER_POINTS_PER_SQUARE_STR)
        else
            cout << "unknown param " << argv[i] << endl;
    }
}
