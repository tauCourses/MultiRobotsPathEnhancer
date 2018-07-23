#include "CMDManager.h"

CMDManager::CMDManager(int argc, char **argv) {
    if (argc < 4) {
        char str[1024];
        sprintf(str, "[USAGE]: inputRobots inputObstacles outputFile [%s] [%s] [%s] [%s %s] [%s %s]",
                PATH_ENHANCE_STR,
                EXPORT_POINTS_STR,
                PRINT_STATISTICS_STR,
                POINTS_PER_SQUARE_STR, NUMBER_STR,
                SEARCH_RADIUS_STR, NUMBER_STR);
        cerr << str << endl;

        throw "not enough args";
    }
    robotFile = argv[1];
    obstecalesFile = argv[2];
    outputFile = argv[3];
    for(int i=4; i < argc; i++)
    {
        if(string(PATH_ENHANCE_STR) == argv[i])
            this->enhancePath = true;
        else if(string(EXPORT_POINTS_STR) == argv[i])
            this->exportPoints = true;
        else if(string(PRINT_STATISTICS_STR) == argv[i])
            this->printStatistics = true;
        else if(string(POINTS_PER_SQUARE_STR) == argv[i])
        {
            if(++i == argc)
                throw "no number specified after POINT_PER_SQUARE";
            pointsPerSquare = atof(argv[i]);
        }
        else if(string(SEARCH_RADIUS_STR) == argv[i])
        {
            if(++i == argc)
                throw "no number specified after POINT_PER_SQUARE";
            searchRadius = atof(argv[i]);
        }
    }
}
