#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <boost/timer.hpp>
#include <stdio.h>

#include "CGAL_defines.h"
#include "PathFinder.h"
#include "CMDManager.h"

using namespace std;
#define PATH_POINTS_FILE "points"
#define ENHANCER_PATH_POINTS_FILE "enhanced_points"

#define PATH_OUTPUT_FILE "output"
#define ENHANCER_PATH_OUTPUT_FILE "enhanced_output"

Point_2 loadPoint_2(std::ifstream &is) {
    Kernel::FT x, y;
    is >> x >> y;
    Point_2 point(x, y);
    return point;
}

Polygon_2 loadPolygon(ifstream &is) {
    size_t polygon_size = 0;
    is >> polygon_size;
    Polygon_2 ret;
    while (polygon_size--)
        ret.push_back(loadPoint_2(is));
    CGAL::Orientation orient = ret.orientation();
    if (CGAL::COUNTERCLOCKWISE == orient)
        ret.reverse_orientation();
    return ret;
}

vector<Polygon_2> loadPolygons(ifstream &is) {
    size_t number_of_polygons = 0;
    is >> number_of_polygons;
    vector<Polygon_2> ret;
    while (number_of_polygons--)
        ret.push_back(loadPolygon(is));
    return ret;
}

vector<Point_2> loadPoints(ifstream &is, int numberOfRobots) {
    vector<Point_2> points;
    for(int i=0;i<numberOfRobots;i++)
        points.emplace_back(loadPoint_2(is));

    return points;
}

int main(int argc, char *argv[]) {
    try
    {
        CMDManager cmd(argc, argv);

        remove(PATH_POINTS_FILE);
        remove(ENHANCER_PATH_POINTS_FILE);

        ifstream inputRobotsFile(cmd.robotFile), inputObstaclesFile(cmd.obstecalesFile);
        if (!inputRobotsFile.is_open() || !inputObstaclesFile.is_open()) {
            if (!inputRobotsFile.is_open()) cerr << "ERROR: Couldn't open file: " << cmd.robotFile << endl;
            if (!inputObstaclesFile.is_open()) cerr << "ERROR: Couldn't open file: " << cmd.obstecalesFile << endl;
            return -1;
        }
        int numberOfRobots;
        inputRobotsFile >> numberOfRobots;
        vector<Point_2> start = loadPoints(inputRobotsFile, numberOfRobots);
        vector<Point_2> end = loadPoints(inputRobotsFile, numberOfRobots);
        inputRobotsFile.close();

        Polygon_2 outer_obstacle = loadPolygon(inputObstaclesFile);
        vector<Polygon_2> obstacles = loadPolygons(inputObstaclesFile);
        inputObstaclesFile.close();

        boost::timer pathFinderTimer;
        WorkingSpace ws(outer_obstacle, obstacles, cmd.exportPoints, cmd.pointsPerSquare);
        ws.printStatistics(cmd.printStatistics);

        PathFinder finder(ws, start, end );
        Path path = finder.findPath(cmd.searchRadius, cmd.heuristicMultiplier, cmd.cutPath);
        if(!path.legal)
        {
            cout << "Time " << pathFinderTimer.elapsed() << " secs" << endl;
            finder.printStatistics(true);
            ws.exportPoints(PATH_POINTS_FILE, path);
            throw "no path found";
        }

        double pathFinderSecs = pathFinderTimer.elapsed();
        cout << "Path created: " << pathFinderTimer.elapsed() << " secs" << endl;
        cout << "Path length: " << path.length << endl;

        finder.printStatistics(cmd.printStatistics);
        path.printStatistics(cmd.printStatistics);
        ws.exportPoints(PATH_POINTS_FILE, path);

        path.exportPath(PATH_OUTPUT_FILE);
        if(cmd.enhancePath) {
            cout << "\nStart enhance path \n\n";
            boost::timer pathEnhanceTimer;

            ws.updatePointsByPath(path, cmd.enhancerPointsPerSquare, cmd.enhancerWidth);
            ws.printStatistics(cmd.printStatistics);

            PathFinder enhancedFinder(ws, start, end, false);
            Path enhancedPath = enhancedFinder.findPath(cmd.enhancerSearchRadius,
                                                        cmd.enhancerHeuristicMultiplier,
                                                        cmd.cutPath, cmd.pointEnforcement);
            if(!enhancedPath.legal)
            {
                enhancedFinder.printStatistics(true);
                ws.exportPoints(ENHANCER_PATH_POINTS_FILE, enhancedPath);
                throw "enhanced path failed!";
            }
            double pathEnhanceSecs = pathEnhanceTimer.elapsed();
            cout << "Path enhanced in:      " << pathEnhanceSecs << " secs" << endl;
            cout << "Total path find time is:      " << pathFinderSecs + pathEnhanceSecs << " secs" << endl;
            cout << "Path enhanced length: " << enhancedPath.length << endl;
            enhancedFinder.printStatistics(cmd.printStatistics);
            enhancedPath.printStatistics(cmd.printStatistics);
            ws.exportPoints(ENHANCER_PATH_POINTS_FILE, enhancedPath);
            enhancedPath.exportPath(ENHANCER_PATH_OUTPUT_FILE);
        }

    }
    catch (const char* c)
    {
        cout << "ERROR: " << c << endl;
        return 0;
    }
    return 0;
}
