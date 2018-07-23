#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <boost/timer.hpp>

#include "CGAL_defines.h"
#include "PathFinder.h"
#include "CMDManager.h"
using namespace std;


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
    CMDManager cmdManager(argc, argv);

    ifstream inputRobotsFile(cmdManager.robotFile), inputObstaclesFile(cmdManager.obstecalesFile);
    if (!inputRobotsFile.is_open() || !inputObstaclesFile.is_open()) {
        if (!inputRobotsFile.is_open()) cerr << "ERROR: Couldn't open file: " << cmdManager.robotFile << endl;
        if (!inputObstaclesFile.is_open()) cerr << "ERROR: Couldn't open file: " << cmdManager.obstecalesFile << endl;
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

    try {
        boost::timer timer;
        WorkingSpace ws(outer_obstacle, obstacles, cmdManager.exportPoints, cmdManager.pointsPerSquare);

        PathFinder finder(ws, cmdManager.searchRadius);
        Path path = finder.findPath(start, end);
        if(!path.legal)
        {
            finder.printStatistics(true);
            throw "no path found";
        }

        double secs = timer.elapsed();
        cout << "Path created:      " << secs << " secs" << endl;

        ws.printStatistics(cmdManager.printStatistics);
        finder.printStatistics(cmdManager.printStatistics);
        path.printStatistics(cmdManager.printStatistics);
        ws.exportPoints();

        path.exportPath(cmdManager.outputFile);

    }
    catch (const char* c)
    {
        cout << "ERROR: " << c << endl;
        return 0;
    }
    return 0;
}
