#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <boost/timer.hpp>

#include "CGAL_defines.h"
#include "PathFinder.h"
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

list<ConfigurationPoint> findPath(vector<Point_2>& start, vector<Point_2>& end,
                                        Polygon_2 &outer_obstacle, vector<Polygon_2> &obstacles) {
    WorkingSpace ws(outer_obstacle, obstacles);
    PathFinder finder(ws);
    if(finder.findPath(start, end))
        return finder.fetchPath();
    throw "no path found";
}

int main(int argc, char *argv[]) {
    if (argc != 4) {
        cerr << "[USAGE]: inputRobots inputObstacles outputFile" << endl;
        return 1;
    }

    ifstream inputRobotsFile(argv[1]), inputObstaclesFile(argv[2]);
    if (!inputRobotsFile.is_open() || !inputObstaclesFile.is_open()) {
        if (!inputRobotsFile.is_open()) cerr << "ERROR: Couldn't open file: " << argv[1] << endl;
        if (!inputObstaclesFile.is_open()) cerr << "ERROR: Couldn't open file: " << argv[2] << endl;
        return -1;
    }
    int numberOfRobots;
    inputRobotsFile >> numberOfRobots;
    vector<Point_2> start = loadPoints(inputRobotsFile, numberOfRobots);
    vector<Point_2> end = loadPoints(inputRobotsFile, numberOfRobots);
    inputRobotsFile.close();

    auto outer_obstacle = loadPolygon(inputObstaclesFile);
    auto obstacles = loadPolygons(inputObstaclesFile);
    inputObstaclesFile.close();
    try {


        boost::timer timer;
        auto result = findPath(start, end, outer_obstacle, obstacles);
        auto secs = timer.elapsed();
        cout << "Path created:      " << secs << " secs" << endl;

        ofstream outputFile;
        outputFile.open(argv[3]);
        if (!outputFile.is_open()) {
            cerr << "ERROR: Couldn't open file: " << argv[3] << endl;
            return -1;
        }
        outputFile << result.size() << endl;
        for (ConfigurationPoint &cp : result) {
            for(Point_2& robot: cp.robots)
            {
                outputFile << robot.x().to_double() << " " << robot.y().to_double() << " ";
            }
            outputFile << endl;
        }
        outputFile.close();
    }
    catch (const char* c)
    {
        cout << "ERROR: " << c << endl;
        return 0;
    }
    return 0;
}
