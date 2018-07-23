#include "ConfigurationPoint.h"

long ConfigurationPoint::objectCounter=1;

double ConfigurationPoint::distanceToConfiguration(CPoint dest) {
    double sum = 0;
    for (int i = 0; i < numberOfRobots; i++) {
        double xdiff = CGAL::to_double(this->robots[i].x()) - CGAL::to_double(dest->robots[i].x());
        double ydiff = CGAL::to_double(this->robots[i].y()) - CGAL::to_double(dest->robots[i].y());
        sum += sqrt(xdiff * xdiff + ydiff * ydiff);
    }

    return sum;
}

bool ConfigurationPoint::isConfigurationLegal() {
    for (int i = 0; i < numberOfRobots; i++) {
        for (int j = i + 1; j < numberOfRobots; j++)
        {
            if(abs(this->robots[i].x() - this->robots[j].x()) < 1 ||
               abs(this->robots[i].y() - this->robots[j].y()) < 1)
                return false;
        }
    }
    return true;
}

ConfigurationPoint::ConfigurationPoint(vector<Point_2>& robots) {
    this->robots = robots;
    this->numberOfRobots = static_cast<int>(robots.size());
}

ConfigurationPoint::ConfigurationPoint(CPoint old, int robotChangedIndex,
                                       Point_2& robotChangedNewPosition) {

    this->numberOfRobots = old->numberOfRobots;
    this->robots = old->robots;
    this->robots[robotChangedIndex] = std::move(robotChangedNewPosition);
    this->robotChangedIndex = robotChangedIndex;

}

bool ConfigurationPoint::operator<(const ConfigurationPoint &rhs) const {
    for(int i=0; i<this->numberOfRobots;i++)
    {
        if(this->robots[i] != rhs.robots[i])
            return this->robots[i] < rhs.robots[i];
    }
    return false;
}

bool ConfigurationPoint::operator!=(const ConfigurationPoint &rhs) const {
    for(int i=0; i<this->numberOfRobots;i++)
    {
        if(this->robots[i] != rhs.robots[i])
            return true;
    }
    return false;
}

ostream &operator<<(std::ostream &os, const ConfigurationPoint &cp)
{
    os << cp.robots[0].x().to_double() << " " << cp.robots[0].y().to_double();
    for(int i=1; i < cp.numberOfRobots; i++)
    {
        os << " " << cp.robots[i].x().to_double() << " " << cp.robots[i].y().to_double();
    }
    return os;
}
