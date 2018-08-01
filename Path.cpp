#include "Path.h"

Path::Path(CPoint start, CPoint end) : start(start), end(end){
    CPoint temp = end;

    while (temp != this->start) {
        cPoints.emplace_front(temp);
        numberOfPoints++;
        length += temp->distanceToConfiguration(temp->last);
        temp = temp->last;
    }
    numberOfPoints++;
    cPoints.emplace_front(temp);
}

Path::Path() {
    this->legal = false;
}

void Path::printStatistics(bool print) {
    if(!print)
        return;

    cout << "PATH STATISTICS:\n";
    if(this->legal)
        cout << "number of configurations " << numberOfPoints << endl;
    else
        cout << "Not legal path";
}

void Path::exportPath(string file) {
    ofstream outputFile;
    outputFile.open(file);
    if (!outputFile.is_open())
        throw "ERROR: Couldn't open output file";

    outputFile << numberOfPoints << endl;
    for (CPoint& cp : cPoints)
        outputFile << *cp << endl;
    outputFile.close();

}
