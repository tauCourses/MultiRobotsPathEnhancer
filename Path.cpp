#include "Path.h"

Path::Path(CPoint start, CPoint end, WorkingSpace& ws) : start(start), end(end){
    CPoint temp = end;

    while (temp != this->start) {
        cPoints.emplace_front(temp);
        numberOfPoints++;
        length += temp->distanceToConfiguration(temp->last);
        if(temp->robotChangedIndex!=-1)
            ws.updatePointMap(temp->robots[temp->robotChangedIndex], POINT_IN_PATH);
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
    {
        cout << "number of cPoints " << numberOfPoints << endl;
        cout << "path Length " << length << endl;
    } else
        cout << "Not legal path";
}

void Path::exportPath(char* file) {
    ofstream outputFile;
    outputFile.open(file);
    if (!outputFile.is_open())
        throw "ERROR: Couldn't open output file";

    outputFile << numberOfPoints << endl;
    for (CPoint cp : cPoints)
        outputFile << *cp << endl;
    outputFile.close();

}