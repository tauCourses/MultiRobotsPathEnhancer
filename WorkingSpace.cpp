#include "WorkingSpace.h"

void polygon_split_observer::after_split_face(Face_handle f1, Face_handle f2, bool)
{
    f2->set_contained(f1->contained());
}

void WorkingSpace::createArrangment() {
    Polygon_set_2 obstacles_set;

    if (outer_poly.is_clockwise_oriented())
        outer_poly.reverse_orientation();

    Polygon_with_holes_2 space(outer_poly, obstacles.begin(), obstacles.end());
    obstacles_set.insert(space);

    arr = obstacles_set.arrangement();

    //ensure that when face split two side safe their property (inside/outside)
    Polygon_set_2::Traits_2 traits;
    polygon_split_observer observer;
    observer.attach(arr);
    Kernel *ker = &traits;
    verticalDecomposition(*ker);
    observer.detach();

    pl.attach(arr);
}

WorkingSpace::WorkingSpace(Polygon_2 &outer_poly, vector<Polygon_2> &obstacles,
                           bool exportPointsEnabled, double pointsPerSquare) :
        outer_poly(outer_poly), obstacles(obstacles),
        exportPointsEnabled(exportPointsEnabled), pointsPerSquare(pointsPerSquare){
    this->createArrangment();
    this->setRandomPoints();

    //this->printArr();
}

vector<Point_2> WorkingSpace::getNeighbors(Point_2& p, double radius, int robotEnforcement) {

    vector<Point_2> L1;
    Fuzzy_sphere rc1(p, radius);
    tree.search(std::back_inserter(L1), rc1);
    if(robotEnforcement != -1)
    {
        L1.erase(std::remove_if(
                L1.begin(), L1.end(),
                [&](const Point_2& x) {
                    return pointsRobotEnforcement[x] != robotEnforcement;
                }), L1.end());
    }
    return L1;
}

void WorkingSpace::insertPoints(vector<Point_2> points, POINT_STATES state, bool robotEnforcement) {
    for(int i=0; i<points.size(); i++) {
        if (!this->inLegalFace(points[i]))
            throw "illegal position for a robot";
        tree.insert(points[i]);
        numberOfLegalPoints++;
        if (robotEnforcement)
            pointsRobotEnforcement.insert(pair<Point_2,int>(points[i], i));

        if(exportPointsEnabled)
            pointsStateMap.insert(pair<Point_2,POINT_STATES>(points[i], state));
    }
}

bool WorkingSpace::segmentQuery(Segment_2 seg) {
    vector<CGAL::Object> vecZoneElems;
    Face_handle hFace;

    CGAL::zone(arr, seg, std::back_inserter(vecZoneElems), pl);
    for (int i = 0; i < vecZoneElems.size(); ++i) {
        if (CGAL::assign(hFace, vecZoneElems[i]) && !hFace->contained())
            return false;
    }
    return true;
}

bool WorkingSpace::inLegalFace(const Point_2 &p) {
    CGAL::Object obj = pl.locate(p); //find p in pl

    Vertex_const_handle vertex;
    if (CGAL::assign(vertex, obj)) {
        Face_iterator it = arr.faces_begin();
        for(;it!=arr.faces_end();it++)
        {
            if(it == arr.unbounded_face())
                continue;

            ccb_haledge_circulator first = it->outer_ccb();
            ccb_haledge_circulator circ = first;
            do {
                Halfedge_const_handle temp = circ;
                if(temp->source()->point() == vertex->point())
                {
                    if(it->contained())
                        return true;
                    else
                        break;
                }
            } while (++circ != first);
        }
        return false;
    }

    Halfedge_const_handle  helfEdge; //check it's a halfedge
    if (CGAL::assign(helfEdge, obj)) {
        if (helfEdge->face()->contained())
            return true;
        else if(helfEdge->twin()->face()->contained())
            return true;
        return false;
    }

    // Check whether the point is contained inside a free bounded face.
    Face_const_handle face;
    if (CGAL::assign(face, obj)) //if obj is face
    {
        if(face->contained())
            return true;
    }
    return false;
}

void WorkingSpace::setRandomPoints() {
    Face_iterator it = arr.faces_begin();
    for(;it!=arr.faces_end();it++)
    {
        numberOfFaces++;
        if(!it->contained())
            continue;
        numberOfConatinedFaces++;
        setFaceRandomPoints(it);
    }
}

void WorkingSpace::setFaceRandomPoints(Face_handle face) {
    ccb_haledge_circulator first = face->outer_ccb();
    ccb_haledge_circulator circ = first;
    vector<Point_2> points;
    do {
        Halfedge_const_handle temp = circ;
        points.emplace_back(temp->source()->point());
    } while (++circ != first);
    FT minx = points[0].x();
    FT miny = points[0].y();
    FT maxx = minx;
    FT maxy = miny;

    for(Point_2 p:points)
    {
        if(p.x() < minx)
            minx = p.x();
        else if(p.x() > maxx)
            maxx = p.x();

        if(p.y() < miny)
            miny = p.y();
        else if(p.y() > maxy)
            maxy = p.y();
    }


    double faceSize = CGAL::to_double((maxx-minx) * (maxy - miny));
    std::random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<double> xUnif = uniform_real_distribution<double>(CGAL::to_double(minx), CGAL::to_double(maxx));
    uniform_real_distribution<double> yUnif = uniform_real_distribution<double>(CGAL::to_double(miny), CGAL::to_double(maxy));
   // std::default_random_engine re;

    int numberOfPointsInFace = (int)(faceSize*this->pointsPerSquare);
    numberOfPointsChecked += numberOfPointsInFace;
    vector<Point_2> cpoints;
    for(int i=0; i<numberOfPointsInFace; i++) {
        Point_2 p = {xUnif(gen), yUnif(gen)};
        if (inLegalFace(p))
        {
            tree.insert(p);
            numberOfLegalPoints++;
            if(this->exportPointsEnabled)
                pointsStateMap.insert(pair<Point_2,POINT_STATES>(p, POINT_CREATED));
        }
    }
}

void WorkingSpace::addVerticalSegment(Vertex_handle v, CGAL::Object obj, Kernel &ker) {
    X_monotone_curve_2 seg;
    Vertex_const_handle vh;
    Halfedge_const_handle hh;
    Face_const_handle fh;
    Vertex_handle v2;

    if (CGAL::assign(vh, obj)) { // The given feature is a vertex.
        seg = X_monotone_curve_2(v->point(), vh->point());
        v2 = arr.non_const_handle(vh);
    } else if (CGAL::assign(hh, obj)) { // The given feature is a halfedge.
        if (hh->is_fictitious()) //We ignore fictitious halfedges.
            return;

        // Check whether v lies in the interior of the x-range of the edge (in
        // which case this edge should be split).
        const typename Kernel::Compare_x_2 cmp_x = ker.compare_x_2_object();
        if (cmp_x(v->point(), hh->target()->point()) == CGAL::EQUAL) {
            // In case the target of the edge already has the same x-coordinate as
            // the vertex v, just connect these two vertices.
            seg = X_monotone_curve_2(v->point(), hh->target()->point());
            v2 = arr.non_const_handle(hh->target());
        }
        else {
            // Compute the vertical projection of v onto the segment associated
            // with the halfedge. Split the edge and connect v with the split point.
            Line_2 Line;
            Line_2 supp_line(hh->source()->point(), hh->target()->point());
            Line_2 vert_line(v->point(), Point_2(v->point().x(), v->point().y() + 1));
            Point_2  point;
            CGAL::assign(point, ker.intersect_2_object()(supp_line, vert_line));
            seg = X_monotone_curve_2(v->point(), point);
            arr.split_edge(arr.non_const_handle(hh),
                           X_monotone_curve_2(hh->source()->point(), point),
                           X_monotone_curve_2(point, hh->target()->point()));
            v2 = arr.non_const_handle(hh->target());
        }
    } else // Ignore faces and empty objects.
        return;

    // Add the vertical segment to the arrangement using its two end vertices.
    arr.insert_at_vertices(seg, v, v2);
}

void WorkingSpace::verticalDecomposition(Kernel &ker) {
    typedef pair<Vertex_const_handle, pair<CGAL::Object, CGAL::Object> > Vd_entry;

    // For each vertex in the arrangment, locate the feature that lies
    // directly below it and the feature that lies directly above it.
    list<Vd_entry>   vd_list;
    CGAL::decompose(arr, back_inserter(vd_list));

    // Go over the vertices (given in ascending lexicographical xy-order),
    // and add segements to the feautres below and above it.
    const typename Kernel::Equal_2 equal = ker.equal_2_object();
    typename list<Vd_entry>::iterator  it, prev = vd_list.end();
    for (it = vd_list.begin(); it != vd_list.end(); ++it) {
        // If the feature above the previous vertex is not the current vertex,
        // add a vertical segment to the feature below the vertex.
        Vertex_const_handle v;
        if ((prev == vd_list.end()) ||
            !CGAL::assign(v, prev->second.second) ||
            !equal(v->point(), it->first->point()))
            addVerticalSegment(arr.non_const_handle(it->first), it->second.first, ker);
        // Add a vertical segment to the feature above the vertex.
        addVerticalSegment(arr.non_const_handle(it->first), it->second.second, ker);
        prev = it;
    }
}

void WorkingSpace::printArr()
{
    cout << "number of faces - " << arr.number_of_faces() << endl;
    Face_iterator it = arr.faces_begin();
    for(;it!=arr.faces_end();it++)
    {
        if(it != arr.unbounded_face()) {
            ccb_haledge_circulator first = it->outer_ccb();
            ccb_haledge_circulator circ = first;
            do {
                Halfedge_const_handle temp = circ;
                cout << temp->source()->point() << " ";
            } while (++circ != first);
            cout << endl;
        } else
            cout << "unboanded!\n";
        if(it->contained())
            cout << "contained!" <<endl;
        else
            cout << "not contained!" <<endl;
    }
}

void WorkingSpace::updatePointMap(Point_2& p, POINT_STATES state) {
    if(!this->exportPointsEnabled)
        return;
    auto it = pointsStateMap.find(p);
    if (it != pointsStateMap.end()) {
        if(it->second<state)
            it->second = state;
    }
}

void WorkingSpace::exportPoints(string fileName, Path& path) {
    if(!this->exportPointsEnabled)
        return;

    if(path.legal)
        for (CPoint& cp : path.cPoints)
            if(cp->robotChangedIndex!=-1)
                updatePointMap(cp->robots[cp->robotChangedIndex], POINT_IN_PATH);

    ofstream outputFile;
    outputFile.open(fileName);

    if(!this->exportPointsEnabled)
        return;

    for (auto it=pointsStateMap.begin(); it!=pointsStateMap.end(); ++it)
        outputFile << it->second << " " << it->first.x().to_double() << " " << it->first.y().to_double() << endl;

    outputFile.close();

}

void WorkingSpace::printStatistics(bool print) {
    if(!print)
        return;

    cout << "WORKING SPACE STATISTICS:\n";
    cout << "number of faces " << numberOfFaces << endl;
    cout << "number of contained faces " << numberOfConatinedFaces << endl;
    cout << "number of points checked " << numberOfPointsChecked << endl;
    cout << "number of legal points  " << numberOfLegalPoints << endl;
}

void WorkingSpace::updatePointsByPath(Path &path, double pointsPerSquare, double rectWidth) {
    this->pointsPerSquare = pointsPerSquare;
    this->rectWidth = rectWidth;
    tree.clear();
    pointsStateMap.clear();
    numberOfPointsChecked = 0;
    numberOfLegalPoints = 0;

    CPoint temp = path.end;

    while (temp != path.start) {
        if(temp->robotChangedIndex!=-1) {
            tree.insert(temp->robots[temp->robotChangedIndex]);
            pointsRobotEnforcement.insert(pair<Point_2,int>(temp->robots[temp->robotChangedIndex], temp->robotChangedIndex));
            numberOfLegalPoints++;
            setSegmentRandomPoints({temp->robots[temp->robotChangedIndex],
                                    temp->last->robots[temp->robotChangedIndex]},
                                    temp->robotChangedIndex);
            if(this->exportPointsEnabled)
                pointsStateMap.insert(pair<Point_2,POINT_STATES>(temp->robots[temp->robotChangedIndex], POINT_CREATED));
        }
        temp = temp->last;

    }
    if(this->exportPointsEnabled)
    {
        for(Point_2& p:path.end->robots)
            updatePointMap(p, END_POINT);
        for(Point_2& p:temp->robots)
            pointsStateMap.insert(pair<Point_2,POINT_STATES>(p, START_POINT));
    }
}

void WorkingSpace::setSegmentRandomPoints(Segment_2 seg, int robot) {
    Vector_2 primary(seg);
    Vector_2 secondary = primary.perpendicular(CGAL::CLOCKWISE);

    secondary = secondary / sqrt(CGAL::to_double(secondary.squared_length()));

    double faceSize = sqrt(CGAL::to_double(primary.squared_length())) * rectWidth;

    std::random_device rd;
    mt19937 gen(rd());
    auto xUnif = uniform_real_distribution<double>(0, 1);
    auto yUnif = uniform_real_distribution<double>(-rectWidth/2, rectWidth/2);

    int numberOfPointsInFace = (int)(faceSize*this->pointsPerSquare);
    numberOfPointsChecked += numberOfPointsInFace;
    vector<Point_2> cpoints;
    for(int i=0; i<numberOfPointsInFace; i++) {
        Point_2 p = seg.source() +  primary * xUnif(gen) + secondary * yUnif(gen);
        if (inLegalFace(p))
        {
            tree.insert(p);
            pointsRobotEnforcement.insert(pair<Point_2,int>(p, robot));
            numberOfLegalPoints++;
            if(this->exportPointsEnabled)
                pointsStateMap.insert(pair<Point_2,POINT_STATES>(p, POINT_CREATED));
        }
    }

}
