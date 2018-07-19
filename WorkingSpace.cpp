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

WorkingSpace::WorkingSpace(Polygon_2 &outer_poly, vector<Polygon_2> &obstacles) :
        outer_poly(outer_poly), obstacles(obstacles){
    this->createArrangment();
    this->setRandomPoints();
}

vector<Point_2> WorkingSpace::getNeighbors(Point_2 p, FT radius) {
    vector<Point_2> L1;
    Fuzzy_sphere rc1(p, radius);
    tree.search(std::back_inserter(L1), rc1);
    return L1;
}

void WorkingSpace::insertPoints(vector<Point_2> points) {
    for(Point_2& s: points) {
        if (!this->inLegalFace(s))
            throw "robot first position is not legal";
        tree.insert(s);
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
        if(!it->contained())
            continue;
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

    uniform_real_distribution<double> xUnif = uniform_real_distribution<double>(CGAL::to_double(minx), CGAL::to_double(maxx));
    uniform_real_distribution<double> yUnif = uniform_real_distribution<double>(CGAL::to_double(miny), CGAL::to_double(maxy));
    std::default_random_engine re;

    int numberOfPoints = (int)(faceSize*NUM_OF_POINTS_PER_SQUARE);
    vector<Point_2> cpoints;
    int numOfPoints = 0;
    for(int i=0; i<numberOfPoints; i++) {
        Point_2 p = {xUnif(re), yUnif(re)};
        if (inLegalFace(p))
        {
            tree.insert(p);
            numOfPoints++;
        }
    }
    //cout << "num of points " << numOfPoints << endl;

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