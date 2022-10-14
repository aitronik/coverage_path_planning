#pragma once

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/draw_polygon_2.h>
#include <CGAL/Segment_2.h>
#include <CGAL/Vector_2.h>
#include <CGAL/Direction_2.h>
#include <CGAL/Triangulation_data_structure.h>
#include <vector>
#include <cassert>
#include <list>
#include <tuple>
#include <iostream>
#include <boost/variant/get.hpp>


typedef CGAL::Exact_predicates_inexact_constructions_kernel K;

using namespace std;

tuple<float, K::Point_2> maxDistance(vector<K::Point_2>& points, K::Segment_2& segment);
tuple<CGAL::Segment_2<K>, K::Point_2> findSweepDirection(shared_ptr<CGAL::Polygon_2<K>> polygon);
vector<K::Point_2> divideSegment(K::Segment_2 segment, float range);
vector<CGAL::Line_2<K>> createGrid(shared_ptr<CGAL::Polygon_2<K>> polygon,  CGAL::Segment_2<K> sweepDirection, K::Point_2 point, float range);
K::Point_2* intersect_polygon_line(shared_ptr<CGAL::Polygon_2<K>> polygon, CGAL::Line_2<K> line);
vector<K::Point_2> generatePath( shared_ptr<CGAL::Polygon_2<K>> polygon, float range); 
