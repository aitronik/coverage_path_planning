#pragma once

#include <iostream>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <cassert>
#include <list>
#include <iterator>
#include <cmath>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/partition_2.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/property_map.h>
#include <CGAL/Point_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Vector_2.h>
// #include <opencv2/core/core.hpp>
// #include <opencv2/imgproc.hpp>
// #include <opencv2/highgui.hpp>
// #include "CoveragePlotHelper.h"


typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Partition_traits_2<K, CGAL::Pointer_property_map<K::Point_2>::type > Partition_traits_2;
typedef Partition_traits_2::Point_2 Point;
typedef Partition_traits_2::Polygon_2 Polygon;  // a polygon of INDICES
typedef std::list<Polygon> Polygon_list;


using namespace std;

vector<pair<float,float>> readFromFile(string filename); 
shared_ptr<CGAL::Polygon_2<K>> createPolygon(vector<K::Point_2> points) ;
// CGAL::Polygon_2<K> createPolygon(vector<K::Point_2> points) ;
void printInfo();
double calculateAngle (CGAL::Vector_2<K> v, CGAL::Vector_2<K> w);
bool adjacency(list<size_t> container1, list<size_t> container2, int& vertex_i, int& vertex_j );
K::Point_2* intersect_polygon_line(shared_ptr<CGAL::Polygon_2<K>> polygon, CGAL::Line_2<K> line);
int indexOf(vector<int> v, int x);
vector<K::Point_2> divideSegment(CGAL::Segment_2<K> segment, float initialSweepDistance);