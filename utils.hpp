#pragma once

#include <iostream>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <cassert>
#include <list>
#include <iterator>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/partition_2.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/property_map.h>
#include <CGAL/Point_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Simple_cartesian.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>


typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Partition_traits_2<K, CGAL::Pointer_property_map<K::Point_2>::type > Partition_traits_2;
typedef Partition_traits_2::Point_2 Point;
typedef Partition_traits_2::Polygon_2 Polygon;  // a polygon of indices
typedef std::list<Polygon> Polygon_list;
// typedef CGAL::Polygon_2<K> Polygon;

using namespace std;

vector<K::Point_2> readFromFile(string filename); 
void printPoint(K::Point_2 p);
shared_ptr<CGAL::Polygon_2<K>> createPolygon(vector<K::Point_2> points) ;
//void plotPoints(const string& name, const vector<Point>& inPoints,  cv::Mat image);
void plotSubPolygon(cv::Mat image, const string name, /*shared_ptr<Polygon>& p*/ Polygon poly, vector<K::Point_2> points, float resolution);
void plotPolygon(cv::Mat image, const string name, shared_ptr<CGAL::Polygon_2<K>> poly, float resolution);
//vector<Point> extractVertices(shared_ptr<Polygon> p);
void printInfo();
void plotPathForConvexPolygon( vector<K::Point_2> grid ,shared_ptr<CGAL::Polygon_2<K>> poly, cv::Mat image, const string name, float resolution);
