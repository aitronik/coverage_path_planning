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
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>


typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Partition_traits_2<K, CGAL::Pointer_property_map<K::Point_2>::type > Partition_traits_2;
typedef Partition_traits_2::Point_2 Point;
typedef Partition_traits_2::Polygon_2 Polygon;  // a polygon of INDICES
typedef std::list<Polygon> Polygon_list;


using namespace std;

vector<K::Point_2> readFromFile(string filename); 
void printPoint(K::Point_2 p);
shared_ptr<CGAL::Polygon_2<K>> createPolygon(vector<K::Point_2> points) ;
float pixelFromMetres (float x, float resolution);
float calculateresolution(vector<K::Point_2> points);
void plotSubPolygon(cv::Mat image, const string name, Polygon poly, vector<K::Point_2> points, float resolution);
void plotPolygon(cv::Mat image, const string name, shared_ptr<CGAL::Polygon_2<K>> poly, float resolution);
void printInfo();
void plotPathForConvexPolygon( vector<CGAL::Segment_2<K>> grid ,shared_ptr<CGAL::Polygon_2<K>> poly, cv::Mat image, const string name, float resolution);
bool adjacency(list<size_t> container1, list<size_t> container2, int& vertex_i, int& vertex_j );
double calculateAngle (CGAL::Vector_2<K> v, CGAL::Vector_2<K> w);