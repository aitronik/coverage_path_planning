#pragma once

#include <iostream>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <math.h>
#include <CGAL/Point_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Simple_cartesian.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point;
typedef CGAL::Polygon_2<K> Polygon;

using namespace std;

vector<Point> readFromFile();
void printPoint(Point p);
Polygon createPolygon(vector<Point> points) ;
void plotPoints(string name, vector<Point> inPoints,  cv::Mat image);
cv::Mat plotPolygon(string name, Polygon p,float resolution);
vector<Point> extractVertices(Polygon p);
