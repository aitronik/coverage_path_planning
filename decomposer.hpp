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
#include "utils.hpp"

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point;
typedef CGAL::Polygon_2<K> Polygon;
typedef CGAL::Vector_2<K> Vector;
typedef CGAL::Line_2<K> Line;

using namespace std;
#define resolution (0.05)

vector<Point> findCriticalPoints(vector<shared_ptr<Polygon>> polygons);
vector<shared_ptr<Polygon>> decompose(vector<shared_ptr<Polygon>> polygons, vector<Point> criticalPoints);
