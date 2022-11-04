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
#include <algorithm>
#include <cassert>
#include <list>
#include <tuple>
#include <iostream>
#include <cmath>
#include <CGAL/partition_2.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/property_map.h>
#include <bits/stdc++.h>
#include <CGAL/enum.h>
#include "utils.hpp"


typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Partition_traits_2<K, CGAL::Pointer_property_map<K::Point_2>::type > Partition_traits_2;
typedef Partition_traits_2::Polygon_2 Polygon;  // a polygon of indicesS
typedef std::list<Polygon> Polygon_list;

using namespace std;

tuple<float, K::Point_2> maxDistance(vector<K::Point_2>& points, K::Segment_2& segment);
tuple<CGAL::Segment_2<K>, K::Point_2> findSweepDirection(shared_ptr<CGAL::Polygon_2<K>> polygon);
vector<K::Point_2> divideSegment(K::Segment_2 segment, float sweep_distance);
vector<CGAL::Line_2<K>> createGrid(shared_ptr<CGAL::Polygon_2<K>> polygon,  CGAL::Segment_2<K> sweepDirection, K::Point_2 point, float sweep_distance);
K::Point_2* intersect_polygon_line(shared_ptr<CGAL::Polygon_2<K>> polygon, CGAL::Line_2<K> line);
vector<CGAL::Segment_2<K>> generatePathForOnePolygon( shared_ptr<CGAL::Polygon_2<K>> polygon, float sweep_distance, vector<bool> borders); 
// bool is_in( vector< int>& v, int k);
int indexOfMinimum(vector<float>& dist, bool* visited);
void Dijkstra(vector<vector<int>>& graph, int sorg, vector<float>& distances);
vector<int> sortPolygons (vector<vector<vector<int>>> adj);