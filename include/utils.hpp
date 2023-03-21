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
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/partition_2.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/property_map.h>
#include <CGAL/Point_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Vector_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Partition_traits_2<K, CGAL::Pointer_property_map<K::Point_2>::type > Partition_traits_2;
typedef Partition_traits_2::Point_2 Point;
typedef Partition_traits_2::Polygon_2 Polygon;  // a polygon of INDICES
typedef std::list<Polygon> Polygon_list;
// typedef Traits::Point_2 Point_2; //non ho molto chiaro cosa cambi tra questi e i Point


using namespace std;


/**
 * @brief legge da file i vertici  
 * @param filename nome del file 
 * @return vector<pair<float,float>> 
 */
vector<pair<float,float>> readFromFile(string filename); 

/**
 * @brief crea un polygon_2 
 * @param points vertici del polygon 
 * @return shared_ptr<CGAL::Polygon_2<K>> 
 */
shared_ptr<CGAL::Polygon_2<K>> createPolygon(vector<K::Point_2> points) ;

/**
 * @brief 
 * 
 * @param container1 
 * @param container2 
 * @param vertex_i 
 * @param vertex_j 
 * @param decomposedVertices 
 * @return true 
 * @return false 
 */
bool adjacency(list<size_t> container1, list<size_t> container2, int& vertex_i, int& vertex_j, vector<K::Point_2> decomposedVertices);

/**
 * @brief 
 * 
 * @param polygon 
 * @param p 
 * @param approx
 * @return true 
 * @return false 
 */
bool isPointIntoConvexPolygon(shared_ptr<CGAL::Polygon_2<K>> polygon, K::Point_2 p, float approx);

/**
 * @brief 
 * 
 * @param line1 
 * @param line2 
 * @param approximation  
 * @return vector<K::Point_2> 
 */
vector<K::Point_2> intersect_lines(CGAL::Line_2<K> line1, CGAL::Line_2<K> line2, float approximation);

/**
 * @brief 
 * 
 * @param line1 
 * @param line2 
 * @param sweepDistance 
 * @return vector<K::Point_2> 
 */
vector<K::Point_2> intersect_lines(CGAL::Line_2<K> line1, CGAL::Line_2<K> line2, float sweepDistance);

/**
 * @brief interseca un poligono con una retta
 * @param polygon 
 * @param line 
 * @param approx
 * @return K::Point_2* coppia di punti in cui si intersecano (eventualmente coincidenti)
 */
vector<K::Point_2> intersect_convex_polygon_line(shared_ptr<CGAL::Polygon_2<K>> polygon, CGAL::Line_2<K> line, float approx);



/**
 * @brief 
 * 
 * @param polygon 
 * @param edgeIndex 
 * @param vertexIndex 
 * @return pair<K::Point_2,int> 
 */
pair<K::Point_2,int> intersect_concave_polygon_at_index(shared_ptr<CGAL::Polygon_2<K>> polygon, size_t edgeIndex, size_t vertexIndex);


/**
 * @brief divide un segmento in punti equidistanti in base a distance (eventualmente viene ridotta)
 * @param segment 
 * @param initialSweepDistance 
 * @return vector<K::Point_2> 
 */
vector<K::Point_2> divideSegment(CGAL::Segment_2<K> segment, float distance);

/**
 * @brief 
 * 
 * @param a 
 * @param b 
 * @param c 
 * @return true 
 * @return false 
 */
 int isLeft(K::Point_2 a, K::Point_2 b, K::Point_2 c); 

/**
 * @brief 
 * 
 * @param a 
 * @param b 
 * @param approx
 * @return true 
 * @return false 
 */
bool arePointsEqual(K::Point_2 a, K::Point_2 b, float approx);

/**
 * @brief 
 * 
 * @param s1 
 * @param s2 
 * @param approx 
 * @return true 
 * @return false 
 */
bool areSegmentsEqual(CGAL::Segment_2<K> s1, CGAL::Segment_2<K> s2, float approx);


/**
 * @brief 
 * 
 * @param p 
 * @param points 
 * @return K::Point_2 
 */
K::Point_2 nearestPoint(K::Point_2 p, vector<K::Point_2> points);


/**
 * @brief 
 * 
 * @param segment1 
 * @param segment2 
 * @param approx 
 * @return pair<bool, CGAL::Segment_2<K>> 
 */
pair<bool, CGAL::Segment_2<K>> concatenateSegments(CGAL::Segment_2<K> segment1, CGAL::Segment_2<K> segment2, float approx);


