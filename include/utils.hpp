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

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Partition_traits_2<K, CGAL::Pointer_property_map<K::Point_2>::type > Partition_traits_2;
typedef Partition_traits_2::Point_2 Point;
typedef Partition_traits_2::Polygon_2 Polygon;  // a polygon of INDICES
typedef std::list<Polygon> Polygon_list;


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

// void printInfo();

/**
 * @brief calcola angolo tra due vettori in radianti (controllare che siano radianti )
 * @param v 
 * @param w 
 * @return double 
 */
double calculateAngle (CGAL::Vector_2<K> v, CGAL::Vector_2<K> w);

/**
 * @brief calcola i vertici di adiacenza tra due poligoni 
 * se ci sono due punti in comune tra container1 e container2 return true e salva i due vertici in vertex_i e vertex_2. 
 * Se c'è un solo punto di adiacenza salva punto,-1 . 
 * Altrimenti  salva -1 e -1 
 * @param container1 
 * @param container2 
 * @param vertex_i 
 * @param vertex_j 
 * @return true se c'è adiacenza 
 * @return false altrimenti
 */
bool adjacency(list<size_t> container1, list<size_t> container2, int& vertex_i, int& vertex_j );

/**
 * @brief interseca un poligono con una retta
 * @param polygon 
 * @param line 
 * @return K::Point_2* coppia di punti in cui si intersecano (eventualmente coincidenti)
 */
K::Point_2* intersect_polygon_line(shared_ptr<CGAL::Polygon_2<K>> polygon, CGAL::Line_2<K> line);

/**
 * @brief restituisce l'indice del vector a cui corrisponde il valore x
 * 
 * @param v 
 * @param x 
 * @return int 
 */
int indexOf(vector<int> v, int x);

/**
 * @brief divide un segmento in punti equidistanti in base a distance (eventualmente viene ridotta)
 * @param segment 
 * @param initialSweepDistance 
 * @return vector<K::Point_2> 
 */
vector<K::Point_2> divideSegment(CGAL::Segment_2<K> segment, float distance);