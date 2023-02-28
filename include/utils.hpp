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
vector<pair<double,double>> readFromFile(string filename); 

/**
 * @brief crea un polygon_2 
 * @param points vertici del polygon 
 * @return shared_ptr<CGAL::Polygon_2<K>> 
 */
shared_ptr<CGAL::Polygon_2<K>> createPolygon(vector<K::Point_2> points) ;

// void printInfo();

/**
 * @brief calcola il coseno dell'angolo tra due vettori in radianti (controllare che siano radianti )
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
vector<K::Point_2> intersect_convex_polygon_line(shared_ptr<CGAL::Polygon_2<K>> polygon, CGAL::Line_2<K> line);



/**
 * @brief 
 * 
 * @param polygon 
 * @param edgeIndex 
 * @return pair<K::Point_2,int> 
 */
pair<K::Point_2,int> intersect_concave_polygon_at_index(shared_ptr<CGAL::Polygon_2<K>> polygon, int edgeIndex, int vertexIndex);

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
 * @brief restituisce -1 se il lato edgeIndex del poligono polygon non è collineare con nessuno dei due lati adiacenti, 
 * altrimenti restituisce l'indice del lato con cui è collineare 
 * @param polygon 
 * @param edgeIndex 
 * @return index of collinear edge 
 * @return -1 
 */
int isCollinear(shared_ptr<CGAL::Polygon_2<K>> polygon, int edgeIndex); 

/**
 * @brief 
 * 
 * @param p 
 */
void printPointCoordinates(K::Point_2 p); 

/**
 * @brief 
 * 
 * @param a 
 * @param b 
 * @return true 
 * @return false 
 */
bool areEqual(K::Point_2 a, K::Point_2 b);

/**
 * @brief 
 * 
 * @param p 
 * @param points 
 * @return K::Point_2 
 */
K::Point_2 nearestPoint(K::Point_2 p, vector<K::Point_2> points);