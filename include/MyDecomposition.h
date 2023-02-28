#pragma once 

#include <stdlib.h>
#include <vector>
#include <string>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
// #include <CGAL/property_map.h>
// #include <CGAL/intersections.h>
#include "utils.hpp"
#include "CoveragePlotHelper.h"


// //POI TOGLI 
// #include <opencv2/core/core.hpp>
// #include <opencv2/imgproc.hpp>
// #include <opencv2/highgui.hpp>

class MyDecomposition {

      public: 

            MyDecomposition();
            ~MyDecomposition();

            bool init(shared_ptr<CGAL::Polygon_2<K>> poly); 
            pair<Polygon_list, vector<K::Point_2>> run(); 

            /**
             * @brief Get the Decomposition object
             * 
             * @return pair<Polygon_list, vector<K::Point_2>> 
             */
            pair<Polygon_list, vector<K::Point_2>> getDecomposition(); 
             
            


      private:
            
            shared_ptr<CGAL::Polygon_2<K>> m_initialPolygon;
            vector<shared_ptr<CGAL::Polygon_2<K>>> m_decomposition;
            Polygon_list m_indexedPolygons; //lista di poligoni di indici
            vector<K::Point_2> m_vertices; //contiene le corrispondenze indice-punto 

            /**
             * @brief 
             * 
             * @param poly 
             * @param startingVertex 
             * @return pair<CGAL::Segment_2<K>, int> 
             */
            pair<CGAL::Segment_2<K>, int> calculateCutter(shared_ptr<CGAL::Polygon_2<K>> poly, int &startingVertex);
            
            /**
             * @brief 
             * 
             * @param perimeter 
             * @return size_t 
             */
            size_t isConcave(shared_ptr<CGAL::Polygon_2<K>> perimeter);
            
            /**
             * @brief Get the Ith Indexed Polygon object
             * 
             * @param i 
             * @return Polygon 
             */
            Polygon getIthIndexedPolygon(size_t i);


            /**
             * @brief 
             * 
             * @return pair<int, int> 
             */
            pair<int, int> allSubPolygonsConvex ();
            
            /**
             * @brief 
             * 
             * @param p 
             * @return int 
             */
            int findIndex(K::Point_2 p);

            /**
             * @brief taglia il poligono poly e inserisce le due parti in m_decomposition , fa la stessa cosa per gli indici con idexededPoly e m_indexededPolygons.
             * 
             * @param poly ddd
             * @param newEdge 
             * @param startVertex 
             * @param endEdge 
             */
            void cutPolygon(shared_ptr<CGAL::Polygon_2<K>> poly/*, Polygon indexededPoly*/, CGAL::Segment_2<K> newEdge, int startVertex , int endEdge);
            
            //POI TOGLI 
            CoveragePlotHelper m_Helper;

    
};