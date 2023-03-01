#pragma once 

#include <stdlib.h>
#include <vector>
#include <string>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include "utils.hpp"
#include "CoveragePlotHelper.h"

class MyDecomposition {

      public: 

            MyDecomposition();
            ~MyDecomposition();

            bool init(shared_ptr<CGAL::Polygon_2<K>> poly); 
            void run(); 

            /**
             * @brief Get the Decomposition object
             * 
             * @return pair<Polygon_list, vector<K::Point_2>> 
             */
            pair<Polygon_list, vector<K::Point_2>> getDecomposition(); 
             
            


      private:
            
            /**
             * @brief poligono iniziale 
             * 
             */
            shared_ptr<CGAL::Polygon_2<K>> m_initialPolygon;
            
            /**
             * @brief vector di sottopoligoni in cui viene decomposto il poligono iniziale
             * 
             */
            vector<shared_ptr<CGAL::Polygon_2<K>>> m_decomposition;


            /**
             * @brief lista di sottopoligoni di indici 
             * 
             */
            Polygon_list m_indexedPolygons; 

            /**
             * @brief vector dei vertici. 
             * m_vertices[i] contiene il punto (K::Point_2) di indice i 
             * 
             */
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
                
};