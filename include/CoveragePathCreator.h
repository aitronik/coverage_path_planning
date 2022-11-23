#pragma once 

#include <stdlib.h>
#include <vector>
#include <string>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/partition_2.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/property_map.h>
#include <CGAL/Polyline_simplification_2/simplify.h>
#include "utils.hpp"
#include "CoveragePlotHelper.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Partition_traits_2<K, CGAL::Pointer_property_map<K::Point_2>::type > Partition_traits_2;
typedef Partition_traits_2::Point_2 Point;
typedef Partition_traits_2::Polygon_2 Polygon;  // a polygon of INDICES
typedef std::list<Polygon> Polygon_list;

namespace PS = CGAL::Polyline_simplification_2;
typedef PS::Stop_above_cost_threshold Stop;


using namespace std;




class CoveragePathCreator {
    public:
      CoveragePathCreator();
      ~CoveragePathCreator();
      bool init( vector<pair<float,float>> points, float sweepDistance, int m_decompositionType );
      bool run();
      vector<pair<float,float>> getFinalPath();
      void setAddPerimeterToPath(bool b); 
      

    private:
        vector<K::Point_2> m_perimeterVertices;
        shared_ptr<CGAL::Polygon_2<K>> m_initialPolygon;
        shared_ptr<CGAL::Polygon_2<K>> m_approximatePolygon;
        Polygon_list m_partitionPolys;
        vector<vector<K::Point_2>> m_intersections;
        vector<shared_ptr<CGAL::Polygon_2<K>>> m_polygonsForPath;
        vector<vector<vector<int>>> m_adj; //matrice di adiacenza 
        vector<int> m_polygonsSorted; 
        vector<vector<CGAL::Segment_2<K>>> m_pathS; 
        vector<CGAL::Segment_2<K>> m_finalPath;
        vector<K::Point_2> m_pathToReturn;
        bool m_addPerimeterToPath;
        int m_decompositionType;
        string m_decompositionName;
        bool m_doPlotting;
        float m_sweepDistance;
        CoveragePlotHelper m_Helper;
        K::Point_2 m_firstVertex;


        bool decompose();
        void createAdjMatrix();
        bool orderSubPolygons();
        tuple<float, K::Point_2> maxDistance(vector<K::Point_2>& points, K::Segment_2& segment); 
        tuple<CGAL::Segment_2<K>, K::Point_2> findSweepDirection(shared_ptr<CGAL::Polygon_2<K>> polygon);
        vector<CGAL::Line_2<K>> createGrid(shared_ptr<CGAL::Polygon_2<K>> polygon,  CGAL::Segment_2<K> sweepDirection, K::Point_2 point);
        vector<CGAL::Segment_2<K>> generatePathForOnePolygon(int cont , vector<bool>& borders);
        vector<int> findMinRoute(int start);
        int indexOfMinimum(vector<float>& dist, bool* visited);
        int initialIndex(float a, float b, float c, float d);  //tra le 4 distanza restituisce 0,1,2,3 in base a qual è la minore 
        int numAdiacency(int node);
        void Dijkstra(vector<vector<int>> &graph, int sorg , vector<float>& distances);
        void cover();
        vector<K::Point_2> generateGridForOnePolygon(int cont , vector<bool>& borders);
        vector<CGAL::Segment_2<K>> generatePathForOnePolygon(vector<K::Point_2> intersections, int start);
        
};