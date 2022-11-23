#pragma once 


#include <stdlib.h>
#include <vector>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/partition_2.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/property_map.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "utils.hpp" //controlla se serve


typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Partition_traits_2<K, CGAL::Pointer_property_map<K::Point_2>::type > Partition_traits_2;
typedef Partition_traits_2::Point_2 Point;
typedef Partition_traits_2::Polygon_2 Polygon;  // a polygon of INDICES
typedef std::list<Polygon> Polygon_list;

using namespace std;

class CoveragePlotHelper {
    public: 

        CoveragePlotHelper();
        ~CoveragePlotHelper();
        bool init(vector<K::Point_2>& perimeter_vertices);
        void plotPerimeter(shared_ptr<CGAL::Polygon_2<K>> poly);
        void plotCoveredPerimeter(shared_ptr<CGAL::Polygon_2<K>> poly);        
        void plotSubPolygon(const Polygon& poly,  vector<K::Point_2>& points, int num, string decomposition_name);
        void plotPathForConvexPolygon(vector<CGAL::Segment_2<K>> grid  /*, shared_ptr<CGAL::Polygon_2<K>> poly*/);
        void plotLineForTest(CGAL::Line_2<K> line);
        void plotFinalPath(vector<CGAL::Segment_2<K>> path, vector<K::Point_2> pointsToPrint,  K::Point_2 start);
        void updatePerimeterImage(shared_ptr<CGAL::Polygon_2<K>> new_poly);

    private: 

        cv::Mat m_perimeterImage;
        cv::Mat m_image_decomposition;
        cv::Mat m_image_path;
        string m_decompositionName;
        float m_resolution; //rapporto pixel/metri
        //così si alternano i colori 
        float pixelFromMetres (float x);
        void calculateResolution(vector<K::Point_2>& perimeter_vertices);
        
};
