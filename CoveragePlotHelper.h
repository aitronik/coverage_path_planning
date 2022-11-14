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
        void plotInitialPerimeter(shared_ptr<CGAL::Polygon_2<K>> poly);
        void plotCoveredPerimeter(shared_ptr<CGAL::Polygon_2<K>> poly);        
        void plotSubPolygon(const Polygon& poly,  vector<K::Point_2>& points, int num, string decomposition_name);
        void plotPathForConvexPolygon(vector<CGAL::Segment_2<K>> grid , shared_ptr<CGAL::Polygon_2<K>> poly);
        void plotLineForTest(CGAL::Line_2<K> line, string decomposition_name);
    private: 

        cv::Mat m_initial_image;
        cv::Mat m_image_decomposition;
        cv::Mat m_image_path;
        float m_resolution; //rapporto pixel/metri

        float pixelFromMetres (float x);
        void calculateResolution(vector<K::Point_2>& perimeter_vertices);
        
};
