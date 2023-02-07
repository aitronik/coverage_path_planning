#include <stdlib.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/partition_2.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/Point_2.h>
#include <CGAL/draw_triangulation_2.h>
#include <CGAL/Aff_transformation_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/General_polygon_set_2.h>
#include <CGAL/intersections.h>
#include <CGAL/Cartesian.h>
#include <CGAL/enum.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/draw_polygon_with_holes_2.h>
#include <CGAL/draw_polygon_2.h>
#include <CGAL/create_offset_polygons_from_polygon_with_holes_2.h>
#include <CGAL/create_offset_polygons_2.h>
#include <CGAL/Straight_skeleton_2.h>
#include <math.h>
#include <cmath>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/partition_2.h>
#include <cassert>
#include <list>
#include <boost/shared_ptr.hpp>
#include "utils.hpp"
#include "CoveragePathCreator.h"
#include "CoveragePlotHelper.h"
#include "PathContouring.h"
#include "pathPoints.h"
#include "print.h"

#include <CGAL/create_straight_skeleton_2.h> 
#include <CGAL/Kernel/global_functions.h>

#include <gtest/gtest.h>

#define TEST(Perimeter, ContourDistance);

using namespace std;

typedef CGAL::Aff_transformation_2<K> Transformation;
typedef CGAL::Exact_predicates_exact_constructions_kernel K2;


void ll2planar(const double latOrig,
               const double lonOrig,
               const double lat,
               const double lon,
               float *north,
               float *east)
{
    /* This function implements Flamsteed or natural projection */
    /* The function permits to convert Lat/Long to linear coordinate */
    double equatRad = 6378137;/* EllipsoidVector[ReferenceEllipsoid].EquatorialRadius; */
    double eccentSq = 0.00669438;/* EllipsoidVector[ReferenceEllipsoid].EccentricitySquared; */
    const double deg2rad = M_PI/180.0;

    double latRad = lat * deg2rad;
    double lonRad = lon * deg2rad;

    double latOrigRad = latOrig * deg2rad;
    double lonOrigRad = lonOrig * deg2rad;
    /* compute polar radius */
    double polarRad = sqrt((1 - eccentSq) * equatRad * equatRad);
    /* planar */
    double planarN = equatRad * equatRad /
            sqrt(equatRad * equatRad * cos(latOrigRad) * cos(latOrigRad) +
                 polarRad * polarRad * sin(latOrigRad) * sin(latOrigRad));

    east[0] = planarN * cos(latRad) * (lonRad - lonOrigRad);
    north[0] = planarN * (latRad - latOrigRad);
}

int main(int argc, char* argv[]) {

    /*Loading data*/
    string filename_mission = argv[1];
    string filename_perimeter = argv[2];
    int number_of_banned_areas = stoi(argv[3]);

    pathPoints mission;
    mission.init();
    mission.loadPoints(filename_mission);

    pathPoints perimeter;
    perimeter.init();
    perimeter.loadPoints(filename_perimeter);

    vector<pathPoints> banned_area;
    for(int i = 4; i <= 3 + number_of_banned_areas; i++){

        string filename_banned_area = argv[i];
        pathPoints ba;
        ba.init();
        ba.loadPoints(filename_banned_area);

        banned_area.push_back(ba);

    }

    /*Path*/
    vector<pair<float, float>> points_path;
    for(int i = 0; i < mission.points.size(); i=i+20){
        
        pair<float, float> point;
        ll2planar(mission.points.at(0).first, mission.points.at(0).second, mission.points.at(i).first, mission.points.at(i).second, &point.first, &point.second);

        points_path.push_back(point);
 
    }

    vector<K::Point_2> path_cgal;
    for(int i = 0; i < points_path.size(); i++){

        K::Point_2 p(points_path.at(i).first, points_path.at(i).second);
        path_cgal.push_back(p);

    }

    /*Perimeter*/
    vector<K::Point_2> perimeter_cgal;
    for(int i = 0; i < perimeter.points.size(); i++){

        K::Point_2 p(perimeter.points.at(i).first, perimeter.points.at(i).second);
        perimeter_cgal.push_back(p);

    }

    /*Banned area*/
    vector<vector<K::Point_2>> banned_areas_cgal;
    for(int i = 0; i < banned_area.size(); i++){

        vector<K::Point_2> banned_area_cgal;

        for(int j = 0; j < banned_area.at(i).points.size(); j++){
            K::Point_2 p(banned_area.at(i).points.at(j).first, banned_area.at(i).points.at(j).second);
            banned_area_cgal.push_back(p);
        }
        
        banned_areas_cgal.push_back(banned_area_cgal);

    }

    /*Contouring class*/
    PathContouring pc;
    float fake_offset = 0.001;
    float contour_offset = 0.1;
    float area_threshold = 0.03;
    float perimeter_offset = 0.05;

    pc.init(fake_offset, contour_offset, area_threshold, perimeter_offset);

    pc.returnFinalContour(path_cgal);
    CGAL::Polygon_with_holes_2<K> perimeter_contour = pc.returnFinalPerimeter(perimeter_cgal, banned_areas_cgal);

    /*Test*/
    TEST(Perimeter, ContourDistance){

        for(int i = 0; i < perimeter_cgal.size() - 1; i++){
            K::Segment_2 s_perimeter(perimeter_cgal.at(i), perimeter_cgal.at(i+1));
            K::Segment_2 s_contour(perimeter_contour.outer_boundary().vertex(i), perimeter_contour.outer_boundary().vertex(i+1));
            float distance = CGAL::squared_distance(s_perimeter, s_contour);
            EXPECT_EQ(sqrt(distance), perimeter_offset) << "Different distance at point n° " << i << endl;
        }

    }
 
    return 0;

}
