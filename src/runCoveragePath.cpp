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
// #include "CoveragePathCreator.h"
// #include "CoveragePlotHelper.h"
#include "PolygonCreator.h"
#include "CoveragePath.h"
#include "pathPoints.h"
#include "print.h"


#include <chrono>

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

    string filename_gemma = "a.txt";
    string filename_sede = "path_sede.txt";

    pathPoints mission;
    mission.init();
    mission.loadPoints(filename_mission);

    pathPoints perimeter;
    perimeter.init();
    perimeter.loadPoints(filename_perimeter);

    pathPoints sede;
    sede.init();
    sede.loadPoints(filename_sede);

    vector<pathPoints> banned_area;
    for(int i = 4; i <= 3 + number_of_banned_areas; i++){

        string filename_banned_area = argv[i];
        pathPoints ba;
        ba.init();
        ba.loadPoints(filename_banned_area);

        banned_area.push_back(ba);

    }

    pathPoints gemma;
    gemma.init();
    gemma.loadPoints(filename_gemma);

    /*Path*/
    vector<pair<float, float>> points_path;
    float minx = 0;
    float miny = 0;
    for(int i = 0; i < mission.points.size(); i=i+20){
        
        pair<float, float> point;
        ll2planar(mission.points.at(0).first, mission.points.at(0).second, mission.points.at(i).first, mission.points.at(i).second, &point.first, &point.second);

        if(point.first < minx){
            minx = point.first;
        }

        if(point.second < miny){
            miny = point.second;
        }

        points_path.push_back(point);
 
    }

    vector<K::Point_2> path_cgal;
    for(int i = 0; i < points_path.size(); i++){

        K::Point_2 p(points_path.at(i).first - 2*minx, points_path.at(i).second - 2*miny);
        path_cgal.push_back(p);

    }

    /*Cleaning levels*/
    // vector<int> cleaning_levels;
    // cleaning_levels.resize(path_cgal.size());
    // for(int i = 0; i < path_cgal.size(); i++){
    //     if(i >=0 && i < 10){
    //         cleaning_levels[i] = 1;
    //     }
    //     else if(i >=10 && i < 30){
    //         cleaning_levels[i] = 2;
    //     }
    //     else if(i >=30 && i < 45){
    //         cleaning_levels[i] = 0;
    //     }
    //     else if(i >=45 && i < 50){
    //         cleaning_levels[i] = 3;
    //     }
    //     else if(i >=50 && i < 70){
    //         cleaning_levels[i] = 1;
    //     }
    //     else if(i >=70 && i < 80){
    //         cleaning_levels[i] = 0;
    //     }
    //     else if(i >=80 && i < 100){
    //         cleaning_levels[i] = 2;
    //     }
    //     else if(i >=100 && i < 110){
    //         cleaning_levels[i] = 3;
    //     }
    //     else if(i >=110 && i < path_cgal.size()){
    //         cleaning_levels[i] = 1;
    //     }
        
    // }

    /*Sede*/
    vector<K::Point_2> sede_cgal;

    vector<float> x_sede; 
    x_sede.resize(sede.points.size());

    vector<float> y_sede;
    y_sede.resize(sede.points.size());

    vector<pair<float, float>> coordinates;
    coordinates.resize(sede.points.size());

    for(int i = 0; i < sede.points.size(); i++){
        x_sede[i] = sede.points.at(i).first;
        y_sede[i] = sede.points.at(i).second;
        coordinates[i] = {sede.points.at(i).first, sede.points.at(i).second};
    }

    
    auto itx = std::unique(coordinates.begin(), coordinates.end());
    coordinates.erase(itx, coordinates.end());

    for(int i = 0; i < coordinates.size(); i = i + 10){
        K::Point_2 p(coordinates.at(i).first, coordinates.at(i).second);
        sede_cgal.push_back(p);
    }
    

    cout << sede_cgal.size() << endl;

    // sede_cgal.clear();
    // sede_cgal = path_cgal;

    vector<int> cleaning_levels;
    cleaning_levels.resize(sede_cgal.size());
    for(int i = 0; i < sede_cgal.size(); i++){
        if(i >=0 && i < int(sede_cgal.size()/10)){
            cleaning_levels[i] = 1;
        }
        else if(i >=int(sede_cgal.size()/10) && i < 3*int(sede_cgal.size()/10)){
            cleaning_levels[i] = 2;
        }
        else if(i >=3*int(sede_cgal.size()/10) && i < 5*int(sede_cgal.size()/10)){
            cleaning_levels[i] = 0;
        }
        else if(i >=5*int(sede_cgal.size()/10) && i < 6*int(sede_cgal.size()/10)){
            cleaning_levels[i] = 3;
        }
        else if(i >=6*int(sede_cgal.size()/10) && i < 7*int(sede_cgal.size()/10)){
            cleaning_levels[i] = 1;
        }
        else if(i >=7*int(sede_cgal.size()/10) && i < 9*int(sede_cgal.size()/10)){
            cleaning_levels[i] = 2;
        }
        else if(i >=9*int(sede_cgal.size()/10) && i < sede_cgal.size()){
            cleaning_levels[i] = 1;
        }
        
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

    /*Area for Gemma class*/
    vector<K::Point_2> gemma_cgal;
    for(int i = 0; i < gemma.points.size(); i++){

        K::Point_2 p(gemma.points.at(i).first, gemma.points.at(i).second);
        gemma_cgal.push_back(p);

    }

    /*Contouring class*/
    PolygonCreator pc;
    float fake_offset = 0.001;
    float contour_offset = 0.4;
    float area_threshold = 1;
    float perimeter_offset = 0.5;
    bool apply_contouring = true;

    //pc.init(fake_offset, contour_offset, area_threshold, perimeter_offset, apply_contouring, cleaning_levels);

    
    //CGAL::Polygon_with_holes_2<K> final_contour = pc.createPolygonFromPath(path_cgal);

    //CGAL::Polygon_with_holes_2<K> perimeter_contour = pc.createPolygon(perimeter_cgal, banned_areas_cgal);

    perimeter_offset = 0.4;

    CoveragePath cp;
    cp.init(0, 0, sede_cgal, {});
    cp.setPolygonCreatorInit(fake_offset, contour_offset, area_threshold, perimeter_offset, apply_contouring, cleaning_levels);
    cp.run();

    

    // /*Test*/
    // TEST(Perimeter, ContourDistance){

    //     for(int i = 0; i < perimeter_cgal.size() - 1; i++){
    //         K::Segment_2 s_perimeter(perimeter_cgal.at(i), perimeter_cgal.at(i+1));
    //         K::Segment_2 s_contour(perimeter_contour.outer_boundary().vertex(i), perimeter_contour.outer_boundary().vertex(i+1));
    //         float distance = CGAL::squared_distance(s_perimeter, s_contour);
    //         EXPECT_EQ(sqrt(distance), perimeter_offset) << "Different distance at point n° " << i << endl;
    //     }

    // }

    
 
    return 0;

}
