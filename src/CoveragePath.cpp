#include "CoveragePath.h"

/*************************************/
CoveragePath::CoveragePath(){

    input_type = 0;
    clean_level = 0;

}

/*************************************/
CoveragePath::~CoveragePath(){

}

/*************************************/
bool CoveragePath::init(int type, int cleanlvl, vector<K::Point_2> path, vector<vector<K::Point_2>> banned_areas_cgal){

    input_type = type;
    clean_level = cleanlvl;
    input_path = path;
    keep_out_zones = banned_areas_cgal;

}

/*************************************/
CGAL::Polygon_with_holes_2<K> CoveragePath::getInputPolygon(){

    CGAL::Polygon_with_holes_2<K> poly_with_holes;
    switch(input_type){
        case PATH:
            poly_with_holes = pc.createPolygonFromPath(input_path);
            break;
        
        case PERIMETER:
            poly_with_holes = pc.createPolygon(input_path, keep_out_zones);
            break;

        default:
            cout << "Input type error. Please specify if the input path is a PATH (0) or PERIMETER (1)." << endl;
    }

}