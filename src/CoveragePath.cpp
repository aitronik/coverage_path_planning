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
void CoveragePath::setPolygonCreatorInit(float fake_offset, float contour_offset, float area_threshold, float perimeter_offset, bool apply_contouring, const vector<int>& cleaningLvl){

    pc.init(fake_offset, contour_offset, area_threshold, perimeter_offset, apply_contouring, cleaningLvl);

}

/*************************************/
void CoveragePath::setCoveragePathCreatorInit(CGAL::Polygon_2<K> polygon, float sweepDistance, int m_decompositionType){

    cpc.init(polygon, sweepDistance, m_decompositionType);

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

    return poly_with_holes;

}

/*************************************/
CGAL::Polygon_with_holes_2<K> CoveragePath::run(){

    CGAL::Polygon_with_holes_2<K> polygon = getInputPolygon();
    setCoveragePathCreatorInit(polygon.outer_boundary(), 0.05, 1);

    cpc.run();

    return polygon;

}