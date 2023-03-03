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
pair<vector<pair<CGAL::Polygon_with_holes_2<K>, int>>, CGAL::Polygon_with_holes_2<K2>> CoveragePath::getInputPolygon(){

    pair<vector<pair<CGAL::Polygon_with_holes_2<K>, int>>, CGAL::Polygon_with_holes_2<K2>> poly_with_holes;
    switch(input_type){
        case PATH:
            poly_with_holes = pc.createPolygonFromPath(input_path);
            break;
        
        case PERIMETER:
            //poly_with_holes = pc.createPolygon(input_path, keep_out_zones);
            break;

        default:
            cout << "Input type error. Please specify if the input path is a PATH (0) or PERIMETER (1)." << endl;
    }

    return poly_with_holes;

}

/*************************************/
void CoveragePath::run(){

    pair<vector<pair<CGAL::Polygon_with_holes_2<K>, int>>, CGAL::Polygon_with_holes_2<K2>> polygonAndContour = getInputPolygon();
    vector<pair<CGAL::Polygon_with_holes_2<K>, int>> polygon = polygonAndContour.first;
    CGAL::Polygon_with_holes_2<K2> contour = polygonAndContour.second;

    for(int i = 0; i < polygon.size(); i++){
        char str[80];
        sprintf(str, "%d:%d", i, i);

        vector<int>path_lvl(input_path.size(), 4);
        
        m_cph.plotPath(input_path, m_cph.m_initialPathImage, 0, 0, 0, 255, false, str, false, true, path_lvl); 
        vector<int>m_cleaningLvl(polygon.at(i).first.outer_boundary().size(), polygon.at(i).second);
        m_cph.printPolygonsWithHolesK(polygon.at(i).first, m_cph.m_initialPathImage, false, 0, 0, 0, 255, str, false, true, true, m_cleaningLvl);
        m_cph.printPolygonsWithHolesK(polygon.at(i).first, m_cph.m_polygonsImage, false, 0, 0, 0, 255, "Final Polygons", false, true, false, m_cleaningLvl);

        
        if(i != polygon.size()-1){
            cv::destroyWindow(str);
        }
    
    }

    cout << polygon.size() << endl;

    vector<int>m_cleaningLvl(polygon.at(polygon.size()-1).first.outer_boundary().size(), polygon.at(polygon.size()-1).second);
    m_cph.printPolygonsWithHolesK(polygon.at(polygon.size()-1).first, m_cph.m_polygonsImage, false, 0, 0, 0, 255, "Final Polygons", false, true, true, m_cleaningLvl);
   
    m_cph.plotOverlappedAreas(polygon);

}