#include "CgalPlotHelper.h"


/*************************************/
CgalPlotHelper::CgalPlotHelper(){

    m_initialPathImage = cv::Mat(1000,1500, CV_8UC3, cv::Scalar(255,255,255));
    m_polygonsImage = cv::Mat(1000,1500, CV_8UC3, cv::Scalar(255,255,255));
    m_contourImage = cv::Mat(1000,1500, CV_8UC3, cv::Scalar(255,255,255));
    m_contourWithoutHolesImage = cv::Mat(1000,1500, CV_8UC3, cv::Scalar(255,255,255));
    m_contourPerimeterImage = cv::Mat(1000,1500, CV_8UC3, cv::Scalar(255,255,255));
    m_filledAreas = cv::Mat(1000,1500, CV_8UC3, cv::Scalar(255,255,255));
    m_decomposedPolys = cv::Mat(1000,1500, CV_8UC3, cv::Scalar(255,255,255));
    m_barycenterPolys = cv::Mat(1000,1500, CV_8UC3, cv::Scalar(255,255,255));

}

/*************************************/
CgalPlotHelper::~CgalPlotHelper(){

}
    
/*************************************/
void CgalPlotHelper::plotPath(vector<K::Point_2> path, 
                            cv::Mat pathImage, 
                            int B_fill,
                            int G_fill,
                            int R_fill,
                            double alpha,
                            bool polygon, 
                            std::string name, 
                            bool fill, 
                            bool resolution,
                            vector<int> cleaningLvl
                            ){

    if(resolution == true){
        m_covph.calculateResolution(path);
    }
    
    int B, G, R;
    vector<cv::Point> path_opencv;
    for(int i = 0; i < path.size(); i++){
        cv::Point point_opencv_tmp(m_covph.pixelFromMetres(path.at(i).x()), m_covph.pixelFromMetres(path.at(i).y()));
        path_opencv.push_back(point_opencv_tmp);

        if(i != 0){
            switch(cleaningLvl[i-1]){
                case 0:
                    B = 255;
                    G = 0;
                    R = 255;
                    break;
                
                case 1:
                    B = 255;
                    G = 0;
                    R = 0;
                    break;

                case 2:
                    B = 0;
                    G = 255;
                    R = 0;
                    break;

                case 3:
                    B = 0;
                    G = 0;
                    R = 255;
                    break;

                default:
                    B = 0;
                    G = 0;
                    R = 0;
            }

            cv::line(pathImage, path_opencv.at(i-1), path_opencv.at(i), cv::Scalar(B,G,R), 2, 4, 0);
        }
    }

    if(polygon == true){
        cv::Point first_point_opencv(m_covph.pixelFromMetres(path.at(0).x()), m_covph.pixelFromMetres(path.at(0).y()));
        cv::Point end_point_opencv(m_covph.pixelFromMetres(path.at(path.size() - 1).x()), m_covph.pixelFromMetres(path.at(path.size() - 1).y()));
        
        path_opencv.push_back(first_point_opencv);
        
        cv::line(pathImage, first_point_opencv, end_point_opencv, cv::Scalar(B,G,R), 2, 8, 0);
    }
    
    if(fill){
        vector<vector<cv::Point>> polygons_opencv;
        polygons_opencv.push_back(path_opencv);

        cv::Mat pathImageTmp = cv::Mat(1000,1500, CV_8UC3, cv::Scalar(255,255,255,255));

        cv::fillPoly(pathImageTmp, polygons_opencv, cv::Scalar(B_fill, G_fill, R_fill), 8, 0);
        cv::addWeighted(pathImageTmp, alpha, pathImage, 1, 0, pathImage);

    }

}

/*************************************/
void CgalPlotHelper::printPolygonsWithHolesK(CGAL::Polygon_with_holes_2<K> polygon, 
                                            cv::Mat pathImage, 
                                            bool fill, 
                                            int B_fill,
                                            int G_fill,
                                            int R_fill,
                                            double alpha,
                                            std::string name, 
                                            bool resolution,
                                            bool pathORpolygon,
                                            bool printOut,
                                            vector<int> cleaningLvl){
                                                
    vector<CGAL::Point_2<K>> outer_border;
    for(int i = 0; i < polygon.outer_boundary().size(); i++){
        K::Point_2 outer_point(CGAL::to_double(polygon.outer_boundary().vertex(i).x()), CGAL::to_double(polygon.outer_boundary().vertex(i).y())); 
        outer_border.push_back(outer_point);
    }

    plotPath(outer_border, pathImage, B_fill, G_fill, R_fill, alpha, true, name, fill, resolution, cleaningLvl);

    if(polygon.number_of_holes() != 0){
        for(int i = 0; i < polygon.number_of_holes(); i++){

            vector<CGAL::Point_2<K>> hole;
            for(int j = 0; j < polygon.holes().at(i).size(); j++){
                K::Point_2 vertex(CGAL::to_double(polygon.holes().at(i).vertex(j).x()), CGAL::to_double(polygon.holes().at(i).vertex(j).y()));
                hole.push_back(vertex);
            }

            plotPath(hole, pathImage, B_fill, G_fill, R_fill, alpha, true, name, fill, resolution, cleaningLvl);
        }
    }
    

    if(printOut == true){
        cv::imshow(name, pathImage);
        cv::waitKey(0);
    }
    
}

/*************************************/
void CgalPlotHelper::printPolygonsWithHolesK2(CGAL::Polygon_with_holes_2<K2> polygon, 
                                            cv::Mat pathImage, 
                                            bool fill, 
                                            int B_fill,
                                            int G_fill,
                                            int R_fill,
                                            double alpha,
                                            std::string name, 
                                            bool resolution,
                                            bool pathORpolygon,
                                            bool printOut,
                                            vector<int> cleaningLvl){
                                                
    vector<CGAL::Point_2<K>> outer_border;
    for(int i = 0; i < polygon.outer_boundary().size(); i++){
        K::Point_2 outer_point(CGAL::to_double(polygon.outer_boundary().vertex(i).x()), CGAL::to_double(polygon.outer_boundary().vertex(i).y())); 
        outer_border.push_back(outer_point);
    }

    plotPath(outer_border, pathImage, B_fill, G_fill, R_fill, alpha, true, name, fill, resolution,cleaningLvl);

    if(polygon.number_of_holes() != 0){
        for(int i = 0; i < polygon.number_of_holes(); i++){

            vector<CGAL::Point_2<K>> hole;
            for(int j = 0; j < polygon.holes().at(i).size(); j++){
                K::Point_2 vertex(CGAL::to_double(polygon.holes().at(i).vertex(j).x()), CGAL::to_double(polygon.holes().at(i).vertex(j).y()));
                hole.push_back(vertex);
            }

            plotPath(hole, pathImage, B_fill, G_fill, R_fill, true, alpha, name, fill, resolution, cleaningLvl);
        }
    }
    

    if(printOut == true){
        cv::imshow(name, pathImage);
        cv::waitKey(0);
    }
    
}

/*************************************/
void CgalPlotHelper::plotOverlappedAreas(vector<pair<CGAL::Polygon_with_holes_2<K>, int>> polygons, CGAL::Polygon_with_holes_2<K2> contour){
    
    cv::Mat overlay = m_polygonsImage.clone();
    m_filledAreas = m_polygonsImage.clone();
    
    for(int i = 0; i < polygons.size(); i++){
        vector<cv::Point> path_opencv;
        vector<vector<cv::Point>> polygons_opencv;

        int B,G,R;
        switch(polygons.at(i).second){
            case 0:
                B = 255;
                G = 0;
                R = 255;
                break;
            
            case 1:
                B = 255;
                G = 0;
                R = 0;
                break;

            case 2:
                B = 0;
                G = 255;
                R = 0;
                break;

            case 3:
                B = 0;
                G = 0;
                R = 255;
                break;

            default:
                B = 0;
                G = 0;
                R = 0;
        }
    
        for(int j = 0; j < polygons.at(i).first.outer_boundary().size(); j++){
            cv::Point point_opencv_tmp(m_covph.pixelFromMetres(CGAL::to_double(polygons.at(i).first.outer_boundary().vertex(j).x())), m_covph.pixelFromMetres(CGAL::to_double(polygons.at(i).first.outer_boundary().vertex(j).y())));
            path_opencv.push_back(point_opencv_tmp);
        }
        cv::Point point_opencv_zero(m_covph.pixelFromMetres(CGAL::to_double(polygons.at(i).first.outer_boundary().vertex(0).x())), m_covph.pixelFromMetres(CGAL::to_double(polygons.at(i).first.outer_boundary().vertex(0).y())));
        path_opencv.push_back(point_opencv_zero);

        polygons_opencv.push_back(path_opencv);

        
        cv::fillPoly(overlay, polygons_opencv, cv::Scalar(B, G, R), 8, 0);

        for(int j = 0; j < polygons.at(i).first.number_of_holes(); j++){
            vector<cv::Point> hole_opencv;
            vector<vector<cv::Point>> holes_opencv;
            for(int k = 0; k < polygons.at(i).first.holes().at(j).size(); k++){
                cv::Point point_opencv_hole(m_covph.pixelFromMetres(CGAL::to_double(polygons.at(i).first.holes().at(j).vertex(k).x())), m_covph.pixelFromMetres(CGAL::to_double(polygons.at(i).first.holes().at(j).vertex(k).y())));
                hole_opencv.push_back(point_opencv_hole);
            }
            holes_opencv.push_back(hole_opencv);
            cv::fillPoly(overlay, holes_opencv, cv::Scalar(255, 255, 255), 8, 0);
        }

        double alpha = 0.2;
        cv::addWeighted(overlay, alpha, m_filledAreas, 1 - alpha, 0, m_filledAreas);

        overlay = m_filledAreas.clone();
    }

    

    cv::imshow("Filled Areas", m_filledAreas);
    cv::waitKey(0);
}