#include "PolygonCreator.h"

PolygonCreator::PolygonCreator(){

    scale_footprint = 0.001;
    contour_offset = 0;
    perimeter_offset = 0;
    areaThreshold = 0;
    apply_contouring = true;

    initialPathImage = cv::Mat(1000,1500, CV_8UC3, cv::Scalar(255,255,255));
    polygonsImage = cv::Mat(1000,1500, CV_8UC3, cv::Scalar(255,255,255));
    contourImage = cv::Mat(1000,1500, CV_8UC3, cv::Scalar(255,255,255));
    contourWithoutHolesImage = cv::Mat(1000,1500, CV_8UC3, cv::Scalar(255,255,255));
    contourPerimeterImage = cv::Mat(1000,1500, CV_8UC3, cv::Scalar(255,255,255));

}

/*************************************/

PolygonCreator::~PolygonCreator(){

}

/*************************************/

bool PolygonCreator::init(float sf, float oc, float at, float po, float ap){

    scale_footprint = sf;
    contour_offset = oc;
    areaThreshold = at;
    perimeter_offset = po;
    apply_contouring = ap;
    CoveragePlotHelper cph;
    
    return true;

}

/*************************************/
vector<K::Vector_2> PolygonCreator::defineVectors(vector<K::Point_2> path){

    vector<K::Vector_2> pathVectors;
    for(int i = 0; i < path.size(); i++){
        if(i != 0){
            K::Vector_2 vector_tmp(path.at(i-1), path.at(i));
            pathVectors.push_back(vector_tmp);
        }
    }

    return pathVectors;

}

/*************************************/
vector<K::Segment_2> PolygonCreator::defineSegments(vector<K::Point_2> path){

    vector<K::Segment_2> path_segments;
    for(int i = 0; i < path.size(); i++){
        if(i != 0){
            K::Segment_2 segment_tmp(path.at(i-1), path.at(i));
            path_segments.push_back(segment_tmp);
        }
    }

    return path_segments;
}

/*************************************/
vector<K::Point_2> PolygonCreator::definePathFromSegments(vector<K::Segment_2> segments){
    
    vector<K::Point_2> path_from_segments;
    for(int i = 0; i < segments.size(); i++){

        K::Segment_2 segment = segments.at(i);
        K::Point_2 first_point(segment.point(0).x(), segment.point(0).y());
        K::Point_2 second_point(segment.point(1).x(), segment.point(1).y());

        if(i == 0){
            path_from_segments.push_back(first_point);
        }
        
        path_from_segments.push_back(second_point);
    }

    return path_from_segments;

}

/*************************************/
void PolygonCreator::plotPath(vector<K::Point_2> path, 
                            cv::Mat pathImage, 
                            int B, 
                            int G, 
                            int R, 
                            bool polygon, 
                            std::string name, 
                            bool fill, 
                            bool resolution,
                            int x_offset, 
                            int y_offset){

    if(resolution == true){
        cph.calculateResolution(path);
    }
    

    vector<cv::Point> path_opencv;
    for(int i = 0; i < path.size(); i++){
        cv::Point point_opencv_tmp(cph.pixelFromMetres(path.at(i).x()) + x_offset, cph.pixelFromMetres(path.at(i).y()) + y_offset);
        path_opencv.push_back(point_opencv_tmp);

        if(i != 0){
            cv::line(pathImage, path_opencv.at(i-1), path_opencv.at(i), cv::Scalar(B,G,R), 2, 4, 0);
        }
    }

    if(polygon == true){
        cv::Point first_point_opencv(cph.pixelFromMetres(path.at(0).x()) + x_offset, cph.pixelFromMetres(path.at(0).y()) + y_offset);
        cv::Point end_point_opencv(cph.pixelFromMetres(path.at(path.size() - 1).x()) + x_offset, cph.pixelFromMetres(path.at(path.size() - 1).y()) + y_offset);
        
        path_opencv.push_back(first_point_opencv);
        
        cv::line(pathImage, first_point_opencv, end_point_opencv, cv::Scalar(B,G,R), 2, 8, 0);
    }
    
    if(fill){
        vector<vector<cv::Point>> polygons_opencv;
        polygons_opencv.push_back(path_opencv);

        cv::fillPoly(pathImage, polygons_opencv, cv::Scalar(B,G,R), 8, 0);
    }

}

/*************************************/
void PolygonCreator::unitVector(K::Vector_2* vector){

    float norm = sqrt(pow(vector->x(), 2) + pow(vector->y(), 2));
    K::Vector_2 v_norm(vector->x()/norm, vector->y()/norm);
    *vector = v_norm;

}

/*************************************/
K::Vector_2 PolygonCreator::perpendicularVector(K::Vector_2 vector, bool down){

    K::Vector_2 vector_perpendicular;
    if(down == true){
        vector_perpendicular = vector.perpendicular(CGAL::Sign {CGAL::COUNTERCLOCKWISE});
    }
    else{
        vector_perpendicular = vector.perpendicular(CGAL::Sign {CGAL::CLOCKWISE});
    }

    return vector_perpendicular;

}

/*************************************/
vector<K::Point_2> PolygonCreator::segmentShift(vector<K::Point_2> path, bool down, float scale_footprint){

    K::Vector_2 initial_vector(path.at(0), path.at(1));
    K::Vector_2 perpendicular_vector;
    vector<K::Point_2> path_shifted;

    perpendicular_vector = perpendicularVector(initial_vector, down);
    unitVector(&perpendicular_vector);

    K::Point_2 first_point = path.at(0) + scale_footprint*perpendicular_vector;
    K::Point_2 second_point = first_point + initial_vector;

    path_shifted.push_back(first_point);
    path_shifted.push_back(second_point);

    return path_shifted;
}

/*************************************/
vector<K::Point_2> PolygonCreator::offsetPath(auto path, float scale_footprint, bool down){

    vector<K::Vector_2> path_vectors = defineVectors(path); 
    vector<K::Point_2> path_offset; 

    K::Vector_2 first_vector = path_vectors.at(0);
    K::Vector_2 first_vector_perpendicular;

    first_vector_perpendicular = perpendicularVector(first_vector, down);
    unitVector(&first_vector_perpendicular);

    K::Point_2 first_point_offset = path.at(0) + scale_footprint*first_vector_perpendicular;
    path_offset.push_back(first_point_offset);

    for(int i = 1; i < path.size()-1; i++){

        K::Vector_2 vector_1 = path_vectors.at(i-1);
        K::Vector_2 vector_perpendicular_1;

        vector_perpendicular_1 = perpendicularVector(vector_1, down);
        unitVector(&vector_perpendicular_1);
        K::Point_2 point_1 = path.at(i) + scale_footprint*vector_perpendicular_1;

        K::Vector_2 vector_2 = path_vectors.at(i);
        K::Vector_2 vector_perpendicular_2;

        vector_perpendicular_2 = perpendicularVector(vector_2, down);
        unitVector(&vector_perpendicular_2);
        K::Point_2 point_2 = path.at(i) + scale_footprint*vector_perpendicular_2;

        K::Point_2 mean_point((point_1.x() + point_2.x())/2, (point_1.y() + point_2.y())/2);
        path_offset.push_back(mean_point);

        if(CGAL::angle(path.at(i-1), path.at(i), path.at(i+1)) == 1){
            K::Vector_2 vector3(path.at(i), mean_point);
            K::Vector_2 vector3_perpendicular = perpendicularVector(vector3, !down);
            unitVector(&vector3_perpendicular);
            K::Point_2 mean_point2 = mean_point + 0.1*scale_footprint*vector3_perpendicular;
            path_offset.push_back(mean_point2);
        }

        
    }

    K::Vector_2 end_vector = path_vectors.at(path_vectors.size()-1);
    K::Vector_2 end_vector_perpendicular;

    end_vector_perpendicular = perpendicularVector(end_vector, down);
    unitVector(&end_vector_perpendicular);

    K::Point_2 end_point = path.at(path.size()-1) + scale_footprint*end_vector_perpendicular;

    path_offset.push_back(end_point);

    return path_offset;

}

/*************************************/
CGAL::Polygon_2<K> PolygonCreator::definePolygon(auto path_down, auto path_up){

    CGAL::Polygon_2<K> polygon;
    reverse(path_down.begin(), path_down.end());

    for(int i = 0; i < path_up.size(); i++){
        K::Point_2 point_up(CGAL::to_double(path_up.at(i).x()), CGAL::to_double(path_up.at(i).y()));
        polygon.push_back(point_up);
    }

    for(int i = 0; i < path_down.size(); i++){
        K::Point_2 point_down(CGAL::to_double(path_down.at(i).x()), CGAL::to_double(path_down.at(i).y()));
        polygon.push_back(point_down);
    }

    return polygon;

}

/*************************************/
void PolygonCreator::printPolygonsWithHoles(auto polygon, 
                                            CoveragePlotHelper cph, 
                                            cv::Mat pathImage, 
                                            bool fill, 
                                            int B, 
                                            int G, 
                                            int R, 
                                            std::string name, 
                                            bool resolution,
                                            bool pathORpolygon){
                                                
    int x_offset = 0;
    int y_offset = 0;
    if(pathORpolygon == true){
        x_offset = 100;
        y_offset = 500;
    }

    vector<CGAL::Point_2<K>> outer_border;
    for(int i = 0; i < polygon.outer_boundary().size(); i++){
        K::Point_2 outer_point(CGAL::to_double(polygon.outer_boundary().vertex(i).x()), CGAL::to_double(polygon.outer_boundary().vertex(i).y())); 
        outer_border.push_back(outer_point);
    }

    plotPath(outer_border, pathImage, B, G, R, true, name, fill, resolution, x_offset, y_offset);

    for(int i = 0; i < polygon.number_of_holes(); i++){

        vector<CGAL::Point_2<K>> hole;
        for(int j = 0; j < polygon.holes().at(i).size(); j++){
            K::Point_2 vertex(CGAL::to_double(polygon.holes().at(i).vertex(j).x()), CGAL::to_double(polygon.holes().at(i).vertex(j).y()));
            hole.push_back(vertex);
        }

        plotPath(hole, pathImage, B, G, R, true, name, fill, resolution, x_offset, y_offset);
    }
    
}

/*************************************/
CGAL::Polygon_2<K2> PolygonCreator::convertPoly2K2(CGAL::Polygon_2<K> polygon){

    CGAL::Polygon_2<K2> polygon_K2;
    for(int i = 0; i < polygon.size(); i++){
        K2::Point_2 polygon_point(CGAL::to_double(polygon.vertex(i).x()), CGAL::to_double(polygon.vertex(i).y()));
        polygon_K2.push_back(polygon_point);
    }

    return polygon_K2;

}

/*************************************/
CGAL::Polygon_2<K> PolygonCreator::convertPoly2K(CGAL::Polygon_2<K2> polygon){

    CGAL::Polygon_2<K> polygon_K;
    for(int i = 0; i < polygon.size(); i++){
        K::Point_2 polygon_point(CGAL::to_double(polygon.vertex(i).x()), CGAL::to_double(polygon.vertex(i).y()));
        polygon_K.push_back(polygon_point);
    }

    return polygon_K;

}

/*************************************/
CGAL::Polygon_with_holes_2<K2> PolygonCreator::convertPolyWithHoles2K2(CGAL::Polygon_with_holes_2<K> polygon_with_holes){

    CGAL::Polygon_2<K2> outer_boundary;
    for(int i = 0; i < polygon_with_holes.outer_boundary().size(); i++){
        K2::Point_2 point_outer_boundary(CGAL::to_double(polygon_with_holes.outer_boundary().vertex(i).x()), CGAL::to_double(polygon_with_holes.outer_boundary().vertex(i).y()));
        outer_boundary.push_back(point_outer_boundary);
    }

    CGAL::Polygon_with_holes_2<K2> poly_K2(outer_boundary);
    for(int i = 0; i < polygon_with_holes.number_of_holes(); i++){
        CGAL::Polygon_2<K2> hole = convertPoly2K2(polygon_with_holes.holes().at(i));
        poly_K2.add_hole(hole);
    }

    return poly_K2;

}

/*************************************/
CGAL::Polygon_with_holes_2<K> PolygonCreator::convertPolyWithHoles2K(CGAL::Polygon_with_holes_2<K2> polygon_with_holes){

    CGAL::Polygon_2<K> outer_boundary;
    for(int i = 0; i < polygon_with_holes.outer_boundary().size(); i++){
        K::Point_2 point_outer_boundary(CGAL::to_double(polygon_with_holes.outer_boundary().vertex(i).x()), CGAL::to_double(polygon_with_holes.outer_boundary().vertex(i).y()));
        outer_boundary.push_back(point_outer_boundary);
    }

    CGAL::Polygon_with_holes_2<K> poly_K(outer_boundary);
    for(int i = 0; i < polygon_with_holes.number_of_holes(); i++){
        CGAL::Polygon_2<K> hole = convertPoly2K(polygon_with_holes.holes().at(i));
        poly_K.add_hole(hole);
    }

    return poly_K;

}

/*************************************/
vector<pair<int, int>> PolygonCreator::pathSegmentation(vector<K::Point_2> path, 
                                                        vector<vector<K::Segment_2>>* segments_no_int, 
                                                        vector<vector<K::Segment_2>>* segments_int){

    /*
    The function iterates over all segments of the path and merges together into a vector of segments (segments_no_int) the subpaths where there are no
    intersections. The intersecting segments, on the other hand, are saved in another vector of segments (segments_int).
    */
    int jump_index = 0;
    int i = jump_index;
    

    vector<pair<int, int>> count_int;
    vector<K::Segment_2> path_segments = defineSegments(path);
    /*
    While loop which exits when the iteration index i (which is initialised to zero in the first step) is great than the length of the 
    vector of the path segments. In the while loop there is a for loop that iterates over the segments of the path. Given the i-th segment, a
    for loop is done on all subsequent segments and the segments intersecting with the i-th segment are identified. Pairs are then created 
    and saved in the segment vector segments_int. Segments that do not have intersections, on the other hand, are saved
    in segments_no_int. Whenever at least one intersection is found, the intersection_flag flag is set to true and the first for
    is interrupted with a break. The jump_index is updated to i+1 and the index i re-initialised with the new value of jump_index.
    In addition, the indexes of the intersections are saved in the count_int vector.
    */
    while(i < path_segments.size()){
        vector<K::Segment_2> segment_tmp;
        for(i = jump_index; i < path_segments.size(); i++){
            vector<K::Segment_2> intersect_tmp;
            K::Segment_2 first_segm = path_segments.at(i);
            bool intersection_flag = false;

            for(int j = i + 1; j < path_segments.size(); j++){
                CGAL::Segment_2<K> second_segm = path_segments.at(j);
                auto result = CGAL::intersection(first_segm, second_segm);
                K::Point_2 intersection_point;
                if(result){
                    intersection_point = boost::get<K::Point_2 >(*result);
                    if(intersection_point.x() != second_segm.point(0).x() && intersection_point.y() != second_segm.point(0).y()){
                        count_int.push_back({i, j});
                        intersection_flag = true;
                
                        intersect_tmp.push_back(first_segm);
                        intersect_tmp.push_back(second_segm);
                    }
                }
            }

            if(intersection_flag == true){
                jump_index = i + 1;
                segments_int->push_back(intersect_tmp);
                break;
            }

            segment_tmp.push_back(path_segments.at(i));
        }

        segments_no_int->push_back(segment_tmp);
    }

    return count_int;

}

/*************************************/
vector<CGAL::Polygon_with_holes_2<K>> PolygonCreator::defineIntersectionPolygons(vector<vector<K::Segment_2>> segments_int){

    vector<CGAL::Polygon_with_holes_2<K>> polygon_final_int;
    /*The code iterates along the first level of the input vector of segment vectors. Each step, therefore, considers a vector of segments.*/
    for(int i = 0; i < segments_int.size(); i++){

        vector<CGAL::Polygon_2<K>> poly_list_int;

        /*The second for loop runs through all the segments of the vector of segments selected in the previous for loop. Dummy polygons are generated
        with an infinitesimal thickness (scale_footprint, which is set to 0.001 metres by default) which are saved in a vector (poly_list_int). 
        Why generate polygons with infinitesimal thickness?
        CGAL works well with polygons and many functions (including contouring functions) can only be used on polygons. Creating polygons with infinitesimal
        thickness allows these functions to be exploited. The overlap error with the original path is negligible.*/
        for(int j = 0; j < segments_int.at(i).size(); j++){
            vector<K::Point_2> path_int;
            K::Segment_2 segment_tmp = segments_int.at(i).at(j);
            K::Point_2 segment_point1(segment_tmp.point(0).x(), segment_tmp.point(0).y());
            K::Point_2 segment_point2(segment_tmp.point(1).x(), segment_tmp.point(1).y());

            path_int.push_back(segment_point1);
            path_int.push_back(segment_point2);

            if(path_int.size() != 0){
                vector<K::Point_2> path_up_int = segmentShift(path_int, false, scale_footprint);
                vector<K::Point_2> path_down_int = segmentShift(path_up_int, true, scale_footprint);
                CGAL::Polygon_2<K> poly_int = definePolygon(path_down_int, path_up_int);

                if(poly_int.is_counterclockwise_oriented() == false){
                    poly_int.reverse_orientation();
                }

                poly_list_int.push_back(poly_int);
            }
        }

        if(poly_list_int.size() != 0){

            CGAL::Polygon_2<K2> polygon_K2_initial = convertPoly2K2(poly_list_int.at(0));
            CGAL::Polygon_with_holes_2<K2> polygon_with_holes_K2(polygon_K2_initial);

            for(int j = 1; j < poly_list_int.size(); j++){
                CGAL::Polygon_2<K2> polygon_K2 = convertPoly2K2(poly_list_int.at(j));
                CGAL::join(polygon_with_holes_K2, polygon_K2, polygon_with_holes_K2);
            }

            PolygonWithHolesPtrVector contour_ptr = CGAL::create_exterior_skeleton_and_offset_polygons_with_holes_2(contour_offset,polygon_with_holes_K2.outer_boundary());

            vector<CGAL::Polygon_with_holes_2<K>> contour;
            for(PolygonWithHolesPtrVector::const_iterator v = contour_ptr.begin(); v != contour_ptr.end(); ++v){
                contour.push_back(**v);
            }

            polygon_final_int.push_back(contour.at(0));
        }
    }

    return polygon_final_int;

}

/*************************************/
vector<CGAL::Polygon_with_holes_2<K>> PolygonCreator::defineIndipendentPolygons(vector<vector<K::Segment_2>> segments_no_int){

    vector<CGAL::Polygon_with_holes_2<K>> polygon_final_no_int;
    /*The code iterates along the first level of the input vector of segment vectors. Each step, therefore, considers a vector of segments.*/
    for(int k = 0; k < segments_no_int.size(); k++){

        vector<K::Segment_2> segments = segments_no_int.at(k);
        vector<K::Point_2> subpath = definePathFromSegments(segments);

        
        /*The second for loop runs through all the segments of the vector of segments selected in the previous for loop. Dummy polygons are generated
        with an infinitesimal thickness (scale_footprint, which is set to 0.001 metres by default) which are saved in a vector (poly_list_int). 
        Why generate polygons with infinitesimal thickness?
        CGAL works well with polygons and many functions (including contouring functions) can only be used on polygons. Creating polygons with infinitesimal
        thickness allows these functions to be exploited. The overlap error with the original path is negligible.*/
        if(subpath.size() != 0){
            vector<K::Point_2> path_up = offsetPath(subpath, scale_footprint, false);
            vector<K::Point_2> path_down = offsetPath(subpath, scale_footprint, true);

            CGAL::Polygon_2<K> polygon = definePolygon(path_down, path_up);

            CGAL::Polygon_with_holes_2<K> poly_with_holes(polygon);

            PolygonWithHolesPtrVector contour_ptr = CGAL::create_exterior_skeleton_and_offset_polygons_with_holes_2(contour_offset,polygon);

            if(contour_ptr.size() != 0){
                vector<CGAL::Polygon_with_holes_2<K>> contour;
                for(PolygonWithHolesPtrVector::const_iterator v = contour_ptr.begin(); v != contour_ptr.end(); ++v){
                    contour.push_back(**v);
                }

                polygon_final_no_int.push_back(contour.at(0));

            }
        }
    }

    return polygon_final_no_int;

}

/*************************************/
CGAL::Polygon_with_holes_2<K2> PolygonCreator::mergePolygons(vector<CGAL::Polygon_with_holes_2<K>> polygon_final_no_int, 
                                                            vector<CGAL::Polygon_with_holes_2<K>> polygon_final_int,
                                                            vector<pair<int, int>> count_int,
                                                            cv::Mat pathImage,
                                                            std::string name){
    
    int index_int = 0;
    int index_seg = 1;

    CGAL::Polygon_with_holes_2<K2> initial_polygon = convertPolyWithHoles2K2(polygon_final_no_int.at(0));
    CGAL::Polygon_with_holes_2<K2> contour_final = initial_polygon;

    printPolygonsWithHoles(initial_polygon, cph, pathImage, false, 255, 0, 0, name, false, true);

    int index = 0;
    int i = index;
    while(i < count_int.size()-1){

        for(i = index; i < count_int.size()-1; i++){
            if((count_int.at(i+1).first - count_int.at(i).first == 1 || count_int.at(i+1).first - count_int.at(i).first == 0)){
                CGAL::Polygon_with_holes_2<K> polygon_intersection(polygon_final_int.at(index_int));
                if(count_int.at(i+1).first - count_int.at(i).first == 1){
                    index_int = index_int + 1;
                }
                
                CGAL::Polygon_with_holes_2<K2> polygon_intersectionK2 = convertPolyWithHoles2K2(polygon_intersection);
                printPolygonsWithHoles(polygon_intersection, cph, pathImage, false, 0, 0, 255, name, false, true);
                CGAL::join(contour_final, polygon_intersectionK2, contour_final);

            }
            else{
                
                CGAL::Polygon_with_holes_2<K> polygon_no_intersection(polygon_final_no_int.at(index_seg));
                index_seg = index_seg + 1;
                CGAL::Polygon_with_holes_2<K2> polygon_no_intersectionK2 = convertPolyWithHoles2K2(polygon_no_intersection);
                printPolygonsWithHoles(polygon_no_intersection, cph, pathImage, false, 255, 0, 0, name, false, true);
                CGAL::join(contour_final, polygon_no_intersectionK2, contour_final);


                CGAL::Polygon_with_holes_2<K> polygon_intersection(polygon_final_int.at(index_int));
                index_int = index_int + 1;
                CGAL::Polygon_with_holes_2<K2> polygon_intersectionK2 = convertPolyWithHoles2K2(polygon_intersection);
                printPolygonsWithHoles(polygon_intersection, cph, pathImage, false, 0, 0, 255, name, false, true);
                CGAL::join(contour_final, polygon_intersectionK2, contour_final);

                index = i+1;

                break;
            }
        }
    }

    CGAL::Polygon_with_holes_2<K> polygon_no_intersection(polygon_final_no_int.at(polygon_final_no_int.size()-1));
    CGAL::Polygon_with_holes_2<K2> polygon_no_intersectionK2 = convertPolyWithHoles2K2(polygon_no_intersection);

    CGAL::join(contour_final, polygon_no_intersectionK2, contour_final);

    printPolygonsWithHoles(polygon_final_no_int.at(polygon_final_no_int.size()-1), cph, pathImage, false, 255, 0, 0, name, false, true); 
    cv::imshow(name, pathImage);
    cv::waitKey(0);

    return(contour_final);

}

/*************************************/
void PolygonCreator::deleteHoles(CGAL::Polygon_with_holes_2<K2>* contour, 
                                                                    float areaThreshold, 
                                                                    cv::Mat pathImage, 
                                                                    std::string name){

    for(auto hole = contour->holes_begin(); hole != contour->holes_end(); hole++){

        double hole_area = abs(CGAL::to_double(hole->area()));
        if(hole_area != 0){
            if(hole_area < areaThreshold){
                CGAL::Polygon_with_holes_2<K2> hole_to_delete(*hole);
                printPolygonsWithHoles(hole_to_delete, cph, pathImage, false, 255, 0, 0, name, false, true);
                contour->erase_hole(hole);
            }
        }
    }

}

/*************************************/
CGAL::Polygon_with_holes_2<K> PolygonCreator::selectMajorPolygon(vector<CGAL::Polygon_with_holes_2<K>> contours){

    CGAL::Polygon_with_holes_2<K> perimeter_contour;
    if(contours.size() == 0){
        perimeter_contour = contours.at(0);
    }
    else{

        double largest_area = 0.0;
        int largest_polygon = 0;
        for(int i = 0; i < contours.size(); i++){
            double polygon_area = abs(CGAL::to_double(contours.at(i).outer_boundary().area()));
            
            if(polygon_area > largest_area){
                largest_area = polygon_area;
                largest_polygon = i;
            }

        }

        perimeter_contour = contours.at(largest_polygon);
    }

    return perimeter_contour;

}

/*************************************/
CGAL::Polygon_2<K> PolygonCreator::polygonFromClosedPath(vector<K::Point_2> path){

    CGAL::Polygon_2<K> polygon;
    for(int i = 0; i < path.size(); i++){
        polygon.push_back(path.at(i));
    }

    return polygon;

}

/*************************************/
CGAL::Polygon_2<K> PolygonCreator::perimeterContour(CGAL::Polygon_2<K> perimeter_polygon){

    CGAL::Polygon_with_holes_2<K> perimeter_contour;
    PolygonWithHolesPtrVector contour_ptr = CGAL::create_interior_skeleton_and_offset_polygons_with_holes_2(perimeter_offset,perimeter_polygon);

    if(contour_ptr.size() != 0){
        vector<CGAL::Polygon_with_holes_2<K>> contours;
        for(PolygonWithHolesPtrVector::const_iterator v = contour_ptr.begin(); v != contour_ptr.end(); ++v){
            contours.push_back(**v);
        }

        perimeter_contour = selectMajorPolygon(contours);
    }

    return perimeter_contour.outer_boundary();

}

/*************************************/
void PolygonCreator::clonePolygon(CGAL::Polygon_with_holes_2<K>* final_poly_with_holes, CGAL::Polygon_2<K> poly_to_clone){

    for(int i = 0; i < poly_to_clone.size(); i++){
        final_poly_with_holes->outer_boundary().push_back(poly_to_clone.vertex(i));
    }

}

/*************************************/
void PolygonCreator::checkBannedAreas(CGAL::Polygon_with_holes_2<K> contour){

    CGAL::Polygon_2<K> inner_boundary = contour.outer_boundary();
    CGAL::Polygon_2<K2> inner_boundary_K2 = convertPoly2K2(inner_boundary);

    for(int hole = 0; hole < contour.number_of_holes(); hole++){
        CGAL::Polygon_2<K> hole_polygon = contour.holes().at(hole);
        CGAL::Polygon_2<K2> hole_polygon_K2 = convertPoly2K2(hole_polygon);

        Pwh_list_2 difference_polygon;
        CGAL::symmetric_difference(inner_boundary_K2, hole_polygon_K2, std::back_inserter(difference_polygon));

        if(difference_polygon.size() == 1){
            cout << "Hole n° " << hole << ": ok" << endl;
        }
        else{
            cout << "Hole n° " << hole << ": NOT ok" << endl;
        }

    }

}

/*************************************/
CGAL::Polygon_with_holes_2<K> PolygonCreator::createPolygonFromPath(vector<K::Point_2> path){

    plotPath(path, initialPathImage, 0, 0, 0, false, "Path", false, true, 100, 500); 
    cv::imshow("Path", initialPathImage);
    cv::waitKey(0);

    vector<vector<K::Segment_2>> segments_no_int;
    vector<vector<K::Segment_2>> segments_int;
    vector<pair<int, int>>  count_int = pathSegmentation(path, &segments_no_int, &segments_int);

    vector<CGAL::Polygon_with_holes_2<K>> poly_intersection = defineIntersectionPolygons(segments_int);
    vector<CGAL::Polygon_with_holes_2<K>> poly_no_intersection = defineIndipendentPolygons(segments_no_int);

    plotPath(path, polygonsImage, 0, 0, 0, false, "Polygons", false, true, 100, 500); 
    CGAL::Polygon_with_holes_2<K2> contour = mergePolygons(poly_no_intersection, poly_intersection, count_int, polygonsImage, "Polygons");

    printPolygonsWithHoles(contour, cph, contourImage, false, 0, 0, 255, "Contour", false, true);
    deleteHoles(&contour, areaThreshold, contourImage, "Contour");
    cv::imshow("Contour", contourImage);
    cv::waitKey(0);
    
    printPolygonsWithHoles(contour, cph, contourWithoutHolesImage, false, 0, 0, 255, "Contour without holes", false, true);
    cv::imshow("Contour without holes", contourWithoutHolesImage);
    cv::waitKey(0);

    CGAL::Polygon_with_holes_2<K> contour_K = convertPolyWithHoles2K(contour);

    return contour_K;

}

/*************************************/
CGAL::Polygon_with_holes_2<K> PolygonCreator::createPolygon(vector<K::Point_2> perimeter, vector<vector<K::Point_2>> banned_areas){

    CGAL::Polygon_2<K> perimeter_polygon = polygonFromClosedPath(perimeter);

    CGAL::Polygon_with_holes_2<K> perimeter_polygon_with_holes(perimeter_polygon);
    printPolygonsWithHoles(perimeter_polygon_with_holes, cph, contourPerimeterImage, false, 0, 0, 0, "Perimeter contour", true, false);

    CGAL::Polygon_2<K> contour_perimeter = perimeterContour(perimeter_polygon);

    CGAL::Polygon_with_holes_2<K> contour_perimeter_with_holes;
    if(apply_contouring == true){
        clonePolygon(&contour_perimeter_with_holes, contour_perimeter);
    }
    else{
        clonePolygon(&contour_perimeter_with_holes, perimeter_polygon);
    }

    printPolygonsWithHoles(contour_perimeter_with_holes, cph, contourPerimeterImage, true, 0, 0, 125, "Perimeter contour", false, false);

    for(int i = 0; i < banned_areas.size(); i++){
        vector<K::Point_2> banned_area = banned_areas.at(i);

        CGAL::Polygon_2<K> banned_area_polygon = polygonFromClosedPath(banned_area);
        CGAL::Polygon_with_holes_2<K> banned_area_with_holes(banned_area_polygon);
        printPolygonsWithHoles(banned_area_with_holes, cph, contourPerimeterImage, true, 255, 255, 255, "Perimeter contour", false, false);
        printPolygonsWithHoles(banned_area_with_holes, cph, contourPerimeterImage, false, 255, 0, 0, "Perimeter contour", false, false);

        contour_perimeter_with_holes.add_hole(banned_area_polygon);
    }

    cv::imshow("Perimeter contour", contourPerimeterImage);
    cv::waitKey(0);

    cout << "ciao" << endl;
    checkBannedAreas(contour_perimeter_with_holes);

    return contour_perimeter_with_holes;

}
