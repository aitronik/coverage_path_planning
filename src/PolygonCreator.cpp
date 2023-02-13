#include "PolygonCreator.h"

PolygonCreator::PolygonCreator(){

    m_scaleFootprint = 0.001;
    m_contourOffset = 0;
    m_perimeterOffset = 0;
    m_areaThreshold = 0;
    m_applyContouring = true;

}

/*************************************/

PolygonCreator::~PolygonCreator(){

}

/*************************************/

bool PolygonCreator::init(float fakeFootprint, float contourOffset, float holeThreshold, float perimeterOffset, float applyContouring, const vector<int>& cleaningLvl){

    m_scaleFootprint = fakeFootprint;
    m_contourOffset = contourOffset;
    m_areaThreshold = holeThreshold;
    m_perimeterOffset = perimeterOffset;
    m_applyContouring = applyContouring;
    m_cleaningLvl = cleaningLvl;
    
    return true;

}

/*************************************/
vector<K::Vector_2> PolygonCreator::pointsToVectors(const vector<K::Point_2>& points){

    vector<K::Vector_2> pathVectors;
    pathVectors.resize(points.size()-1);

    for(int i = 1; i < points.size(); i++){
        pathVectors[i-1] = K::Vector_2(points.at(i-1), points.at(i));
    }

    return pathVectors;

}

/*************************************/
vector<K::Segment_2> PolygonCreator::pointsToSegments(const vector<K::Point_2>& points){

    vector<K::Segment_2> pathSegments;
    pathSegments.resize(points.size()-1);

    for(int i = 1; i < points.size(); i++){
        pathSegments[i-1] = K::Segment_2(points.at(i-1), points.at(i));
    }

    return pathSegments;
}

/*************************************/
vector<K::Point_2> PolygonCreator::segmentsToPoints(const vector<K::Segment_2>& segments){
    
    vector<K::Point_2> segmentsPoints;
    segmentsPoints.resize(segments.size() + 1);

    segmentsPoints[0] = K::Point_2(segments.at(0).point(0).x(), segments.at(0).point(0).y());

    for(int i = 0; i < segments.size(); i++){
        segmentsPoints[i+1] = K::Point_2(segments.at(i).point(1).x(), segments.at(i).point(1).y());;
    }

    return segmentsPoints;

}

/*************************************/
void PolygonCreator::unitVector(K::Vector_2* vector){

    float norm = sqrt(pow(vector->x(), 2) + pow(vector->y(), 2));
    K::Vector_2 vNorm(vector->x()/norm, vector->y()/norm);
    *vector = vNorm;

}

/*************************************/
K::Vector_2 PolygonCreator::perpendicularVector(K::Vector_2 vector, bool isComputedCounterClockwise){

    K::Vector_2 vectorPerpendicular;
    if(isComputedCounterClockwise == true){
        vectorPerpendicular = vector.perpendicular(CGAL::Sign {CGAL::COUNTERCLOCKWISE});
    }
    else{
        vectorPerpendicular = vector.perpendicular(CGAL::Sign {CGAL::CLOCKWISE});
    }

    return vectorPerpendicular;

}

/*************************************/
vector<K::Point_2> PolygonCreator::segmentShift(const K::Segment_2 segment, const bool isComputedCounterClockwise, const float scaleFootprint){

    vector<K::Point_2> segmentPoints;
    segmentPoints.resize(2);
    segmentPoints[0] = segment.point(0);
    segmentPoints[1] = segment.point(1);

    K::Vector_2 initialVector(segmentPoints.at(0), segmentPoints.at(1));
    K::Vector_2 perpVector;

    vector<K::Point_2> pathShifted;
    pathShifted.resize(2);

    perpVector = perpendicularVector(initialVector, isComputedCounterClockwise);
    unitVector(&perpVector);

    K::Point_2 firstPoint = segmentPoints.at(0) + scaleFootprint*perpVector;
    K::Point_2 secondPoint = firstPoint + initialVector;

    pathShifted[0] = firstPoint;
    pathShifted[1] = secondPoint;

    return pathShifted;
}

/*************************************/
vector<K::Point_2> PolygonCreator::offsetPath(const auto& path, const float scaleFootprint, const bool isComputedCounterClockwise){

    vector<K::Vector_2> pathVectors = pointsToVectors(path); 
    vector<K::Point_2> pathOffset; 

    K::Vector_2 firstVectorPerpendicular;
    firstVectorPerpendicular = perpendicularVector(pathVectors.at(0), isComputedCounterClockwise);
    unitVector(&firstVectorPerpendicular);

    pathOffset.push_back(path.at(0) + scaleFootprint*firstVectorPerpendicular);

    for(int i = 1; i < path.size()-1; i++){

        K::Vector_2 vectorPerpendicular1 = perpendicularVector(pathVectors.at(i-1), isComputedCounterClockwise);
        unitVector(&vectorPerpendicular1);
        K::Point_2 point1 = path.at(i) + scaleFootprint*vectorPerpendicular1;

        K::Vector_2 vectorPerpendicular2 = perpendicularVector(pathVectors.at(i), isComputedCounterClockwise);
        unitVector(&vectorPerpendicular2);
        K::Point_2 point2 = path.at(i) + scaleFootprint*vectorPerpendicular2;

        K::Point_2 meanPoint((point1.x() + point2.x())/2, (point1.y() + point2.y())/2);
        pathOffset.push_back(meanPoint);

        if(CGAL::angle(path.at(i-1), path.at(i), path.at(i+1)) == 1){
            K::Vector_2 vector3(path.at(i), meanPoint);
            K::Vector_2 vectorPerpendicular3 = perpendicularVector(vector3, !isComputedCounterClockwise);
            unitVector(&vectorPerpendicular3);
            
            pathOffset.push_back(meanPoint + 0.1*scaleFootprint*vectorPerpendicular3);
        }

        
    }


    K::Vector_2 endVectorPerpendicular;
    endVectorPerpendicular = perpendicularVector(pathVectors.at(pathVectors.size()-1), isComputedCounterClockwise);
    unitVector(&endVectorPerpendicular);

    pathOffset.push_back(path.at(path.size()-1) + scaleFootprint*endVectorPerpendicular);

    return pathOffset;

}

/*************************************/
CGAL::Polygon_2<K> PolygonCreator::definePolygon(auto& pathDown, const auto& pathUp){

    CGAL::Polygon_2<K> polygon;
    polygon.resize(pathDown.size() + pathUp.size());

    reverse(pathDown.begin(), pathDown.end());

    for(int i = 0; i < pathUp.size(); i++){
        polygon[i] = K::Point_2(CGAL::to_double(pathUp.at(i).x()), CGAL::to_double(pathUp.at(i).y()));
    }

    for(int i = 0; i < pathDown.size(); i++){
        polygon[i + pathUp.size()] = K::Point_2(CGAL::to_double(pathDown.at(i).x()), CGAL::to_double(pathDown.at(i).y()));
    }

    return polygon;

}

/*************************************/
CGAL::Polygon_2<K2> PolygonCreator::convertPoly2K2(const CGAL::Polygon_2<K> polygon){

    CGAL::Polygon_2<K2> polygonK2;
    polygonK2.resize(polygon.size());

    for(int i = 0; i < polygon.size(); i++){
        polygonK2[i] = K2::Point_2(CGAL::to_double(polygon.vertex(i).x()), CGAL::to_double(polygon.vertex(i).y()));;
    }

    return polygonK2;

}

/*************************************/
CGAL::Polygon_2<K> PolygonCreator::convertPoly2K(const CGAL::Polygon_2<K2> polygon){

    CGAL::Polygon_2<K> polygonK;
    polygonK.resize(polygon.size());

    for(int i = 0; i < polygon.size(); i++){
        polygonK[i] = K::Point_2(CGAL::to_double(polygon.vertex(i).x()), CGAL::to_double(polygon.vertex(i).y()));;
    }

    return polygonK;

}

/*************************************/
CGAL::Polygon_with_holes_2<K2> PolygonCreator::convertPolyWithHoles2K2(const CGAL::Polygon_with_holes_2<K> polygonWithHoles){

    CGAL::Polygon_2<K2> outerBoundary;
    outerBoundary.resize(polygonWithHoles.outer_boundary().size());

    for(int i = 0; i < polygonWithHoles.outer_boundary().size(); i++){
        outerBoundary[i] = K2::Point_2(CGAL::to_double(polygonWithHoles.outer_boundary().vertex(i).x()), CGAL::to_double(polygonWithHoles.outer_boundary().vertex(i).y()));
    }

    CGAL::Polygon_with_holes_2<K2> polyK2(outerBoundary);
    for(int i = 0; i < polygonWithHoles.number_of_holes(); i++){
        CGAL::Polygon_2<K2> hole = convertPoly2K2(polygonWithHoles.holes().at(i));
        polyK2.add_hole(hole);
    }

    return polyK2;

}

/*************************************/
CGAL::Polygon_with_holes_2<K> PolygonCreator::convertPolyWithHoles2K(const CGAL::Polygon_with_holes_2<K2> polygonWithHoles){

    CGAL::Polygon_2<K> outerBoundary;
    outerBoundary.resize(polygonWithHoles.outer_boundary().size());

    for(int i = 0; i < polygonWithHoles.outer_boundary().size(); i++){
        outerBoundary[i] = K::Point_2(CGAL::to_double(polygonWithHoles.outer_boundary().vertex(i).x()), CGAL::to_double(polygonWithHoles.outer_boundary().vertex(i).y()));;
    }

    CGAL::Polygon_with_holes_2<K> polyK(outerBoundary);
    for(int i = 0; i < polygonWithHoles.number_of_holes(); i++){
        CGAL::Polygon_2<K> hole = convertPoly2K(polygonWithHoles.holes().at(i));
        polyK.add_hole(hole);
    }

    return polyK;

}

/*************************************/
vector<pair<int, int>> PolygonCreator::pathSegmentation(const vector<K::Point_2>& path, 
                                                        vector<vector<K::Segment_2>>* segmentsNoInt, 
                                                        vector<vector<K::Segment_2>>* segmentsInt){

    /*
    The function iterates over all segments of the path and merges together into a vector of segments (segments_no_int) the subpaths where there are no
    intersections. The intersecting segments, on the other hand, are saved in another vector of segments (segments_int).
    */
    int jumpIndex = 0;
    int i = jumpIndex;
    

    vector<pair<int, int>> countInt;
    vector<K::Segment_2> pathSegments = pointsToSegments(path);
    /*
    While loop which exits when the iteration index i (which is initialised to zero in the first step) is great than the length of the 
    vector of the path segments. In the while loop there is a for loop that iterates over the segments of the path. Given the i-th segment, a
    for loop is done on all subsequent segments and the segments intersecting with the i-th segment are identified. Pairs are then created 
    and saved in the segment vector segments_int. Segments that do not have intersections, on the other hand, are saved
    in segments_no_int. Whenever at least one intersection is found, the intersection_flag flag is set to true and the first for
    is interrupted with a break. The jump_index is updated to i+1 and the index i re-initialised with the new value of jump_index.
    In addition, the indexes of the intersections are saved in the count_int vector.
    */
    while(i < pathSegments.size()){
        vector<K::Segment_2> segmentTmp;
        for(i = jumpIndex; i < pathSegments.size(); i++){
            vector<K::Segment_2> intersectTmp;
            bool intersectionFlag = false;

            for(int j = i + 1; j < pathSegments.size(); j++){
                auto result = CGAL::intersection(pathSegments.at(i), pathSegments.at(j));
                K::Point_2 intersectionPoint;
                if(result){
                    intersectionPoint = boost::get<K::Point_2 >(*result);
                    if(intersectionPoint.x() != pathSegments.at(j).point(0).x() && intersectionPoint.y() != pathSegments.at(j).point(0).y()){
                        countInt.push_back({i, j});
                        intersectionFlag = true;
                
                        intersectTmp.push_back(pathSegments.at(i));
                        intersectTmp.push_back(pathSegments.at(j));
                    }
                }
            }

            if(intersectionFlag == true){
                jumpIndex = i + 1;
                segmentsInt->push_back(intersectTmp);
                break;
            }

            segmentTmp.push_back(pathSegments.at(i));
        }

        segmentsNoInt->push_back(segmentTmp);
    }

    return countInt;

}

/*************************************/
vector<CGAL::Polygon_with_holes_2<K>> PolygonCreator::defineIntersectionPolygons(const vector<vector<K::Segment_2>>& segmentsInt){

    vector<CGAL::Polygon_with_holes_2<K>> polygonFinalInt;
    /*The code iterates along the first level of the input vector of segment vectors. Each step, therefore, considers a vector of segments.*/
    for(int i = 0; i < segmentsInt.size(); i++){

        vector<CGAL::Polygon_2<K>> polyListInt;

        /*The second for loop runs through all the segments of the vector of segments selected in the previous for loop. Dummy polygons are generated
        with an infinitesimal thickness (scale_footprint, which is set to 0.001 metres by default) which are saved in a vector (poly_list_int). 
        Why generate polygons with infinitesimal thickness?
        CGAL works well with polygons and many functions (including contouring functions) can only be used on polygons. Creating polygons with infinitesimal
        thickness allows these functions to be exploited. The overlap error with the original path is negligible.*/
        for(int j = 0; j < segmentsInt.at(i).size(); j++){
            K::Segment_2 segmentTmp = segmentsInt.at(i).at(j);
            vector<K::Point_2> pathTmp;
            pathTmp.resize(2);
            pathTmp[0] = segmentTmp.point(0);
            pathTmp[1] = segmentTmp.point(1);

            if(pathTmp.size() != 0){
                vector<K::Point_2> pathUpInt = segmentShift(segmentTmp, false, m_scaleFootprint);
                vector<K::Point_2> pathDownInt = segmentShift(segmentTmp, true, m_scaleFootprint);
                
                CGAL::Polygon_2<K> polyInt = definePolygon(pathDownInt, pathUpInt);

                if(polyInt.is_counterclockwise_oriented() == false){
                    polyInt.reverse_orientation();
                }

                polyListInt.push_back(polyInt);
            }
        }

        if(polyListInt.size() != 0){

            CGAL::Polygon_2<K2> polygonK2Initial = convertPoly2K2(polyListInt.at(0));
            CGAL::Polygon_with_holes_2<K2> polygonWithHolesK2(polygonK2Initial);

            for(int j = 1; j < polyListInt.size(); j++){
                CGAL::Polygon_2<K2> polygonK2 = convertPoly2K2(polyListInt.at(j));
                CGAL::join(polygonWithHolesK2, polygonK2, polygonWithHolesK2);
            }

            PolygonWithHolesPtrVector contour_ptr = CGAL::create_exterior_skeleton_and_offset_polygons_with_holes_2(m_contourOffset,polygonWithHolesK2.outer_boundary());

            vector<CGAL::Polygon_with_holes_2<K>> contour;
            for(PolygonWithHolesPtrVector::const_iterator v = contour_ptr.begin(); v != contour_ptr.end(); ++v){
                contour.push_back(**v);
            }

            polygonFinalInt.push_back(contour.at(0));
        }
    }

    return polygonFinalInt;

}

/*************************************/
vector<CGAL::Polygon_with_holes_2<K>> PolygonCreator::defineIndipendentPolygons(const vector<vector<K::Segment_2>>& segmentsNoInt){

    vector<CGAL::Polygon_with_holes_2<K>> polygonFinalNoInt;
    /*The code iterates along the first level of the input vector of segment vectors. Each step, therefore, considers a vector of segments.*/
    for(int k = 0; k < segmentsNoInt.size(); k++){

        vector<K::Segment_2> segments = segmentsNoInt.at(k);
        
        if(segments.size() != 0){

            vector<K::Point_2> subpath = segmentsToPoints(segments);

        
            /*The second for loop runs through all the segments of the vector of segments selected in the previous for loop. Dummy polygons are generated
            with an infinitesimal thickness (scale_footprint, which is set to 0.001 metres by default) which are saved in a vector (poly_list_int). 
            Why generate polygons with infinitesimal thickness?
            CGAL works well with polygons and many functions (including contouring functions) can only be used on polygons. Creating polygons with infinitesimal
            thickness allows these functions to be exploited. The overlap error with the original path is negligible.*/
            if(subpath.size() != 0){
                vector<K::Point_2> pathUp = offsetPath(subpath, m_scaleFootprint, false);
                vector<K::Point_2> pathDown = offsetPath(subpath, m_scaleFootprint, true);

                CGAL::Polygon_2<K> polygon = definePolygon(pathDown, pathUp);

                CGAL::Polygon_with_holes_2<K> polyWithHoles(polygon);

                PolygonWithHolesPtrVector contourPtr = CGAL::create_exterior_skeleton_and_offset_polygons_with_holes_2(m_contourOffset,polygon);

                if(contourPtr.size() != 0){
                    vector<CGAL::Polygon_with_holes_2<K>> contour;
                    for(PolygonWithHolesPtrVector::const_iterator v = contourPtr.begin(); v != contourPtr.end(); ++v){
                        contour.push_back(**v);
                    }

                    polygonFinalNoInt.push_back(contour.at(0));

                }
            }

        }
    
    }

    return polygonFinalNoInt;

}

/*************************************/
CGAL::Polygon_with_holes_2<K2> PolygonCreator::mergePolygons(const vector<CGAL::Polygon_with_holes_2<K>>& polygonFinalNoInt, 
                                                            const vector<CGAL::Polygon_with_holes_2<K>>& polygonFinalInt,
                                                            const vector<pair<int, int>>& countInt,
                                                            const std::string name){
    
    int indexInt = 0;
    int indexSeg = 1;

    CGAL::Polygon_with_holes_2<K2> initialPolygon = convertPolyWithHoles2K2(polygonFinalNoInt.at(0));
    CGAL::Polygon_with_holes_2<K2> contourFinal = initialPolygon;

    m_cph.printPolygonsWithHolesK2(initialPolygon, m_cph.m_polygonsImage, false, 0, 0, 0, name, false, true, false, m_cleaningLvl);

    int index = 0;
    int i = index;
    while(i < countInt.size()-1){

        for(i = index; i < countInt.size()-1; i++){
            if((countInt.at(i+1).first - countInt.at(i).first == 1 || countInt.at(i+1).first - countInt.at(i).first == 0)){
                CGAL::Polygon_with_holes_2<K> polygonIntersection(polygonFinalInt.at(indexInt));
                if(countInt.at(i+1).first - countInt.at(i).first == 1){
                    indexInt = indexInt + 1;
                }
                
                CGAL::Polygon_with_holes_2<K2> polygonIntersectionK2 = convertPolyWithHoles2K2(polygonIntersection);
                m_cph.printPolygonsWithHolesK(polygonIntersection, m_cph.m_polygonsImage, false, 0, 0, 0, name, false, true, false, m_cleaningLvl);
                CGAL::join(contourFinal, polygonIntersectionK2, contourFinal);
            }
            else{
                
                CGAL::Polygon_with_holes_2<K> polygonNoIntersection(polygonFinalNoInt.at(indexSeg));
                indexSeg = indexSeg + 1;
                CGAL::Polygon_with_holes_2<K2> polygonNoIntersectionK2 = convertPolyWithHoles2K2(polygonNoIntersection);
                m_cph.printPolygonsWithHolesK(polygonNoIntersection, m_cph.m_polygonsImage, false, 0, 0, 0, name, false, true, false, m_cleaningLvl);
                CGAL::join(contourFinal, polygonNoIntersectionK2, contourFinal);


                CGAL::Polygon_with_holes_2<K> polygonIntersection(polygonFinalInt.at(indexInt));
                indexInt = indexInt + 1;
                CGAL::Polygon_with_holes_2<K2> polygonIntersectionK2 = convertPolyWithHoles2K2(polygonIntersection);
                m_cph.printPolygonsWithHolesK(polygonIntersection, m_cph.m_polygonsImage, false, 0, 0, 0, name, false, true, false, m_cleaningLvl);
                CGAL::join(contourFinal, polygonIntersectionK2, contourFinal);

                index = i+1;

                break;
            }
        }
    }

    CGAL::Polygon_with_holes_2<K> polygonNoIntersection(polygonFinalNoInt.at(polygonFinalNoInt.size()-1));
    CGAL::Polygon_with_holes_2<K2> polygonNoIntersectionK2 = convertPolyWithHoles2K2(polygonNoIntersection);

    CGAL::join(contourFinal, polygonNoIntersectionK2, contourFinal);

    m_cph.printPolygonsWithHolesK(polygonFinalNoInt.at(polygonFinalNoInt.size()-1), m_cph.m_polygonsImage, false, 0, 0, 0, name, false, true, true, m_cleaningLvl); 

    return(contourFinal);

}

/*************************************/
void PolygonCreator::deleteHoles(CGAL::Polygon_with_holes_2<K2>* contour, 
                                const float areaThreshold,
                                const std::string name){

    for(auto hole = contour->holes_begin(); hole != contour->holes_end(); hole++){

        double holeArea = abs(CGAL::to_double(hole->area()));
        if(holeArea != 0){
            if(holeArea < areaThreshold){
                CGAL::Polygon_with_holes_2<K2> holeToDelete(*hole);
                m_cph.printPolygonsWithHolesK2(holeToDelete, m_cph.m_contourImage, false, 0, 0, 0, name, false, true, true, m_cleaningLvl);
                contour->erase_hole(hole);
            }
        }
    }

}

/*************************************/
CGAL::Polygon_with_holes_2<K> PolygonCreator::selectMajorPolygon(const vector<CGAL::Polygon_with_holes_2<K>>& contours){

    CGAL::Polygon_with_holes_2<K> perimeterContour;
    if(contours.size() == 0){
        perimeterContour = contours.at(0);
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

        perimeterContour = contours.at(largest_polygon);
    }

    return perimeterContour;

}

/*************************************/
CGAL::Polygon_2<K> PolygonCreator::polygonFromClosedPath(const vector<K::Point_2>& path){

    CGAL::Polygon_2<K> polygon;
    polygon.resize(path.size());
    for(int i = 0; i < path.size(); i++){
        polygon[i] = path.at(i);
    }

    return polygon;

}

/*************************************/
CGAL::Polygon_2<K> PolygonCreator::perimeterContour(const CGAL::Polygon_2<K> perimeterPolygon){

    CGAL::Polygon_with_holes_2<K> perimeterContour;
    PolygonWithHolesPtrVector contourPtr = CGAL::create_interior_skeleton_and_offset_polygons_with_holes_2(m_perimeterOffset, perimeterPolygon);

    if(contourPtr.size() != 0){
        vector<CGAL::Polygon_with_holes_2<K>> contours;
        for(PolygonWithHolesPtrVector::const_iterator v = contourPtr.begin(); v != contourPtr.end(); ++v){
            contours.push_back(**v);
        }

        perimeterContour = selectMajorPolygon(contours);
    }

    return perimeterContour.outer_boundary();

}

/*************************************/
void PolygonCreator::clonePolygon(CGAL::Polygon_with_holes_2<K>* finalPolyWithHoles, const CGAL::Polygon_2<K> polyToClone){

    for(int i = 0; i < polyToClone.size(); i++){
        finalPolyWithHoles->outer_boundary().push_back(polyToClone.vertex(i));
    }

}

/*************************************/
void PolygonCreator::checkBannedAreas(const CGAL::Polygon_with_holes_2<K> contour){

    CGAL::Polygon_2<K> innerBoundary = contour.outer_boundary();
    CGAL::Polygon_2<K2> innerBoundaryK2 = convertPoly2K2(innerBoundary);

    for(int hole = 0; hole < contour.number_of_holes(); hole++){
        CGAL::Polygon_2<K> holePolygon = contour.holes().at(hole);
        CGAL::Polygon_2<K2> holePolygonK2 = convertPoly2K2(holePolygon);

        Pwh_list_2 differencePolygon;
        CGAL::symmetric_difference(innerBoundaryK2, holePolygonK2, std::back_inserter(differencePolygon));

        if(differencePolygon.size() == 1){
            cout << "Hole n° " << hole << ": ok" << endl;
        }
        else{
            cout << "Hole n° " << hole << ": NOT ok" << endl;
        }

    }

}

/*************************************/
CGAL::Polygon_with_holes_2<K> PolygonCreator::createPolygonFromPath(const vector<K::Point_2>& path){

    m_cph.plotPath(path, m_cph.m_initialPathImage, false, 0, 0, 0, "Path", false, true, m_cleaningLvl); 
    cv::imshow("Path", m_cph.m_initialPathImage);
    cv::waitKey(0);

    clock_t tic = clock();

    vector<vector<K::Segment_2>> segmentsNoInt;
    vector<vector<K::Segment_2>> segmentsInt;
    vector<pair<int, int>>  count_int = pathSegmentation(path, &segmentsNoInt, &segmentsInt);

    vector<CGAL::Polygon_with_holes_2<K>> polyIntersection = defineIntersectionPolygons(segmentsInt);
    vector<CGAL::Polygon_with_holes_2<K>> polyNoIntersection = defineIndipendentPolygons(segmentsNoInt);
    
    m_cph.plotPath(path, m_cph.m_polygonsImage, false, 0, 0, 0, "Polygons", false, true, m_cleaningLvl); 
    CGAL::Polygon_with_holes_2<K2> contour = mergePolygons(polyNoIntersection, polyIntersection, count_int, "Polygons");

    

    clock_t toc = clock();
    double time = (double)(toc-tic);
    cout << "It took "<< 1000*(time/CLOCKS_PER_SEC) << "millisecond(ms)."<< endl;


    m_cph.printPolygonsWithHolesK2(contour, m_cph.m_contourImage, false, 0, 0, 0, "Contour", false, true, true, m_cleaningLvl);
    deleteHoles(&contour, m_areaThreshold, "Contour");

    m_cph.printPolygonsWithHolesK2(contour, m_cph.m_contourWithoutHolesImage, false, 0, 0, 0, "Contour without holes", false, true, true, m_cleaningLvl);

    CGAL::Polygon_with_holes_2<K> contourK = convertPolyWithHoles2K(contour);

    return contourK;

}

/*************************************/
CGAL::Polygon_with_holes_2<K> PolygonCreator::createPolygon(vector<K::Point_2> perimeter, vector<vector<K::Point_2>> banned_areas){

    CGAL::Polygon_2<K> perimeter_polygon = polygonFromClosedPath(perimeter);

    CGAL::Polygon_with_holes_2<K> perimeter_polygon_with_holes(perimeter_polygon);
    m_cph.printPolygonsWithHolesK(perimeter_polygon_with_holes, m_cph.m_contourPerimeterImage, false, 0, 0, 0, "Perimeter contour", true, false, true, m_cleaningLvl);

    CGAL::Polygon_2<K> contour_perimeter = perimeterContour(perimeter_polygon);

    CGAL::Polygon_with_holes_2<K> contour_perimeter_with_holes;
    if(m_applyContouring == true){
        clonePolygon(&contour_perimeter_with_holes, contour_perimeter);
    }
    else{
        clonePolygon(&contour_perimeter_with_holes, perimeter_polygon);
    }

    m_cph.printPolygonsWithHolesK(contour_perimeter_with_holes, m_cph.m_contourPerimeterImage, true, 0, 0, 255, "Perimeter contour", false, false, true, m_cleaningLvl);

    for(int i = 0; i < banned_areas.size(); i++){
        vector<K::Point_2> banned_area = banned_areas.at(i);

        CGAL::Polygon_2<K> banned_area_polygon = polygonFromClosedPath(banned_area);
        CGAL::Polygon_with_holes_2<K> banned_area_with_holes(banned_area_polygon);
        m_cph.printPolygonsWithHolesK(banned_area_with_holes, m_cph.m_contourPerimeterImage, true, 255, 255, 255, "Perimeter contour", false, false, true, m_cleaningLvl);
        m_cph.printPolygonsWithHolesK(banned_area_with_holes, m_cph.m_contourPerimeterImage, false, 0, 0, 0, "Perimeter contour", false, false, true, m_cleaningLvl);

        contour_perimeter_with_holes.add_hole(banned_area_polygon);
    }

    checkBannedAreas(contour_perimeter_with_holes);

    return contour_perimeter_with_holes;

}
