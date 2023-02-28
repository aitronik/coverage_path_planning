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
void PolygonCreator::pointsToSegments(const vector<K::Point_2>& points, vector<K::Segment_2>* pathSegments, vector<int>* cleaningLvlSegments){

    pathSegments->resize(points.size()-1);
    cleaningLvlSegments->resize(pathSegments->size());

    for(int i = 1; i < points.size(); i++){
        pathSegments->at(i-1) = K::Segment_2(points.at(i-1), points.at(i));
        cleaningLvlSegments->at(i-1) = std::max(m_cleaningLvl.at(i-1), m_cleaningLvl.at(i));
    }

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
vector<K::Point_2> PolygonCreator::polygonToPoints(const CGAL::Polygon_2<K> polygon){
    
    vector<K::Point_2> points;
    points.resize(polygon.size());
    for(int i = 0; i < polygon.size(); i++){
        points[i] = polygon.vertex(i);
    }

    return points;

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
CGAL::Polygon_with_holes_2<K> PolygonCreator::deleteDegeneratedSegments(CGAL::Polygon_with_holes_2<K> polygonWithHoles){
    
    vector<pair<float, float>> vertices;
    vertices.resize(CGAL::to_double(polygonWithHoles.outer_boundary().size()));
    for(int i = 0; i < polygonWithHoles.outer_boundary().size(); i++){
        vertices[i] = {CGAL::to_double(polygonWithHoles.outer_boundary().vertex(i).x()), CGAL::to_double(polygonWithHoles.outer_boundary().vertex(i).y())};
    }

    auto itx = std::unique(vertices.begin(), vertices.end());
    vertices.erase(itx, vertices.end());

    if(vertices.size() != 0){
        if(vertices.at(0) == vertices.at(vertices.size()-1)){
            vertices.erase(vertices.end());
        }
    }

    CGAL::Polygon_2<K> outerBoundary;
    outerBoundary.resize(vertices.size());
    for(int i = 0; i < vertices.size(); i++){
        outerBoundary[i] = K::Point_2(vertices.at(i).first, vertices.at(i).second);
    }

    CGAL::Polygon_with_holes_2<K> newPolygon(outerBoundary);
    for(int i = 0; i < polygonWithHoles.number_of_holes(); i++){
        newPolygon.add_hole(polygonWithHoles.holes().at(i));
    }

    return(newPolygon);
}

/*************************************/
vector<K2::Segment_2> PolygonCreator::convertSegmentsToK2(vector<K::Segment_2>& segments){
    
    vector<K2::Segment_2> outputSegments;
    outputSegments.resize(segments.size());
    for(int i = 0; i < segments.size(); i++){
        K2::Point_2 p1(segments.at(i).point(0).x(), segments.at(i).point(0).y());
        K2::Point_2 p2(segments.at(i).point(1).x(), segments.at(i).point(1).y());

        K2::Segment_2 s(p1, p2);
        outputSegments[i] = s;
    }

    return outputSegments;
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

vector<pair<CGAL::Polygon_with_holes_2<K>, int>> PolygonCreator::convertPolygonIntPairVector2K(vector<pair<CGAL::Polygon_with_holes_2<K2>, int>> polyIntVector){
    
    vector<pair<CGAL::Polygon_with_holes_2<K>, int>> outputVector;
    for(int i = 0; i < polyIntVector.size(); i++){
        outputVector.push_back({convertPolyWithHoles2K(polyIntVector.at(i).first), polyIntVector.at(i).second});
    }

    return(outputVector);
}

/*************************************/
CGAL::Polygon_2<K> PolygonCreator::simplifyPolygon(const CGAL::Polygon_2<K> polygon){

    CGAL::Bbox_2 bbox = polygon.bbox();
    
    double fxmin = bbox.xmin();
    double fxmax = bbox.xmax();
    double fymin = bbox.ymin();
    double fymax = bbox.ymax();

    CGAL::Polygon_2<K> outputHole;

    outputHole.push_back(K::Point_2(fxmin,fymin));
    outputHole.push_back(K::Point_2(fxmax,fymin));
    outputHole.push_back(K::Point_2(fxmax,fymax));
    outputHole.push_back(K::Point_2(fxmin,fymax));

    return outputHole;

}

/*************************************/
CGAL::Polygon_2<K> PolygonCreator::simplifyPolygon2(vector<K::Point_2>& pathUp, vector<K::Point_2>& pathDown){

    
    reverse(pathDown.begin(), pathDown.end());

    CGAL::Polygon_2<K> polyTmpZero;
    polyTmpZero.resize(4);
    polyTmpZero[0] = pathUp.at(0);
    polyTmpZero[1] = pathUp.at(1);
    polyTmpZero[2] = pathDown.at(1);
    polyTmpZero[3] = pathDown.at(0);

    CGAL::Polygon_with_holes_2<K2> subPolygons(convertPoly2K2(polyTmpZero));

    for(int i = 1; i < pathUp.size()-1; i++){
        CGAL::Polygon_2<K> polyTmp;
        polyTmp.resize(4);
        polyTmp[0] = pathUp.at(i);
        polyTmp[1] = pathUp.at(i+1);
        polyTmp[2] = pathDown.at(i+1);
        polyTmp[3] = pathDown.at(i);

        CGAL::join(subPolygons, convertPoly2K2(polyTmp), subPolygons);
    }


    return convertPoly2K(subPolygons.outer_boundary());

}

/*************************************/
void PolygonCreator::pathSegmentation(const vector<K::Point_2>& path, 
                                                        vector<pair<vector<K::Segment_2>, vector<int>>>* segmentsNoInt, 
                                                        vector<pair<vector<K::Segment_2>, int>>* segmentsInt){

    /*
    The function iterates over all segments of the path and merges together into a vector of segments (segments_no_int) the subpaths where there are no
    intersections. The intersecting segments, on the other hand, are saved in another vector of segments (segments_int).
    */
    int jumpIndex = 0;
    int i = jumpIndex;

    vector<pair<int, int>> countInt;
    vector<K::Segment_2> pathSegments;
    vector<int> cleaningLvlSegments;

    pointsToSegments(path, &pathSegments, &cleaningLvlSegments);

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
        vector<int> cleanLvlTmp;
        for(i = jumpIndex; i < pathSegments.size(); i++){
            vector<K::Segment_2> intersectTmp;
            vector<int> cleaningLvlSegmentsInt;
            bool intersectionFlag = false;

            for(int j = i + 1; j < pathSegments.size(); j++){
                if(pathSegments.at(i).point(0).operator!=(pathSegments.at(i).point(1)) && pathSegments.at(j).point(0).operator!=(pathSegments.at(j).point(1))){
                    if(pathSegments.at(i).point(0).operator!=(pathSegments.at(j).point(1)) && pathSegments.at(i).point(1).operator!=(pathSegments.at(i).point(0))){
                        auto result = CGAL::intersection(pathSegments.at(i), pathSegments.at(j));
                        if(result){
                            K::Point_2* intersectionPoint = boost::get<K::Point_2 >(&*result);
                            if(intersectionPoint->x() != pathSegments.at(j).point(0).x() && intersectionPoint->y() != pathSegments.at(j).point(0).y()){
                                countInt.push_back({i, j});
                                intersectionFlag = true;
                        
                                intersectTmp.push_back(pathSegments.at(i));
                                intersectTmp.push_back(pathSegments.at(j));

                                cleaningLvlSegmentsInt.push_back(std::max(cleaningLvlSegments[i], cleaningLvlSegments[j]));
                            }
                        }
                    }
                }
                
            }

            if(intersectionFlag == true){
                jumpIndex = i + 1;
                segmentsInt->push_back({intersectTmp, *std::max_element(cleaningLvlSegmentsInt.begin(), cleaningLvlSegmentsInt.end())});
                break;
            }

            segmentTmp.push_back(pathSegments.at(i));
            cleanLvlTmp.push_back(cleaningLvlSegments[i]);
        }

        segmentsNoInt->push_back({segmentTmp, cleanLvlTmp});
    }

}

/*************************************/
vector<pair<CGAL::Polygon_with_holes_2<K>, int>> PolygonCreator::defineIntersectionPolygons(const vector<pair<vector<K::Segment_2>, int>>& segmentsInt){

    vector<pair<CGAL::Polygon_with_holes_2<K>, int>> polyListWithCleaningLvl;
    /*The code iterates along the first level of the input vector of segment vectors. Each step, therefore, considers a vector of segments.*/
    for(int i = 0; i < segmentsInt.size(); i++){

        vector<CGAL::Polygon_2<K>> polyListInt;
        /*The second for loop runs through all the segments of the vector of segments selected in the previous for loop. Dummy polygons are generated
        with an infinitesimal thickness (scale_footprint, which is set to 0.001 metres by default) which are saved in a vector (poly_list_int). 
        Why generate polygons with infinitesimal thickness?
        CGAL works well with polygons and many functions (including contouring functions) can only be used on polygons. Creating polygons with infinitesimal
        thickness allows these functions to be exploited. The overlap error with the original path is negligible.*/
        for(int j = 0; j < segmentsInt.at(i).first.size(); j++){
            K::Segment_2 segmentTmp = segmentsInt.at(i).first.at(j);
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

            polyListWithCleaningLvl.push_back({contour.at(0), segmentsInt.at(i).second});
        }
    }

    return polyListWithCleaningLvl;

}

/*************************************/
vector<pair<vector<K::Segment_2>, int>> PolygonCreator::checkCleaningLvlContinuity(const pair<vector<K::Segment_2>, vector<int>>& subpath){

    int jumpIndex = 0;
    int i = jumpIndex;

    vector<pair<vector<K::Segment_2>, int>> decomposedSegments;
    while(i < subpath.first.size()){

        jump:
            vector<K::Segment_2> segmentsSubpath;
            for(i = jumpIndex; i < subpath.first.size(); i++){
                
                segmentsSubpath.push_back(subpath.first.at(i));
                for(int j = i + 1; j < subpath.first.size(); j++){

                    if(subpath.second.at(i) != subpath.second.at(j)){
                        segmentsSubpath.push_back(subpath.first.at(j));
                        decomposedSegments.push_back({segmentsSubpath, subpath.second.at(jumpIndex)});
                        jumpIndex = j+1;
                        goto jump;
                    }
                    else{
                        segmentsSubpath.push_back(subpath.first.at(j));
                    }
                }

                decomposedSegments.push_back({segmentsSubpath, subpath.second.at(jumpIndex)});
                goto exit;
            }
            
    }

    exit:
        return decomposedSegments;

}

/*************************************/
vector<vector<pair<vector<K::Segment_2>, int>>> PolygonCreator::cleanLvlDecomposition(const vector<pair<vector<K::Segment_2>, vector<int>>>& segmentsNoInt){

    vector<vector<pair<vector<K::Segment_2>, int>>> segmentsListWithCleaningLvl;
    for(int i = 0; i < segmentsNoInt.size(); i++){
        vector<pair<vector<K::Segment_2>, int>> segmentsWithCleaningLvlTmp = checkCleaningLvlContinuity(segmentsNoInt.at(i));
        segmentsListWithCleaningLvl.push_back(segmentsWithCleaningLvlTmp);
    }

    return segmentsListWithCleaningLvl;

}

/*************************************/
vector<vector<pair<CGAL::Polygon_with_holes_2<K>, int>>> PolygonCreator::defineIndipendentPolygons(const vector<vector<pair<vector<K::Segment_2>, int>>>& segmentsListWithCleaningLvl){

    vector<vector<pair<CGAL::Polygon_with_holes_2<K>, int>>>  polygonFinalNoInt;
    /*The code iterates along the first level of the input vector of segment vectors. Each step, therefore, considers a vector of segments.*/
    for(int k = 0; k < segmentsListWithCleaningLvl.size(); k++){

        vector<pair<CGAL::Polygon_with_holes_2<K>, int>> polygonFinalNoIntTmp;
        for(int i = 0; i < segmentsListWithCleaningLvl.at(k).size(); i++){
            
            vector<K::Segment_2> segments = segmentsListWithCleaningLvl.at(k).at(i).first;
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
                    CGAL::Polygon_2<K> polygonSimple;
                    PS::Squared_distance_cost cost;

                    if(polygon.is_simple() == false){
                        polygonSimple = simplifyPolygon2(pathUp, pathDown);
                    }
                    else{
                        polygonSimple = polygon;
                    }

                    CGAL::Polygon_with_holes_2<K> polyWithHoles(polygonSimple);

                    PolygonWithHolesPtrVector contourPtr = CGAL::create_exterior_skeleton_and_offset_polygons_with_holes_2(m_contourOffset,polygonSimple);

                    if(contourPtr.size() != 0){
                        vector<CGAL::Polygon_with_holes_2<K>> contour;
                        for(PolygonWithHolesPtrVector::const_iterator v = contourPtr.begin(); v != contourPtr.end(); ++v){
                            contour.push_back(**v);
                        }

                        polygonFinalNoIntTmp.push_back({contour.at(0), segmentsListWithCleaningLvl.at(k).at(i).second});

                    }
                }

            }
        }

        polygonFinalNoInt.push_back(polygonFinalNoIntTmp);
    }

    return polygonFinalNoInt;

}

/*************************************/
void PolygonCreator::plotPolygonsWithCleaningLvl(const vector<vector<pair<CGAL::Polygon_with_holes_2<K>, int>>>& polygonFinalNoInt, 
                                const vector<vector<pair<CGAL::Polygon_with_holes_2<K>, int>>>& polygonFinalInt,
                                const std::string& name){

    for(int i = 0; i < polygonFinalNoInt.size(); i++){
        for(int j = 0; j < polygonFinalNoInt.at(i).size(); j++){
            CGAL::Polygon_with_holes_2<K> polygonNoIntersection(polygonFinalNoInt.at(i).at(j).first);
            vector<int> subPathCleaningLvl(polygonFinalNoInt.at(i).at(j).first.outer_boundary().size(), polygonFinalNoInt.at(i).at(j).second);

            m_cph.printPolygonsWithHolesK(polygonNoIntersection, m_cph.m_polygonsImage, false, 0, 0, 0, 255, name, false, true, false, subPathCleaningLvl);
        }
    }

    for(int i = 0; i < polygonFinalInt.size(); i++){
        for(int j = 0; j < polygonFinalInt.at(i).size(); j++){
            CGAL::Polygon_with_holes_2<K> polygonIntersection(polygonFinalInt.at(i).at(j).first);
            vector<int> subPathCleaningLvl(polygonFinalInt.at(i).at(j).first.outer_boundary().size(), polygonFinalInt.at(i).at(j).second);

            m_cph.printPolygonsWithHolesK(polygonIntersection, m_cph.m_polygonsImage, false, 0, 0, 0, 255, name, false, true, false, subPathCleaningLvl);
        }
    }

    cv::imshow(name, m_cph.m_polygonsImage);
    cv::waitKey(0);

}

/*************************************/
vector<pair<CGAL::Polygon_with_holes_2<K>, int>> PolygonCreator::splitPolygonsList(const vector<vector<pair<CGAL::Polygon_with_holes_2<K>, int>>>& polygonFinalNoInt, 
                                                                             const vector<vector<pair<CGAL::Polygon_with_holes_2<K>, int>>>& polygonFinalInt){

    vector<pair<CGAL::Polygon_with_holes_2<K>, int>> polygonUnion;
    for(int i = 0; i < polygonFinalNoInt.size(); i++){
        for(int j = 0; j < polygonFinalNoInt.at(i).size(); j++){
            polygonUnion.push_back({polygonFinalNoInt.at(i).at(j).first, polygonFinalNoInt.at(i).at(j).second});
        }
    }

    for(int i = 0; i < polygonFinalInt.size(); i++){
        for(int j = 0; j < polygonFinalInt.at(i).size(); j++){
            polygonUnion.push_back({polygonFinalInt.at(i).at(j).first, polygonFinalInt.at(i).at(j).second});
        }
    }

    return polygonUnion;
}

/*************************************/
vector<pair<CGAL::Polygon_with_holes_2<K2>, int>> PolygonCreator::checkSeeds(vector<pair<CGAL::Polygon_with_holes_2<K>, int>>& subpolygons){

    vector<pair<CGAL::Polygon_with_holes_2<K2>, int>> seeds;

    int jumpIndex = 0;
    int polygonCounter = 0;

    jump:   

    CGAL::Polygon_with_holes_2<K2> seed(convertPolyWithHoles2K2(subpolygons.at(jumpIndex).first));

    for(int i = 0; i < subpolygons.size(); i++){
        if(CGAL::do_intersect(seed, convertPolyWithHoles2K2(subpolygons.at(i).first))){
            CGAL::join(seed, convertPolyWithHoles2K2(subpolygons.at(i).first), seed);
            polygonCounter++;
        }
    }

    seeds.push_back({seed, subpolygons.at(jumpIndex).second});

    //reverse(subpolygons.begin(), subpolygons.end());
    
    bool seedMember;
    
    for(int i = 0; i < subpolygons.size(); i++){
        
        for(int j = 0; j < seeds.size(); j++){
            Pwh_list_2 intR;
            CGAL::intersection(seeds.at(j).first.outer_boundary(), convertPoly2K2(subpolygons.at(i).first.outer_boundary()), std::back_inserter(intR));

            if(intR.empty() == false){
                for(auto it = intR.begin(); it != intR.end(); it++){
                    if(it->outer_boundary().area() != subpolygons.at(i).first.outer_boundary().area() && it->outer_boundary().area() != 0){
                        CGAL::join(seeds.at(j).first, convertPolyWithHoles2K2(subpolygons.at(i).first), seeds.at(j).first);
                        polygonCounter++;
                    }
                }
                seedMember = true;
            }
            else{
                seedMember = false;
            }
        }
        
        if(polygonCounter >= subpolygons.size() - 1){
            goto exit;
        }
        else{
            if(seedMember == false){
                jumpIndex = i;
                goto jump;
            }
        } 
    }

    exit:
        CGAL::Polygon_with_holes_2<K2> finalSeed(convertPolyWithHoles2K2(subpolygons.at(subpolygons.size()-1).first));

        for(int i = 0; i < subpolygons.size(); i++){
            if(CGAL::do_intersect(finalSeed, convertPolyWithHoles2K2(subpolygons.at(i).first))){
                CGAL::join(finalSeed, convertPolyWithHoles2K2(subpolygons.at(i).first), finalSeed);
                polygonCounter++;
            }
        }

        seeds.push_back({finalSeed, subpolygons.at(subpolygons.size()-1).second});
        
        return(seeds);

}

/*************************************/
vector<vector<pair<CGAL::Polygon_with_holes_2<K>, int>>> PolygonCreator::checkIntersectingPolygonUnions(const vector<pair<CGAL::Polygon_with_holes_2<K>, int>>& polygonFinalInt){

    vector<vector<pair<CGAL::Polygon_with_holes_2<K>, int>>> polygonsLvls;
    polygonsLvls.resize(4);
    for(int i = 0; i < polygonFinalInt.size(); i++){
        switch(polygonFinalInt.at(i).second){
            case 0:
                polygonsLvls.at(0).push_back({polygonFinalInt.at(i).first, 0});
                break;
            
            case 1:
                polygonsLvls.at(1).push_back({polygonFinalInt.at(i).first, 1});
                break;

            case 2:
                polygonsLvls.at(2).push_back({polygonFinalInt.at(i).first, 2});
                break;

            case 3:
                polygonsLvls.at(3).push_back({polygonFinalInt.at(i).first, 3});
                break;
        }
    }

    vector<vector<pair<CGAL::Polygon_with_holes_2<K>, int>>> unionIntPolygon;
    for(int i = 0; i < polygonsLvls.size(); i++){
        if(polygonsLvls.at(i).size() != 0){
            vector<pair<CGAL::Polygon_with_holes_2<K2>, int>> seedPolygonsK2 = checkSeeds(polygonsLvls.at(i));
            vector<pair<CGAL::Polygon_with_holes_2<K>, int>> seedPolygonsK2NoDeg;
            seedPolygonsK2NoDeg.resize(seedPolygonsK2.size());

            for(int j = 0; j < seedPolygonsK2.size(); j++){
                CGAL::Polygon_with_holes_2<K> polygonNoDeg =  deleteDegeneratedSegments(convertPolyWithHoles2K(seedPolygonsK2.at(j).first));
                seedPolygonsK2NoDeg[j] = {polygonNoDeg, seedPolygonsK2.at(j).second};
            }
            
            unionIntPolygon.push_back(seedPolygonsK2NoDeg);
        } 
        
    }

    return(unionIntPolygon);
}

/*************************************/
vector<pair<CGAL::Polygon_with_holes_2<K>, int>> PolygonCreator::mergePolygonsWithSameCleaningLevel(const vector<pair<CGAL::Polygon_with_holes_2<K>, int>>& polygonUnion){

    vector<vector<pair<CGAL::Polygon_with_holes_2<K>, int>>> polygons = checkIntersectingPolygonUnions(polygonUnion);
    vector<pair<CGAL::Polygon_with_holes_2<K>, int>> splitPolygonsTmp = splitPolygonsList(polygons, {});
    vector<vector<pair<CGAL::Polygon_with_holes_2<K>, int>>> finalPolygons = checkIntersectingPolygonUnions(splitPolygonsTmp);

    for(int i = 0; i < finalPolygons.size(); i++){
        if(finalPolygons.at(i).at(finalPolygons.at(i).size()-1).first.outer_boundary().area() == finalPolygons.at(i).at(finalPolygons.at(i).size()-2).first.outer_boundary().area()){
            finalPolygons.at(i).pop_back();
        }
    }

    return(splitPolygonsList(finalPolygons, {}));

}

/*************************************/
vector<pair<CGAL::Polygon_with_holes_2<K>, int>> PolygonCreator::deleteAreaOverlaps(const vector<pair<CGAL::Polygon_with_holes_2<K>, int>>& polygonUnionFinal){

    vector<vector<pair<CGAL::Polygon_with_holes_2<K>, int>>> polygonUnionFinalNoOverlaps;
    for(int i = 0; i < polygonUnionFinal.size(); i++){
        
        vector<pair<CGAL::Polygon_with_holes_2<K2>, int>> targetPoly;
        targetPoly.push_back({convertPolyWithHoles2K2(polygonUnionFinal.at(i).first), polygonUnionFinal.at(i).second});
        jump:
        for(int j = 0; j < targetPoly.size(); j++){

            for(int k = 0; k < polygonUnionFinal.size(); k++){
                if(k != i){
                    if(CGAL::do_intersect(targetPoly.at(j).first, convertPolyWithHoles2K2(polygonUnionFinal.at(k).first))){
                        if(targetPoly.at(j).second < polygonUnionFinal.at(k).second){
                            Pwh_list_2 diffR;
                            CGAL::difference(targetPoly.at(j).first, convertPolyWithHoles2K2(polygonUnionFinal.at(k).first), std::back_inserter(diffR));
                            targetPoly.erase(targetPoly.begin() + j);
                            for(auto it = diffR.begin(); it != diffR.end(); it++){
                                targetPoly.push_back({*it, polygonUnionFinal.at(i).second});
                            }

                            goto jump;
                        }
                    }
                }      
            }
        }

        polygonUnionFinalNoOverlaps.push_back(convertPolygonIntPairVector2K(targetPoly));
    }

    return(splitPolygonsList(polygonUnionFinalNoOverlaps, {}));
}

/*************************************/
CGAL::Polygon_with_holes_2<K2> PolygonCreator::mergePolygons(const vector<pair<CGAL::Polygon_with_holes_2<K>, int>>& polygonUnion){

    CGAL::Polygon_with_holes_2<K2> firstPolygon(convertPoly2K2(polygonUnion.at(0).first.outer_boundary()));
    vector<CGAL::Polygon_with_holes_2<K2>> mergedPolygons;
    mergedPolygons.push_back(firstPolygon);  

    int i = 0;
    int counter = 1;
    while(counter < polygonUnion.size()){
        CGAL::Polygon_with_holes_2<K2> merge = mergedPolygons.at(i);
        for(int j = 0; j < polygonUnion.size(); j++){
            if(CGAL::do_intersect(merge, convertPolyWithHoles2K2(polygonUnion.at(j).first)) == true){
                if(polygonUnion.at(j).first.outer_boundary().size() != 0){
                    CGAL::join(merge, convertPolyWithHoles2K2(polygonUnion.at(j).first), merge);
                }   
                counter++;
            }
        }
        mergedPolygons.push_back(merge);
        i++;
    }

    return(mergedPolygons.at(mergedPolygons.size()-1));
}

/*************************************/
bool PolygonCreator::checkPolygonsBelong(CGAL::Polygon_2<K> hole, vector<pair<CGAL::Polygon_with_holes_2<K>, int>>* polygonUnionFinalNoOverlaps){
    
    bool findPolygon = false;
    for(int i = 0; i < polygonUnionFinalNoOverlaps->size(); i++){
        for(int j = 0; j < polygonUnionFinalNoOverlaps->at(i).first.number_of_holes(); j++){
            if(hole == polygonUnionFinalNoOverlaps->at(i).first.holes().at(j)){
                polygonUnionFinalNoOverlaps->at(i).first.erase_hole(polygonUnionFinalNoOverlaps->at(i).first.holes_begin() + j);
                findPolygon = true;
            }
        }
    }

    if(findPolygon == true){
        return true;
    }
    else{
        return false;
    }

}

/*************************************/
void PolygonCreator::checkPolygonsAdjacence(CGAL::Polygon_2<K> inputHole, vector<pair<CGAL::Polygon_with_holes_2<K>, int>>* polygonUnionFinalNoOverlaps){

    CGAL::Polygon_2<K> hole;
    if(inputHole.is_simple() == false){
        hole = simplifyPolygon(inputHole);
    }
    else{
        hole = inputHole;
    }

    int highestLevel = -1;
    int polygonIndex = -1;

    if(hole.is_clockwise_oriented() == true){
        hole.reverse_orientation();
    }

    for(int i = 0; i < polygonUnionFinalNoOverlaps->size(); i++){
        if(CGAL::do_intersect(convertPoly2K2(hole), convertPolyWithHoles2K2(polygonUnionFinalNoOverlaps->at(i).first)) == true){
            if(polygonUnionFinalNoOverlaps->at(i).second >= highestLevel){
                highestLevel = polygonUnionFinalNoOverlaps->at(i).second;
                polygonIndex = i;
            }
        }
    }

    if(polygonIndex != -1){
        CGAL::Polygon_with_holes_2<K2> polygonHoleUnion;
        CGAL::join(convertPoly2K2(hole), convertPolyWithHoles2K2(polygonUnionFinalNoOverlaps->at(polygonIndex).first), polygonHoleUnion);
        polygonUnionFinalNoOverlaps->insert(polygonUnionFinalNoOverlaps->begin() + polygonIndex, {convertPolyWithHoles2K(polygonHoleUnion), polygonUnionFinalNoOverlaps->at(polygonIndex).second});
        polygonUnionFinalNoOverlaps->erase(polygonUnionFinalNoOverlaps->begin() + polygonIndex + 1);
    }

}

/*************************************/
void PolygonCreator::deleteHoles(CGAL::Polygon_with_holes_2<K2>* contour, 
                                vector<pair<CGAL::Polygon_with_holes_2<K>, int>>* polygonUnionFinalNoOverlaps,
                                const float areaThreshold){

    if(contour->number_of_holes() != 0){
        jump:
        for(int i = 0; i < contour->number_of_holes(); i++){
            if(abs(CGAL::to_double(contour->holes().at(i).area())) <= areaThreshold){
                bool polygonBelong;
                polygonBelong = checkPolygonsBelong(convertPoly2K(contour->holes().at(i)), polygonUnionFinalNoOverlaps);

                if(polygonBelong == false){
                    checkPolygonsAdjacence(convertPoly2K(contour->holes().at(i)), polygonUnionFinalNoOverlaps);
                }

                contour->holes().erase(contour->holes().begin() + i);
                goto jump;
            }
        }
    }
    
}

/*************************************/
vector<vector<int>> PolygonCreator::createAdjMatrixFirstLevel(vector<pair<CGAL::Polygon_with_holes_2<K>, int>>& polygons){
    
    vector<vector<int>> m_adj;
    int N = polygons.size(); 
    m_adj.resize(N);

    int adjMatrix[N+1][N+1];

    for(int i = 0; i < polygons.size(); i++){

        CGAL::Polygon_with_holes_2<K> poly1WithHoles = deleteDegeneratedSegments(polygons.at(i).first);
        CGAL::Polygon_2<K> poly1;
        if(poly1WithHoles.outer_boundary().is_simple() == false){
           poly1 = simplifyPolygon(poly1WithHoles.outer_boundary());
        }
        else{
            poly1 = poly1WithHoles.outer_boundary();
        }
        
        for(int j = 0; j < polygons.size(); j++){

            CGAL::Polygon_with_holes_2<K> poly2WithHoles = deleteDegeneratedSegments(polygons.at(j).first);
            CGAL::Polygon_2<K> poly2;
            if(poly2WithHoles.outer_boundary().is_simple() == false){
                poly2 = simplifyPolygon(poly2WithHoles.outer_boundary());
            }
            else{
                poly2 = poly2WithHoles.outer_boundary();
            }

            CGAL::Polygon_with_holes_2<K2> polyUnion;
            if(j != i){
                if(CGAL::join(convertPoly2K2(poly1), convertPoly2K2(poly2), polyUnion) == true){
                    m_adj.at(i).push_back(j);
                    adjMatrix[i+1][j+1] = 1;
                }
                else{
                    adjMatrix[i+1][j+1] = 0;
                }
            }
            else{
                adjMatrix[i+1][j+1] = 1;
            }
        }

        cout << "--------------POLYGON N: " << i << "--------------" << endl;
        for(int j = 0; j < m_adj.at(i).size(); j++){
            cout << "Poly " << m_adj.at(i).at(j) << endl;
        }
    }

    for(int i = 1; i < N+1; i++){
        adjMatrix[0][i] = i;
        adjMatrix[i][0] = i;
    }

    adjMatrix[0][0] = 0;

    cout << endl;
    cout << "-----------------------------------" << endl;

    for(int i = 0; i < N+1; i++){
        for(int j = 0; j < N+1; j++){
            if(i == 0 && j >= 10){
                cout << adjMatrix[i][j] << " ";
            }
            else if(j == 0 && i >= 10){
                cout << adjMatrix[i][j] << " ";
            }
            else{
                cout << adjMatrix[i][j] << "  ";
            }
        }
        cout << endl;
    }

    cout << "-----------------------------------" << endl;

    return m_adj;

}

/*************************************/
void PolygonCreator::plotAdjacency(vector<pair<CGAL::Polygon_with_holes_2<K>, int>>& polygons, vector<vector<int>>& adj){

    vector<C::Point_2> barycentres;
    barycentres.resize(polygons.size());

    for(int i = 0; i < polygons.size(); i++){

        vector<int>cleaningLvl(polygons.at(i).first.outer_boundary().size(), polygons.at(i).second);
        m_cph.printPolygonsWithHolesK(polygons.at(i).first, m_cph.m_barycenterPolys, false, 0, 0, 0, 255, "Barycenter Polygons", false, true, false, cleaningLvl);
    
        vector<K::Point_2> polygonPoints = polygonToPoints(polygons.at(i).first.outer_boundary());
        
        vector<pair<C::Point_2, double>> weightedPolygonPoints;
        weightedPolygonPoints.resize(polygonPoints.size());
        for(int j = 0; j < polygonPoints.size(); j++){
            weightedPolygonPoints[j] = std::make_pair(C::Point_2(CGAL::to_double(polygonPoints.at(j).x()), CGAL::to_double(polygonPoints.at(j).y())), 1.0);
        }

        barycentres[i] = CGAL::barycenter(weightedPolygonPoints.begin(), weightedPolygonPoints.end());

    }
   
    for(int i = 0; i < adj.size(); i++){
        
        if(adj.at(i).empty() == false){
            for(int j = 0; j < adj.at(i).size(); j++){
                vector<K::Point_2> barycenterLine;
                barycenterLine.resize(2);
                
                barycenterLine[0] = K::Point_2(CGAL::to_double(barycentres[i].x()), CGAL::to_double(barycentres[i].y()));
                barycenterLine[1] = K::Point_2(CGAL::to_double(barycentres[adj.at(i).at(j)].x()), CGAL::to_double(barycentres[adj.at(i).at(j)].y()));

                vector<int> cleaningLvl(2, polygons.at(i).second);

                m_cph.plotPath(barycenterLine, m_cph.m_barycenterPolys, 0, 0, 0, 255, false, "Barycenter Polygons", false, false, cleaningLvl);       
            }
        }
        
    }

    vector<int>cleaningLvlFinal(polygons.at(polygons.size()-1).first.outer_boundary().size(), polygons.at(polygons.size()-1).second);
    m_cph.printPolygonsWithHolesK(polygons.at(polygons.size()-1).first, m_cph.m_barycenterPolys, false, 0, 0, 0, 255, "Barycenter Polygons", false, true, true, cleaningLvlFinal);

}

/*************************************/
void PolygonCreator::checkAdjacency(vector<pair<CGAL::Polygon_with_holes_2<K>, int>>& polygons){

    CoveragePathCreator cpc;
    vector<CGAL::Polygon_2<K>> decomposedSegments;

    Polygon_list decomposedListOfPolys;
    vector<K::Point_2> decomposedListOfVertices;

    vector<pair<CGAL::Polygon_with_holes_2<K>, int>> simplifiedPolygons;
    simplifiedPolygons.resize(polygons.size());

    for(int i = 0; i < polygons.size(); i++){

        vector<K::Point_2> polygonPoints = cpc.simplifyPerimeter(polygons.at(i).first.outer_boundary());
        //vector<K::Point_2> polygonPoints = polygonToPoints(polygons.at(i).first.outer_boundary());

        CGAL::Polygon_2<K> polySimp;
        polySimp.resize(polygonPoints.size());
        for(int j = 0; j < polygonPoints.size(); j++){
            polySimp[j] = polygonPoints.at(j); 
        }

        CGAL::Polygon_with_holes_2<K> polySimpWithHoles(polySimp);
        simplifiedPolygons[i] = {polySimpWithHoles, polygons.at(i).second};

        pair<Polygon_list, vector<K::Point_2>> decomposedPolys = cpc.decompose(polygonPoints, 1);

        vector<CGAL::Polygon_2<K>> polygonsVector;

        for(auto it = decomposedPolys.first.begin(); it != decomposedPolys.first.end(); it++){
            
            CGAL::Polygon_2<K> polyTmp;
            for(Point p: it->container()){
                K::Point_2 point(decomposedPolys.second[p].x(), decomposedPolys.second[p].y());
                polyTmp.push_back(point);
            }

            polygonsVector.push_back(polyTmp);
        }

        for(int j = 0; j < polygonsVector.size(); j++){
            decomposedSegments.push_back(polygonsVector.at(j));

            CGAL::Polygon_with_holes_2<K> polyWithholesTmp(polygonsVector.at(j));
            vector<int> clnLvls(polyWithholesTmp.outer_boundary().size(), polygons.at(i).second);
            m_cph.printPolygonsWithHolesK(polyWithholesTmp, m_cph.m_decomposedPolys, false, 0, 0, 0, 255, "Decomposed Polygons", false, true, false, clnLvls);
        }

    }
    
    CGAL::Polygon_with_holes_2<K> polyWithholesTmpFin(decomposedSegments.at(decomposedSegments.size()-1));
    vector<int> clnLvlsFin(polyWithholesTmpFin.outer_boundary().size(), polygons.at(polygons.size()-1).second);
    m_cph.printPolygonsWithHolesK(polyWithholesTmpFin, m_cph.m_decomposedPolys, false, 0, 0, 0, 255, "Decomposed Polygons", false, true, true, clnLvlsFin);

    vector<vector<int>> adj = createAdjMatrixFirstLevel(simplifiedPolygons);
    plotAdjacency(polygons, adj);

}

/*************************************/
pair<Polygon_list, vector<K::Point_2>> PolygonCreator::decomposeContour(pair<CGAL::Polygon_with_holes_2<K>, int> contour){

    m_cpc.init(contour.first.outer_boundary(), 0.1, 1);
    m_cpc.run();
    pair<Polygon_list, vector<K::Point_2>> decomposedPolys = {m_cpc.m_decomposedPolysOfIndices, m_cpc.m_decomposedVertices};

    vector<CGAL::Polygon_2<K>> polygonsVector;

    for(auto it = decomposedPolys.first.begin(); it != decomposedPolys.first.end(); it++){
        
        CGAL::Polygon_2<K> polyTmp;
        for(Point p: it->container()){
            K::Point_2 point(decomposedPolys.second[p].x(), decomposedPolys.second[p].y());
            polyTmp.push_back(point);
        }

        polygonsVector.push_back(polyTmp);
    }

    for(int j = 0; j < polygonsVector.size(); j++){
        CGAL::Polygon_with_holes_2<K> polyWithholesTmp(polygonsVector.at(j));
        vector<int> clnLvls(polyWithholesTmp.outer_boundary().size(), contour.second);
        m_cph.printPolygonsWithHolesK(polyWithholesTmp, m_cph.m_decomposedPolys, false, 0, 0, 0, 255, "Decomposed Polygons", false, true, false, clnLvls);
    }

    CGAL::Polygon_with_holes_2<K> polyWithholesTmpFin(polygonsVector.at(polygonsVector.size()-1));
    vector<int> clnLvlsFin(polyWithholesTmpFin.outer_boundary().size(), contour.second);
    m_cph.printPolygonsWithHolesK(polyWithholesTmpFin, m_cph.m_decomposedPolys, false, 0, 0, 0, 255, "Decomposed Polygons", false, true, true, clnLvlsFin);

    return decomposedPolys;

}

vector<pair<K::Point_2, int>> PolygonCreator::defineOptimalPath(int numOfSubPolygons, Polygon_list subPolygons){

    cout << "FINE" << endl;

    vector<pair<float,float>> finalPath = m_cpc.getFinalPath();
    vector<K::Point_2> finalPathCGAL;
    finalPathCGAL.resize(finalPath.size());
    for(int i = 0; i < finalPath.size(); i++){
        finalPathCGAL[i] = K::Point_2(finalPath.at(i).first, finalPath.at(i).second);
    }

    vector<int> cleaningLvl(finalPathCGAL.size(), 4);
    m_cph.plotPath(finalPathCGAL, m_cph.m_decomposedPolys, 0, 0, 0, 255, false, "Path", false, true, cleaningLvl);


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
pair<vector<pair<CGAL::Polygon_with_holes_2<K>, int>>, CGAL::Polygon_with_holes_2<K2>> PolygonCreator::createPolygonFromPath(const vector<K::Point_2>& path){

    m_cph.plotPath(path, m_cph.m_initialPathImage, 0, 0, 0, 255, false, "Path", false, true, m_cleaningLvl); 
    cv::imshow("Path", m_cph.m_initialPathImage);
    cv::waitKey(0);

    clock_t tic = clock();

    vector<pair<vector<K::Segment_2>, vector<int>>> segmentsNoInt; 
    vector<pair<vector<K::Segment_2>, int>> segmentsInt;

    cout << "pathSegmentation" << endl;
    pathSegmentation(path, &segmentsNoInt, &segmentsInt);

    cout << "defineIntersectionPolygons" << endl;
    vector<pair<CGAL::Polygon_with_holes_2<K>, int>> polyIntersection = defineIntersectionPolygons(segmentsInt);

    cout << "cleanLvlDecomposition" << endl;
    vector<vector<pair<vector<K::Segment_2>, int>>> segmentsNoIntDecomposed = cleanLvlDecomposition(segmentsNoInt);

    cout << "segmentsNoIntDecomposed" << endl;
    vector<vector<pair<CGAL::Polygon_with_holes_2<K>, int>>> polyNoIntersection = defineIndipendentPolygons(segmentsNoIntDecomposed);
    
    m_cph.plotPath(path, m_cph.m_polygonsImage, 0, 0, 0, 255, false, "Polygons", false, true, m_cleaningLvl); 

    cout << "checkIntersectingPolygonUnions" << endl;
    vector<vector<pair<CGAL::Polygon_with_holes_2<K>, int>>> seedsInt = checkIntersectingPolygonUnions(polyIntersection);
    
    cout << "plotPolygonsWithCleaningLvl" << endl;
    plotPolygonsWithCleaningLvl(polyNoIntersection, seedsInt, "Polygons");

    cout << "splitPolygonsList" << endl;
    vector<pair<CGAL::Polygon_with_holes_2<K>, int>> polygonUnion = splitPolygonsList(polyNoIntersection, seedsInt);

    cout << "Final seeds" << endl;
    vector<pair<CGAL::Polygon_with_holes_2<K>, int>> polygonUnionFinal = mergePolygonsWithSameCleaningLevel(polygonUnion);

    cout << "Delete overlaps" << endl;
    vector<pair<CGAL::Polygon_with_holes_2<K>, int>> polygonUnionFinalNoOverlaps = deleteAreaOverlaps(polygonUnionFinal);

    cout << "mergePolygons" << endl;
    CGAL::Polygon_with_holes_2<K2> contour = mergePolygons(polygonUnionFinalNoOverlaps);

    vector<int> cleaningLvlContour(contour.outer_boundary().size(), 1);
    m_cph.printPolygonsWithHolesK2(contour, m_cph.m_contourImage, false, 0, 0, 0, 255, "Contour", false, true, true, cleaningLvlContour);
    
    cout << "deleteHoles" << endl;
    deleteHoles(&contour, &polygonUnionFinalNoOverlaps, m_areaThreshold);

    vector<int> cleaningLvlContour2(contour.outer_boundary().size(), 3);
    m_cph.printPolygonsWithHolesK2(contour, m_cph.m_contourImage, false, 0, 0, 0, 255, "Contour", false, true, true, cleaningLvlContour2);

    cout << "Delete overlaps" << endl;
    vector<pair<CGAL::Polygon_with_holes_2<K>, int>> polygonUnionFinalNoOverlaps2 = deleteAreaOverlaps(polygonUnionFinalNoOverlaps);

    // checkAdjacency(polygonUnionFinalNoOverlaps2);

    cout << "decomposeContour" << endl;
    pair<Polygon_list, vector<K::Point_2>> decomposedPolygons = decomposeContour(std::make_pair(convertPolyWithHoles2K(contour), 1));

    cout << "defineOptimalPath" << endl;
    defineOptimalPath(decomposedPolygons.first.size(), decomposedPolygons.first);

    clock_t toc = clock();
    double time = (double)(toc-tic);
    cout << "It took "<< 1000*(time/CLOCKS_PER_SEC) << " millisecond(ms)."<< endl;

    cout << "Total area: " << contour.outer_boundary().area() << endl;

    return {polygonUnionFinalNoOverlaps2, contour};

}

/*************************************/
CGAL::Polygon_with_holes_2<K> PolygonCreator::createPolygon(vector<K::Point_2> perimeter, vector<vector<K::Point_2>> banned_areas){

    CGAL::Polygon_2<K> perimeter_polygon = polygonFromClosedPath(perimeter);

    CGAL::Polygon_with_holes_2<K> perimeter_polygon_with_holes(perimeter_polygon);
    m_cph.printPolygonsWithHolesK(perimeter_polygon_with_holes, m_cph.m_contourPerimeterImage, false, 0, 0, 0, 255, "Perimeter contour", true, false, true, m_cleaningLvl);

    CGAL::Polygon_2<K> contour_perimeter = perimeterContour(perimeter_polygon);

    CGAL::Polygon_with_holes_2<K> contour_perimeter_with_holes;
    if(m_applyContouring == true){
        clonePolygon(&contour_perimeter_with_holes, contour_perimeter);
    }
    else{
        clonePolygon(&contour_perimeter_with_holes, perimeter_polygon);
    }

    m_cph.printPolygonsWithHolesK(contour_perimeter_with_holes, m_cph.m_contourPerimeterImage, true, 0, 0, 255, 255, "Perimeter contour", false, false, true, m_cleaningLvl);

    for(int i = 0; i < banned_areas.size(); i++){
        vector<K::Point_2> banned_area = banned_areas.at(i);

        CGAL::Polygon_2<K> banned_area_polygon = polygonFromClosedPath(banned_area);
        CGAL::Polygon_with_holes_2<K> banned_area_with_holes(banned_area_polygon);
        m_cph.printPolygonsWithHolesK(banned_area_with_holes, m_cph.m_contourPerimeterImage, true, 255, 255, 255, 255, "Perimeter contour", false, false, true, m_cleaningLvl);
        m_cph.printPolygonsWithHolesK(banned_area_with_holes, m_cph.m_contourPerimeterImage, false, 0, 0, 0, 255, "Perimeter contour", false, false, true, m_cleaningLvl);

        contour_perimeter_with_holes.add_hole(banned_area_polygon);
    }

    checkBannedAreas(contour_perimeter_with_holes);

    return contour_perimeter_with_holes;

}
