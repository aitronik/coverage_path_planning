#pragma once 

#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/optional.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/intersections.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/create_offset_polygons_from_polygon_with_holes_2.h>
#include <CGAL/draw_polygon_2.h>
#include <CGAL/Kernel/global_functions.h>
#include <CGAL/enum.h>
#include <CGAL/barycenter.h>
#include <CGAL/Polyline_simplification_2/simplify.h>

#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_segment_traits_2.h>

#include "CoveragePathCreator.h"
#include "CgalPlotHelper.h"
#include "utils.hpp"

#define CGAL_LICENSE_CHECK_H
#define CGAL_LICENSE_WARNING 

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Exact_predicates_exact_constructions_kernel   K2;

typedef CGAL::Simple_cartesian<double>  C;

typedef CGAL::Polygon_with_holes_2<K>       PolygonWithHoles;
typedef boost::shared_ptr<PolygonWithHoles> PolygonWithHolesPtr;
typedef std::vector<PolygonWithHolesPtr>    PolygonWithHolesPtrVector;

typedef std::list<CGAL::Polygon_with_holes_2<K2>>       Pwh_list_2;
typedef std::list<CGAL::Polygon_with_holes_2<K>>        PwhListK;

namespace PS = CGAL::Polyline_simplification_2;
typedef PS::Stop_above_cost_threshold Stop;

using TraitsT = CGAL::Arr_segment_traits_2<K2>;
using ArrangementT = CGAL::Arrangement_2<TraitsT>;
using PointT = TraitsT::Point_2;
using SegmentT = TraitsT::Segment_2;

class PolygonCreator {

    public:

        PolygonCreator();
        ~PolygonCreator();
        bool init(float fakeFootprint, float contourOffset, float holeThreshold, float perimeterOffset, float applyContouring, const vector<int>& cleaningLvl);

        /*----------------- OUTPUT FUNCTIONS -----------------*/

        /**
         * @brief Final pipeline for the path contouring. The function takes as input the path as a vector of CGAL points and returns back
         * several polygons with holes, according to the cleaning levels. Each polygon has its own cleaning level.
         * 
         * @param path Input path (vector of CGAL points)
         * @return vector<pair<CGAL::Polygon_with_holes_2<K>, int>> Output polygons with their cleaning level
         */
        pair<vector<pair<CGAL::Polygon_with_holes_2<K>, int>>, CGAL::Polygon_with_holes_2<K2>> createPolygonFromPath(const vector<K::Point_2>& path);

       /**
        * @brief Final pipeline for the perimeter. The function takes as input the path as a vector of CGAL points and returns back a polygons with
        * holes that represents the inner contour of the perimeter, in which the keep out zones are the holes.
        * 
        * @param[in] perimeter Input perimeter
        * @param[in] perimeter Input banned area
        * @return CGAL::Polygon_with_holes_2<K2> Final perimeter with contour
        */
        CGAL::Polygon_with_holes_2<K> createPolygon(vector<K::Point_2> perimeter, vector<vector<K::Point_2>> bannedAreas);



    private:

        /**
         * @brief Float defining the infinitesimal thickness of the initial polygons
         * 
         */
        float m_scaleFootprint;

        /**
         * @brief Float defining the thickness of the contour
         * 
         */
        float m_contourOffset;

       /**
        * @brief Float defining the thickness of the perimeter contour
        * 
        */
        float m_perimeterOffset;

        /**
         * @brief Float defining the area threshold for the holes in the final contour
         * 
         */
        float m_areaThreshold;

        /**
         * @brief Boolean that defines if the class has to apply the contouring function
         * 
         */
        bool m_applyContouring;

       /**
        * @brief CgalPlotHelper class that supports CGAL plots
        * 
        */
        CgalPlotHelper m_cph;

       /**
        * @brief Vector of cleaning levels
        * 
        */
       vector<int> m_cleaningLvl;

       /**
        * @brief Coverage Path Creator class
        * 
        */
       CoveragePathCreator m_cpc;

       /*----------------- PATH FUNCTIONS -----------------*/

        /**
         * @brief Function which, from a vector of CGAL points (Point_2), returns a vector of CGAL vectors on Kernel K (Vector_2)
         * 
         * @param[in] points Input path (vector of CGAL points)
         * @return vector<K::Vector_2> Ouput vector of CGAL vectors
         */
        static vector<K::Vector_2> pointsToVectors(const vector<K::Point_2>& points);

        /**
         * @brief Function which, from a vector of CGAL points (Point_2), returns a vector of CGAL segments on Kernel K (Vector_2).
         * In addition, a cleaning level is linked to each segment.
         * 
         * @param[in] points Input path (vector of CGAL points)
         * @param[in] pathSegments Output segments path (vector of CGAL points)
         * @param[in] cleaningLvlSegments Output segments cleaning levels linked to the segments (vector of int)
         */
        void pointsToSegments(const vector<K::Point_2>& points, vector<K::Segment_2>* pathSegments, vector<int>* cleaningLvlSegments);

        /**
         * @brief Function which, from a CGAL vector of points (Segment_2), returns the corresponding CGAL vector of points on Kernel K (Point_2)
         * 
         * @param[in] segments Input vector of segments
         * @return vector<K::Point_2> Ouput vector of CGAL points
         */
        static vector<K::Point_2> segmentsToPoints(const vector<K::Segment_2>& segments);

       /**
        * @brief Function that returns the vector of points that define the inpuy polygon
        * 
        * @param polygon Input polygon
        * @return vector<K::Point_2> Output points
        */
        static vector<K::Point_2> polygonToPoints(const CGAL::Polygon_2<K> polygon);

        /**
         * @brief Function for calculating a unit vector
         * 
         * @param[in] vector Input CGAL vector
         */
        void unitVector(K::Vector_2* vector);

        /**
         * @brief Function to calculate a vector perpendicular to the input vector
         * 
         * @param[in] vector Input CGAL vector
         * @param[in] isComputedCounterClockwise Boolean which specifies whether the perpendicular vector is calculated by rotating clockwise 
         * or counterclockwise 
         * @return K::Vector_2 Output vector perpendicular to the input one
         */
        static K::Vector_2 perpendicularVector(const K::Vector_2 vector, const bool isComputedCounterClockwise);

        /**
         * @brief Function allowing a segment to be shifted left (counterclockwise) or right (clockwise). The function returns the points
         * that make up the shifted segment.
         * 
         * @param[in] segment Input points of the segment (vector of CGAL points)
         * @param[in] isComputedCounterClockwise Boolean specifying whether to translate the segment left (counterclockwise) or right (clockwise)
         * @param[in] scaleFootprint Translation amplitude
         * @return vector<K::Point_2> Output points of the shifted segment
         */
        vector<K::Point_2> segmentShift(const K::Segment_2 segment, const bool isComputedCounterClockwise, const float scaleFootprint);

        /**
         * @brief Function that allows an entire path to be shifted left (counterclockwise) or right (clockwise)
         * 
         * @param[in] path Input path (vector of CGAL points)
         * @param[in] scaleFootprint Translation amplitude
         * @param[in] isComputedCounterClockwise Boolean specifying whether to translate the segment left (counterclockwise) or right (clockwise)
         * @return vector<K::Point_2> Output points (input ones translated of scaleFootprint (left or right))
         */
        vector<K::Point_2> offsetPath(const auto& path, const float scaleFootprint, const bool isComputedCounterClockwise);

        /**
         * @brief Function to define a polygon given the input paths of upper and lower points. Given a certain OPEN path (which is one of the inputs of this class), 
         * the polygon is estimated using the paths translated to the right and left (see previous functions).
         * 
         * @param[in] pathRight Right shifted path of CGAL points (UP)
         * @param[in] pathLeft Left path of CGAL points (DOWN)
         * @return CGAL::Polygon_2<K> output polygon
         */
        static CGAL::Polygon_2<K> definePolygon(auto& pathLeft, const auto& pathRight);

       /**
        * @brief Function that deletes the degenerated segments (segments with null length) in a polygon
        * 
        * @param polygonWithHoles Input polygon with holes
        * @return CGAL::Polygon_with_holes_2<K> Output polygon without degenerated segments
        */
        CGAL::Polygon_with_holes_2<K> deleteDegeneratedSegments(CGAL::Polygon_with_holes_2<K> polygonWithHoles);

       /**
        * @brief 
        * 
        * @param polygon 
        * @return CGAL::Polygon_2<K> 
        */
        CGAL::Polygon_2<K> deleteIdenticalSegments(CGAL::Polygon_2<K> polygon);

       /**
        * @brief Function that converts a vector of segments from EPICK to EPECK
        * 
        * @param segments Input vector of segments
        * @return vector<K2::Segments_2> Output vector of segments
        */
        static vector<K2::Segment_2> convertSegmentsToK2(vector<K::Segment_2>& segments);

        /**
         * @brief Function to convert a polygon constructed on the Kernel K(Exact_predicates_inexact_constructions_kernel) with 
         * one constructed on the Kernel K2 (Exact_predicates_exact_constructions_kernel);
         * 
         * @param[in] polygon Input polygon
         * @return CGAL::Polygon_2<K2> Output converted polygon
         */
        static CGAL::Polygon_2<K2> convertPoly2K2(const CGAL::Polygon_2<K> polygon);

       /**
        * @brief Function to convert a polygon constructed on Kernel K2(Exact_predicates_exact_constructions_kernel) with 
                one constructed on the Kernel K (Exact_predicates_inexact_constructions_kernel);
        * 
        * @param polygon Input polygon
        * @return CGAL::Polygon_2<K2> Output converted polygon
        */
        static CGAL::Polygon_2<K> convertPoly2K(const CGAL::Polygon_2<K2> polygon);

        /**
         * @brief Function to convert a polygon with holes constructed on Kernel K(Exact_predicates_inexact_constructions_kernel) with 
                one constructed on the Kernel K2 (Exact_predicates_exact_constructions_kernel);
         * 
         * @param[in] polygon_with_holes Input polygon with holes
         * @return CGAL::Polygon_with_holes_2<K2> Output converted polygon with holes
         */
        static CGAL::Polygon_with_holes_2<K2> convertPolyWithHoles2K2(const CGAL::Polygon_with_holes_2<K> polygonWithHoles);

       /**
        * @brief Function to convert a polygon with holes constructed on Kernel K2(Exact_predicates_exact_constructions_kernel) with 
                one constructed on the Kernel K (Exact_predicates_inexact_constructions_kernel);
        * 
        * @param[in] polygon_with_holes Input polygon with holes
        * @return CGAL::Polygon_with_holes_2<K> Output converted polygon with holes
        */
        static CGAL::Polygon_with_holes_2<K> convertPolyWithHoles2K(const CGAL::Polygon_with_holes_2<K2> polygonWithHoles);

       /**
        * @brief Function that converts a vector of pair CGAL::Polygon_with_holes_2<K2> - int to CGAL::Polygon_with_holes_2<K> - int
        * 
        * @param polyIntVector Input vector
        * @return vector<pair<CGAL::Polygon_with_holes_2<K>, int>> Output vector
        */
       vector<pair<CGAL::Polygon_with_holes_2<K>, int>> convertPolygonIntPairVector2K(vector<pair<CGAL::Polygon_with_holes_2<K2>, int>> polyIntVector);

       /**
        * @brief 
        * 
        * @param pathUp 
        * @param pathDown 
        * @return CGAL::Polygon_2<K> 
        */
       CGAL::Polygon_2<K> simplifyPolygon2(vector<K::Point_2>& pathUp, vector<K::Point_2>& pathDown);

       /**
        * @brief Create a Arrangement object
        * 
        * @param polygon 
        * @return vector<CGAL::Polygon_2> 
        */
       vector<CGAL::Polygon_2<K2>> createArrangement(CGAL::Polygon_with_holes_2<K2> polygon);

       /**
        * @brief 
        * 
        * @param polygon 
        * @return vector<CGAL::Polygon_2<K2>> 
        */
       vector<CGAL::Polygon_with_holes_2<K2>> simplifyPolygon3(CGAL::Polygon_with_holes_2<K2> polygon);

        /**
         * @brief Function allowing the path to be segmented according to self-intersections. In particular, two types of subpaths are defined:
                - portions of the path in which no intersections occur;
                - portions of the path in which intersecting segments are joined.
                In addition, the indexes of the intersections are saved in a vector.
         * 
         * @param[in] path Input path (vector of CGAL points)
         * @param[in] segments_no_int Segments without intersections
         * @param[in] segments_int Segments with intersections
         * @return vector<pair<int, int>> Output vector of intersection indexes
         */
        void pathSegmentation(const vector<K::Point_2>& path, 
                                                 vector<pair<vector<K::Segment_2>, vector<int>>>* segmentsNoInt, 
                                                 vector<pair<vector<K::Segment_2>, int>>* segmentsInt);

        /**
         * @brief Function that returns as output the polygons (with arbitrary thickness, equal to contour_offset) relating to the intersecting segments.
         * 
         * @param[in] segments_int Input segments (portions of the path in which intersecting segments are joined)
         * @return vector<CGAL::Polygon_with_holes_2<K>> Output polygons with holes
         */
        vector<pair<CGAL::Polygon_with_holes_2<K>, int>> defineIntersectionPolygons(const vector<pair<vector<K::Segment_2>, int>>& segmentsInt);

       /**
        * @brief The function splits the subpath based on the cleaning levels
        * 
        * @param subpath Input subpath
        * @return vector<pair<vector<K::Segment_2>, int>> Output splitted subpath based on cleaning level
        */
        vector<pair<vector<K::Segment_2>, int>> checkCleaningLvlContinuity(const pair<vector<K::Segment_2>, vector<int>>& subpath);

        /**
        * @brief Function that decompose the non intersecating polygons into subpolygons based on cleaning level.
        * 
        * @param segmentsNoInt Non intersecating polygons
        * @return vector<vector<pair<vector<K::Segment_2>, int>>> Output polygons with cleaning level
        */
        vector<vector<pair<vector<K::Segment_2>, int>>> cleanLvlDecomposition(const vector<pair<vector<K::Segment_2>, vector<int>>>& segmentsNoInt);

        /**
         * @brief Function that returns as output the polygons (with arbitrary thickness, equal to contour_offset) relating to the segments that do NOT 
                intersect each other.
         * 
         * @param[in] segmentsListWithCleaningLvl Input segments (portions of the path in which no intersections occur)
         * @return vector<vector<pair<CGAL::Polygon_with_holes_2<K>, int>>> Output polygons with holes
         */
        vector<vector<pair<CGAL::Polygon_with_holes_2<K>, int>>> defineIndipendentPolygons(const vector<vector<pair<vector<K::Segment_2>, int>>>& segmentsListWithCleaningLvl);

       /**
        * @brief Function that plots the subpolygons based on the cleaning level
        * 
        * @param polygonFinalNoInt Vector of input polygons (the ones without intersections)
        * @param polygonFinalInt Vector of input polygons (the ones with intersections)
        * @param name Name of the CV matrix
        */
        void plotPolygonsWithCleaningLvl(const vector<vector<pair<CGAL::Polygon_with_holes_2<K>, int>>>& polygonFinalNoInt, 
                                          const vector<vector<pair<CGAL::Polygon_with_holes_2<K>, int>>>& polygonFinalInt,
                                          const std::string& name);

       /**
        * @brief Function that splits and groups all the polygons in a unique vector of polygons
        * 
        * @param polygonFinalNoInt Vector of input polygons (the ones without intersections)
        * @param polygonFinalInt Vector of input polygons (the ones with intersections)
        */
        static vector<pair<CGAL::Polygon_with_holes_2<K>, int>> splitPolygonsList(const vector<vector<pair<CGAL::Polygon_with_holes_2<K>, int>>>& polygonFinalNoInt, 
                                                                             const vector<vector<pair<CGAL::Polygon_with_holes_2<K>, int>>>& polygonFinalInt);

       /**
        * @brief Function that, given a certain number of subpolygons, returns all the possible unions of the subpolygons 
        * 
        * @param subpolygons Input subpolygons
        * @return vector<pair<CGAL::Polygon_with_holes_2<K>, int>> Output merged polygons
        */
        vector<pair<CGAL::Polygon_with_holes_2<K2>, int>> checkSeeds(vector<pair<CGAL::Polygon_with_holes_2<K>, int>>& subpolygons);

       /**
        * @brief Adiacent interction polygons with the same cleaning level are joined togheter
        * 
        * @param polygonFinalInt Vector of input polygons (the ones with intersections)
        * @return vector<pair<CGAL::Polygon_with_holes_2<K>, int>> Output array of joined polygons
        */
        vector<vector<pair<CGAL::Polygon_with_holes_2<K>, int>>> checkIntersectingPolygonUnions(const vector<pair<CGAL::Polygon_with_holes_2<K>, int>>& polygonFinalInt);

        /**
         * @brief Function that joins all polygons (with or without intersection) in order of appearance.
         * 
         * @param[in] polygonUnion Vector of input polygons (the complete list of polygons with the associated cleaning level)
         * @return CGAL::Polygon_with_holes_2<K2> 
         */
        CGAL::Polygon_with_holes_2<K2> mergePolygons(vector<pair<CGAL::Polygon_with_holes_2<K>, int>>& polygonUnion);
        
        /**
         * @brief Given the final vector of polygons, the function merges the adjacent polygons with the same cleaning level
         * 
         * @param polygonUnion Input final vector of polygons
         * @return vector<pair<CGAL::Polygon_with_holes_2<K>, int>> Merged Polygons
         */
        vector<pair<CGAL::Polygon_with_holes_2<K>, int>> mergePolygonsWithSameCleaningLevel(const vector<pair<CGAL::Polygon_with_holes_2<K>, int>>& polygonUnion);
        
       /**
        * @brief Function that deletes the areas overlaps
        * 
        * @param polygonUnionFinal Input polygons with overlaps
        * @return vector<pair<CGAL::Polygon_with_holes_2<K>, int>> Output polygons without overlaps 
        */
        vector<pair<CGAL::Polygon_with_holes_2<K>, int>> deleteAreaOverlaps(const vector<pair<CGAL::Polygon_with_holes_2<K>, int>>& polygonUnionFinal);
        
        /**
         * @brief Function that checks if an input hole belongs to one of the polygons, which were segmented using cleaning levels. If the hole belongs
         * to one of the polygons, it is deleted.
         * 
         * @param hole Input hole
         * @param polygonUnionFinalNoOverlaps Vector of polygons (eventually cleaned from the hole) 
         */
        static bool checkPolygonsBelong(CGAL::Polygon_2<K> hole, vector<pair<CGAL::Polygon_with_holes_2<K>, int>>* polygonUnionFinalNoOverlaps);
        
        /**
         * @brief Function that splits a non-simple polygon into simple subpolygons
         * 
         * @param polygon Input non-simple polygon
         * @return vector<CGAL::Polygon_2<K>> Output vector of simple polygons
         */
        CGAL::Polygon_2<K> simplifyPolygon(const CGAL::Polygon_2<K> polygon);

        /**
         * @brief Function that check the holes external to the polygons and merge them to the adjacent polygon with highest cleaning level
         * 
         * @param hole Input hole
         * @param polygonUnionFinalNoOverlaps Vector of polygons (eventually merged with the hole)
         */
        void checkPolygonsAdjacence(CGAL::Polygon_2<K> inputHole, vector<pair<CGAL::Polygon_with_holes_2<K>, int>>* polygonUnionFinalNoOverlaps);

        /**
         * @brief Function that eliminates undercut holes from the final contour.
         * 
         * @param[in] contour Final contour
         * @param[in] areaThreshold Area threshold for the holes of the contour 
         * @param[in] pathImage CV Matrix on which the path/polygon is plotted
         * @param[in] name Name of the CV matrix
         */
        void deleteHoles(CGAL::Polygon_with_holes_2<K2>* contour, 
                            vector<pair<CGAL::Polygon_with_holes_2<K>, int>>* polygonUnionFinalNoOverlaps, 
                            const float areaThreshold);

       /**
        * @brief Create an adjacency matrix given an input set of polygons
        * 
        * @param polygons Input set of polygons
        */
       vector<vector<int>> createAdjMatrixFirstLevel(vector<pair<CGAL::Polygon_with_holes_2<K>, int>>& polygons);

       /**
        * @brief Plot the polygons and their connections (adjacency)
        * 
        * @param polygons Input set of polygons
        * @param m_adj Adjacency matrix
        */
       void plotAdjacency(vector<pair<CGAL::Polygon_with_holes_2<K>, int>>& polygons, vector<vector<int>>& adj);

       /**
        * @brief Function that decomposes the contour of the input path
        * 
        * @param contour Contour of the path
        * @return pair<Polygon_list, vector<K::Point_2>> Output list of polygons 
        */
        pair<Polygon_list, vector<K::Point_2>> decomposeContour(pair<CGAL::Polygon_with_holes_2<K>, int> contour);

       /**
        * @brief 
        * 
        * @param polygonUnionFinalNoOverlaps2 
        */
        void defineOptimalPath(vector<pair<CGAL::Polygon_with_holes_2<K>, int>> polygonUnionFinalNoOverlaps2);

        /*----------------- PERIMETER FUNCTIONS -----------------*/

       /**
        * @brief Function that returns the contour of the perimeter
        * 
        * @param[in] perimeter_polygon Input polygon of the perimeter
        * @return CGAL::Polygon_with_holes_2<K2> Contour of the perimeter
        */
       CGAL::Polygon_2<K> perimeterContour(const CGAL::Polygon_2<K> perimeterPolygon);

       /**
        * @brief Function that selects the polygon with the largest area
        * 
        * @param[in] contour Vector of polygons generated with the contouring function
        * @return CGAL::Polygon_with_holes_2<K2> Largest polygon
        */
       static CGAL::Polygon_with_holes_2<K> selectMajorPolygon(const vector<CGAL::Polygon_with_holes_2<K>>& contours);

       /**
        * @brief Function that returns the output polygon from a closed path of points
        * 
        * @param[in] perimeter Input closed path of points
        * @return CGAL::Polygon_2<K> Output polygon
        */
       static CGAL::Polygon_2<K> polygonFromClosedPath(const vector<K::Point_2>& path);

       /**
        * @brief Function that copies the input polygon (poly_to_clone) as outer boundary for the final_poly_with_holes
        * 
        * @param[in] final_poly_with_holes Polygon with holes in which the poly_to_clone polygon is cloned
        * @param[in] poly_to_clone Input polygon to clone
        */
       void clonePolygon(CGAL::Polygon_with_holes_2<K>* finalPolyWithHoles, const CGAL::Polygon_2<K> polyToClone);

       /**
        * @brief Function that returns if one (or more) banned area(s) split(s) the perimeter in subpolygons
        * 
        * @param[in] contour Input contour (perimeter) with holes (banned area)
        */
       static void checkBannedAreas(const CGAL::Polygon_with_holes_2<K> contour);

};

