#pragma once 

#include <stdlib.h>
#include <iostream>
#include <vector>
#include <boost/shared_ptr.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/intersections.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/create_offset_polygons_from_polygon_with_holes_2.h>
#include <CGAL/draw_polygon_2.h>
#include <CGAL/Kernel/global_functions.h>
#include <CGAL/enum.h>

#include "CgalPlotHelper.h"
#include "utils.hpp"

#define CGAL_LICENSE_CHECK_H
#define CGAL_LICENSE_WARNING 

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Exact_predicates_exact_constructions_kernel   K2;

typedef CGAL::Polygon_with_holes_2<K>       PolygonWithHoles;
typedef boost::shared_ptr<PolygonWithHoles> PolygonWithHolesPtr;
typedef std::vector<PolygonWithHolesPtr>    PolygonWithHolesPtrVector;

typedef std::list<CGAL::Polygon_with_holes_2<K2>>       Pwh_list_2;

class PolygonCreator {

    public:

        PolygonCreator();
        ~PolygonCreator();
        bool init(float fakeFootprint, float contourOffset, float holeThreshold, float perimeterOffset, float applyContouring, const vector<int>& cleaningLvl);

        /*----------------- OUTPUT FUNCTIONS -----------------*/

        /**
         * @brief Final pipeline for the path contouring. The function takes as input the path as a vector of CGAL points and returns back
         * the contouring the path itself. The contour is defined as a polygon with holes.
         * 
         * @param path Input path (vector of CGAL points)
         * @return CGAL::Polygon_with_holes_2<K2> Output contour
         */
        CGAL::Polygon_with_holes_2<K> createPolygonFromPath(const vector<K::Point_2>& path);

       /**
        * @brief Final pipeline for the perimeter
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

       /*----------------- PATH FUNCTIONS -----------------*/

        /**
         * @brief Function which, from a vector of CGAL points (Point_2), returns a vector of CGAL vectors on Kernel K (Vector_2)
         * 
         * @param[in] points Input path (vector of CGAL points)
         * @return vector<K::Vector_2> Ouput vector of CGAL vectors
         */
        static vector<K::Vector_2> pointsToVectors(const vector<K::Point_2>& points);

        /**
         * @brief Function which, from a vector of CGAL points (Point_2), returns a vector of CGAL segments on Kernel K (Vector_2)
         * 
         * @param[in] points Input path (vector of CGAL points)
         * @return vector<K::Segment_2> Ouput vector of CGAL segments
         */
        static vector<K::Segment_2> pointsToSegments(const vector<K::Point_2>& points);

        /**
         * @brief Function which, from a CGAL segment vector (Segment_2), returns a CGAL point vector (path) on Kernel K (Point_2)
         * 
         * @param[in] segments Input vector of segments
         * @return vector<K::Point_2> Ouput vector of CGAL points
         */
        static vector<K::Point_2> segmentsToPoints(const vector<K::Segment_2>& segments);

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
         * @param[in] isComputedCounterClockwise Boolean which specifies whether the perpendicular vector is calculated rotating clockwise or counterclockwise 
         * @return K::Vector_2 Output vector perpendiculat to the input one
         */
        static K::Vector_2 perpendicularVector(const K::Vector_2 vector, const bool isComputedCounterClockwise);

        /**
         * @brief Function allowing a segment to be shifted left or right
         * 
         * @param[in] segment Input points of the segment (vector of CGAL points)
         * @param[in] isComputedCounterClockwise Boolean specifying whether to translate the segment left or right 
         * @param[in] scaleFootprint Translation amplitude
         * @return vector<K::Point_2> Output perpendicular vector
         */
        vector<K::Point_2> segmentShift(const K::Segment_2 segment, const bool isComputedCounterClockwise, const float scaleFootprint);

        /**
         * @brief Function that allows an entire path to be shifted up or down
         * 
         * @param[in] path Input path (vector of CGAL points)
         * @param[in] scale_footprint Translation amplitude
         * @param[in] down Boolean specifying whether to translate the segment up or down 
         * @return vector<K::Point_2> Output points (input ones translated of scale_footprint)
         */
        vector<K::Point_2> offsetPath(const auto& path, const float scale_footprint, const bool down);

        /**
         * @brief Function to define a polygon given the input paths of upper and lower points
         * 
         * @param[in] path_down Upper path of CGAL points
         * @param[in] path_up Lower path of CGAL points
         * @return CGAL::Polygon_2<K> output polygon
         */
        static CGAL::Polygon_2<K> definePolygon(auto& pathDown, const auto& pathUp);

        /**
         * @brief Function to convert a polygon constructed on the Kernel K(Exact_predicates_inexact_constructions_kernel) with 
                one constructed on the Kernel K2 (Exact_predicates_exact_constructions_kernel);
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
        static vector<pair<int, int>> pathSegmentation(const vector<K::Point_2>& path, 
                                                        vector<vector<K::Segment_2>>* segmentsNoInt, 
                                                        vector<vector<K::Segment_2>>* segmentsInt);

        /**
         * @brief Function that returns as output the polygons (with arbitrary thickness, equal to contour_offset) relating to the intersecting segments.
         * 
         * @param[in] segments_int[in] Input segments (portions of the path in which intersecting segments are joined)
         * @return vector<CGAL::Polygon_with_holes_2<K>> Output polygons with holes
         */
        vector<CGAL::Polygon_with_holes_2<K>> defineIntersectionPolygons(const vector<vector<K::Segment_2>>& segmentsInt);

        /**
         * @brief Function that returns as output the polygons (with arbitrary thickness, equal to contour_offset) relating to the segments that do NOT 
                intersect each other.
         * 
         * @param[in] segments_no_int Input segments (portions of the path in which no intersections occur)
         * @return vector<CGAL::Polygon_with_holes_2<K>> Output polygons with holes
         */
        vector<CGAL::Polygon_with_holes_2<K>> defineIndipendentPolygons(const vector<vector<K::Segment_2>>& segmentsNoInt);

        /**
         * @brief Function that joins all polygons (with or without intersection) in order of appearance.
         * 
         * @param[in] polygon_final_no_int Vector of input polygons (the ones without intersections)
         * @param[in] polygon_final_int Vector of input polygons (the ones with intersections)
         * @param[in] count_int Vector of intersection indexes
         * @param[in] pathImage CV Matrix on which the path/polygon is plotted
         * @param[in] name Name of the CV matrix
         * @return CGAL::Polygon_with_holes_2<K2> 
         */
        CGAL::Polygon_with_holes_2<K2> mergePolygons(const vector<CGAL::Polygon_with_holes_2<K>>& polygonFinalNoInt, 
                                                        const vector<CGAL::Polygon_with_holes_2<K>>& polygonFinalInt,
                                                        const vector<pair<int, int>>& countInt,
                                                        const std::string name);
        
        
        /**
         * @brief Function that eliminates undercut holes from the final contour.
         * 
         * @param[in] contour Final contour
         * @param[in] areaThreshold Area threshold for the holes of the contour 
         * @param[in] pathImage CV Matrix on which the path/polygon is plotted
         * @param[in] name Name of the CV matrix
         */
        void deleteHoles(CGAL::Polygon_with_holes_2<K2>* contour, const float areaThreshold, const std::string name);

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

