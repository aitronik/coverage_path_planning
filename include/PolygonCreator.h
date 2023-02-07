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

#include "CoveragePlotHelper.h"

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
        bool init(float sf, float oc, float at, float po, float ap);

        /*----------------- OUTPUT FUNCTIONS -----------------*/

        /**
         * @brief Final pipeline for the path
         * 
         * @param path Input path (vector of CGAL points)
         * @return CGAL::Polygon_with_holes_2<K2> Output contour
         */
        CGAL::Polygon_with_holes_2<K2> createPolygonFromPath(vector<K::Point_2> path);

       /**
        * @brief Final pipeline for the perimeter
        * 
        * @param[in] perimeter Input perimeter
        * @param[in] perimeter Input banned area
        * @return CGAL::Polygon_with_holes_2<K2> Final perimeter with contour
        */
        CGAL::Polygon_with_holes_2<K> createPolygon(vector<K::Point_2> perimeter, vector<vector<K::Point_2>> banned_areas);



    private:
        /**
         * @brief Initial path image
         * 
         */
        cv::Mat initialPathImage;

        /**
         * @brief Polygons image
         * 
         */
        cv::Mat polygonsImage;

        /**
         * @brief Contour image
         * 
         */
        cv::Mat contourImage;

        /**
         * @brief Contour image
         * 
         */
        cv::Mat contourWithoutHolesImage;

        /**
         * @brief Contour perimeter image
         * 
         */
        cv::Mat contourPerimeterImage;

        /**
         * @brief CoveragePlotHelper class (by Gemma)
         * 
         */
        CoveragePlotHelper cph;

        /**
         * @brief Float defining the infinitesimal thickness of the initial polygons
         * 
         */
        float scale_footprint;

        /**
         * @brief Float defining the thickness of the contour
         * 
         */
        float contour_offset;

       /**
        * @brief Float defining the thickness of the perimeter contour
        * 
        */
        float perimeter_offset;

        /**
         * @brief Float defining the area threshold for the holes in the final contour
         * 
         */
        float areaThreshold;

        /**
         * @brief Boolean that defines if the class has to apply the contouring function
         * 
         */
        bool apply_contouring;

       /*----------------- PATH FUNCTIONS -----------------*/

        /**
         * @brief Function which, from a vector of CGAL points (Point_2), returns a vector of CGAL vectors on Kernel K (Vector_2)
         * 
         * @param[in] path Input path (vector of CGAL points)
         * @return vector<K::Vector_2> Ouput vector of CGAL vectors
         */
        vector<K::Vector_2> defineVectors(vector<K::Point_2> path);

        /**
         * @brief Function which, from a vector of CGAL points (Point_2), returns a vector of CGAL segments on Kernel K (Vector_2)
         * 
         * @param[in] path Input path (vector of CGAL points)
         * @return vector<K::Segment_2> Ouput vector of CGAL segments
         */
        vector<K::Segment_2> defineSegments(vector<K::Point_2> path);

        /**
         * @brief Function which, from a CGAL segment vector (Segment_2), returns a CGAL point vector (path) on Kernel K (Point_2)
         * 
         * @param[in] segments Input vector of segments
         * @return vector<K::Point_2> Ouput vector of CGAL points
         */
        vector<K::Point_2> definePathFromSegments(vector<K::Segment_2> segments);

        /**
         * @brief Function to plot the path using OPENCV
         * 
         * @param[in] path Input path/polygon (vector of CGAL points)
         * @param[in] pathImage CV Matrix on which the path/polygon is plotted
         * @param[in] B Blue level
         * @param[in] G Green level
         * @param[in] R Red level
         * @param[in] polygon Boolean that specifies if the input vector of points is a polygon
         * @param[in] name Name of the CV matrix
         * @param[in] fill Boolean which specifies whether to fill the polygon (only for input polygons)
         * @param[in] resolution Boolean which specifies if the input path should be used to define the resolution of the plot
         * @param[in] x_offset Offset on the x-axis
         * @param[in] y_offset Offset on the y-axis
         */
        void plotPath(vector<K::Point_2> path, 
                     cv::Mat pathImage, 
                     int B, 
                     int G, 
                     int R, 
                     bool polygon, 
                     std::string name, 
                     bool fill, 
                     bool resolution,
                     int x_offset, 
                     int y_offset);

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
         * @param[in] down Boolean which specifies whether the perpendicular vector is calculated rotating clockwise or counterclockwise 
         * @return K::Vector_2 Output vector perpendiculat to the input one
         */
        K::Vector_2 perpendicularVector(K::Vector_2 vector, bool down);

        /**
         * @brief Function allowing a segment to be shifted up or down
         * 
         * @param[in] path Input points of the segment (vector of CGAL points)
         * @param[in] down Boolean specifying whether to translate the segment up or down 
         * @param[in] scale_footprint Translation amplitude
         * @return vector<K::Point_2> Output perpendicular vector
         */
        vector<K::Point_2> segmentShift(vector<K::Point_2> path, bool down, float scale_footprint);

        /**
         * @brief Function that allows an entire path to be shifted up or down
         * 
         * @param[in] path Input path (vector of CGAL points)
         * @param[in] scale_footprint Translation amplitude
         * @param[in] down Boolean specifying whether to translate the segment up or down 
         * @return vector<K::Point_2> Output points (input ones translated of scale_footprint)
         */
        vector<K::Point_2> offsetPath(auto path, float scale_footprint, bool down);

        /**
         * @brief Function to define a polygon given the input paths of upper and lower points
         * 
         * @param[in] path_down Upper path of CGAL points
         * @param[in] path_up Lower path of CGAL points
         * @return CGAL::Polygon_2<K> output polygon
         */
        CGAL::Polygon_2<K> definePolygon(auto path_down, auto path_up);

        /**
         * @brief Function to plot a polygon with holes
         * 
         * @param[in] polygon Input polygon
         * @param[in] cph Input auxiliary class (CoveragePlotHelper by Gemma)
         * @param[in] pathImage CV Matrix on which the path/polygon is plotted
         * @param[in] fill Boolean which specifies whether to fill the polygon (only for input polygons)
         * @param[in] B Blue level
         * @param[in] G Green level
         * @param[in] R Red level
         * @param[in] name Name of the CV matrix
         * @param[in] resolution Boolean which specifies if the input path should be used to define the resolution of the plot
         * @param[in] pathORpolygon Boolean which specifies if the input is a path (true) or a polygon (false). 
         * This parameter is used to set an offset in the path display.
         */
        void printPolygonsWithHoles(auto polygon, 
                                    CoveragePlotHelper cph, 
                                    cv::Mat pathImage, 
                                    bool fill, 
                                    int B, 
                                    int G, 
                                    int R, 
                                    std::string name, 
                                    bool resolution,
                                    bool pathORpolygon);

        /**
         * @brief Function to convert a polygon constructed on the Kernel K(Exact_predicates_inexact_constructions_kernel) with 
                one constructed on the Kernel K2 (Exact_predicates_exact_constructions_kernel);
         * 
         * @param[in] polygon Input polygon
         * @return CGAL::Polygon_2<K2> Output converted polygon
         */
        CGAL::Polygon_2<K2> convertPoly2K2(CGAL::Polygon_2<K> polygon);

        /**
         * @brief Function to convert a polygon with holes constructed on Kernel K(Exact_predicates_inexact_constructions_kernel) with 
                one constructed on the Kernel K2 (Exact_predicates_exact_constructions_kernel);
         * 
         * @param[in] polygon_with_holes Input polygon with holes
         * @return CGAL::Polygon_with_holes_2<K2> Output converted polygon with holes
         */
        CGAL::Polygon_with_holes_2<K2> convertPolyWithHoles2K2(CGAL::Polygon_with_holes_2<K> polygon_with_holes);

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
        vector<pair<int, int>> pathSegmentation(vector<K::Point_2> path, 
                                                vector<vector<K::Segment_2>>* segments_no_int, 
                                                vector<vector<K::Segment_2>>* segments_int);

        /**
         * @brief Function that returns as output the polygons (with arbitrary thickness, equal to contour_offset) relating to the intersecting segments.
         * 
         * @param[in] segments_int[in] Input segments (portions of the path in which intersecting segments are joined)
         * @return vector<CGAL::Polygon_with_holes_2<K>> Output polygons with holes
         */
        vector<CGAL::Polygon_with_holes_2<K>> defineIntersectionPolygons(vector<vector<K::Segment_2>> segments_int);

        /**
         * @brief Function that returns as output the polygons (with arbitrary thickness, equal to contour_offset) relating to the segments that do NOT 
                intersect each other.
         * 
         * @param[in] segments_no_int Input segments (portions of the path in which no intersections occur)
         * @return vector<CGAL::Polygon_with_holes_2<K>> Output polygons with holes
         */
        vector<CGAL::Polygon_with_holes_2<K>> defineIndipendentPolygons(vector<vector<K::Segment_2>> segments_no_int);

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
        CGAL::Polygon_with_holes_2<K2> mergePolygons(vector<CGAL::Polygon_with_holes_2<K>> polygon_final_no_int, 
                                                            vector<CGAL::Polygon_with_holes_2<K>> polygon_final_int,
                                                            vector<pair<int, int>> count_int,
                                                            cv::Mat pathImage,
                                                            std::string name);
        
        
        /**
         * @brief Function that eliminates undercut holes from the final contour.
         * 
         * @param[in] contour Final contour
         * @param[in] areaThreshold Area threshold for the holes of the contour 
         * @param[in] pathImage CV Matrix on which the path/polygon is plotted
         * @param[in] name Name of the CV matrix
         */
        void deleteHoles(CGAL::Polygon_with_holes_2<K2>* contour, float areaThreshold, cv::Mat pathImage, std::string name);

        /*----------------- PERIMETER FUNCTIONS -----------------*/

       /**
        * @brief Function that returns the contour of the perimeter
        * 
        * @param[in] perimeter_polygon Input polygon of the perimeter
        * @return CGAL::Polygon_with_holes_2<K2> Contour of the perimeter
        */
       CGAL::Polygon_2<K> perimeterContour(CGAL::Polygon_2<K> perimeter_polygon);

       /**
        * @brief Function that selects the polygon with the largest area
        * 
        * @param[in] contour Vector of polygons generated with the contouring function
        * @return CGAL::Polygon_with_holes_2<K2> Largest polygon
        */
       CGAL::Polygon_with_holes_2<K> selectMajorPolygon(vector<CGAL::Polygon_with_holes_2<K>> contours);

       /**
        * @brief Function that returns the output polygon from a closed path of points
        * 
        * @param perimeter Input closed path of points
        * @return CGAL::Polygon_2<K> Output polygon
        */
       CGAL::Polygon_2<K> polygonFromClosedPath(vector<K::Point_2> path);

       /**
        * @brief Function that returns if one (or more) banned area(s) split(s) the perimeter in subpolygons
        * 
        * @param contour Input contour (perimeter) with holes (banned area)
        */
       void checkBannedAreas(CGAL::Polygon_with_holes_2<K> contour);

};

