#pragma once 

#include <stdlib.h>
#include <iostream>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_with_holes_2.h>

#include "CoveragePlotHelper.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Exact_predicates_exact_constructions_kernel   K2;

class CgalPlotHelper {

    public:

        CgalPlotHelper();
        ~CgalPlotHelper();

        /**
         * @brief Initial path image
         * 
         */
        cv::Mat m_initialPathImage;

        /**
         * @brief Polygons image
         * 
         */
        cv::Mat m_polygonsImage;

        /**
         * @brief Contour image
         * 
         */
        cv::Mat m_contourImage;

        /**
         * @brief Contour image
         * 
         */
        cv::Mat m_contourWithoutHolesImage;

        /**
         * @brief Contour perimeter image
         * 
         */
        cv::Mat m_contourPerimeterImage;

        /**
         * @brief CoveragePlotHelper class (by Gemma)
         * 
         */
        CoveragePlotHelper m_covph;

        /**
         * @brief Function to plot the path using OPENCV
         * 
         * @param[in] path Input path/polygon (vector of CGAL points)
         * @param[in] pathImage CV Matrix on which the path/polygon is plotted
         * @param[in] B_fill Blue level
         * @param[in] G_fill Green level
         * @param[in] R_fill Red level
         * @param[in] polygon Boolean that specifies if the input vector of points is a polygon
         * @param[in] name Name of the CV matrix
         * @param[in] fill Boolean which specifies whether to fill the polygon (only for input polygons)
         * @param[in] resolution Boolean which specifies if the input path should be used to define the resolution of the plot
         */
        void plotPath(vector<K::Point_2> path, 
                     cv::Mat pathImage, 
                     int B_fill,
                     int G_fill,
                     int R_fill,
                     bool polygon, 
                     std::string name, 
                     bool fill, 
                     bool resolution,
                     vector<int> cleaningLvl);


        /**
         * @brief Function to plot a polygon with holes (KERNEL K)
         * 
         * @param[in] polygon Input polygon
         * @param[in] cph Input auxiliary class (CoveragePlotHelper by Gemma)
         * @param[in] pathImage CV Matrix on which the path/polygon is plotted
         * @param[in] fill Boolean which specifies whether to fill the polygon (only for input polygons)
         * @param[in] B_fill Blue level
         * @param[in] G_fill Green level
         * @param[in] R_fill Red level
         * @param[in] name Name of the CV matrix
         * @param[in] resolution Boolean which specifies if the input path should be used to define the resolution of the plot
         * @param[in] pathORpolygon Boolean which specifies if the input is a path (true) or a polygon (false). 
         * @param[in] printOut Boolean which specifies if the function has to print out the polygon
         */
        void printPolygonsWithHolesK(CGAL::Polygon_with_holes_2<K> polygon,
                                    cv::Mat pathImage, 
                                    bool fill, 
                                    int B_fill,
                                    int G_fill,
                                    int R_fill,
                                    std::string name, 
                                    bool resolution,
                                    bool pathORpolygon,
                                    bool printOut,
                                    vector<int> cleaningLvl);
        
        /**
         * @brief Function to plot a polygon with holes (KERNEL K2)
         * 
         * @param[in] polygon Input polygon
         * @param[in] cph Input auxiliary class (CoveragePlotHelper by Gemma)
         * @param[in] pathImage CV Matrix on which the path/polygon is plotted
         * @param[in] fill Boolean which specifies whether to fill the polygon (only for input polygons)
         * @param[in] B_fill Blue level
         * @param[in] G_fill Green level
         * @param[in] R_fill Red level
         * @param[in] name Name of the CV matrix
         * @param[in] resolution Boolean which specifies if the input path should be used to define the resolution of the plot
         * @param[in] pathORpolygon Boolean which specifies if the input is a path (true) or a polygon (false). 
         * @param[in] printOut Boolean which specifies if the function has to print out the polygon
         */
        void printPolygonsWithHolesK2(CGAL::Polygon_with_holes_2<K2> polygon,
                                    cv::Mat pathImage, 
                                    bool fill, 
                                    int B_fill,
                                    int G_fill,
                                    int R_fill,
                                    std::string name, 
                                    bool resolution,
                                    bool pathORpolygon,
                                    bool printOut,
                                    vector<int> cleaningLvl);

};