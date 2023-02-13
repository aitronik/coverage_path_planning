#pragma once 

#include <stdlib.h>
#include <iostream>

#include "CoveragePathCreator.h"
#include "CoveragePlotHelper.h"
#include "PolygonCreator.h"

class CoveragePath {

    public:
    
        CoveragePath();
        ~CoveragePath();
        bool init(int type, int cleanlvl, vector<K::Point_2> path, vector<vector<K::Point_2>> banned_areas_cgal);

        /**
         * @brief Initialization of the PolygonCreator class
         * 
         * @param fake_offset Float defining the fake thickness of the contour
         * @param contour_offset Float defining the fake thickness of the contour
         * @param area_threshold Float defining the area threshold for the holes in the final contour
         * @param perimeter_offset Float defining the thickness of the perimeter contour
         * @param apply_contouring Boolean that defines if the class has to apply the contouring function
         */
        void setPolygonCreatorInit(float fake_offset, float contour_offset, float area_threshold, float perimeter_offset, bool apply_contouring,const vector<int>& cleaningLvl);

        /**
         * @brief Total pipeline of the class
         * 
         */
        CGAL::Polygon_with_holes_2<K> run();
    
    private:
        
        /**
         * @brief Bool that contains the two possible inputs to the class (path or perimeter)
         * 
         */
        int input_type;

        /**
         * @brief Clean level
         * 
         */
        int clean_level;

        /**
         * @brief Input path. It could be an open path or open/closed perimeter
         * 
         */
        vector<K::Point_2> input_path;

        /**
         * @brief Vector of vectors of CGAL points containing the keep out zones 
         * 
         */
        vector<vector<K::Point_2>> keep_out_zones;

        /**
         * @brief Enum that describes the input types (PATH or PERIMETER)
         * 
         */
        enum type {PATH=0, PERIMETER=1};

        /**
         * @brief CoveragePathCreator class
         * 
         */
        CoveragePathCreator cpc;

        /**
         * @brief PolygonCreator class
         * 
         */
        PolygonCreator pc;

        /**
         * @brief Get the Input Polygon for CoveragePathCreator
         * 
         * @return CGAL::Polygon_with_holes_2<K> Polygon to be used in CoveragePathCreator
         */
        CGAL::Polygon_with_holes_2<K> getInputPolygon();

        /**
         * @brief Initialization of the CoveragePathCreator class
         * 
         * @param[in] polygon 
         * @param[in] sweepDistance 
         * @param[in] m_decompositionType 
         */
        void setCoveragePathCreatorInit(CGAL::Polygon_2<K> polygon, float sweepDistance, int m_decompositionType);

};