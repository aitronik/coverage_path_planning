#pragma once 

#include <stdlib.h>
#include <iostream>

#include "CoveragePlotHelper.h"
#include "PolygonCreator.h"

enum type {PATH=0, PERIMETER=1};

class CoveragePath {

    public:
    
        CoveragePath();
        ~CoveragePath();
        bool init(int type, int cleanlvl, vector<K::Point_2> path, vector<vector<K::Point_2>> banned_areas_cgal);

    
    private:
        
        /**
         * @brief Bool that contains the two possible inputs to the class (path or perimeter)
         * 
         */
        bool input_type;

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
         * @brief vector of vectors of CGAL points containing the keep out zones 
         * 
         */
        vector<vector<K::Point_2>> keep_out_zones;

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

};