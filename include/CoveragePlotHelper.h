#pragma once 


#include <stdlib.h>
#include <vector>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/partition_2.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/property_map.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "utils.hpp" //controlla se serve

class CoveragePlotHelper {
    public: 
    
        CoveragePlotHelper();
        ~CoveragePlotHelper();
        bool init(vector<K::Point_2>& perimeter_vertices);

        /**
         * @brief stampa un poligono in nero su un'immagine denominata "Perimeter"
         * @param poly 
         */
        void plotPerimeter(shared_ptr<CGAL::Polygon_2<K>> poly);


        /**
         * @brief stampa un sottopoligono numerato "num" , in nero su un'immagine denominata con decomposition_name
         * @param poly poligono da stampare 
         * @param points vertici 
         * @param num numero del poligono nell'ordinamento 
         * @param decomposition_name nome della decomposizione
         */
        void plotSubPolygon(const Polygon& poly,  vector<K::Point_2>& points, int num, string decomposition_name);

        /**
         * @brief stampa il path di un singolo poligono sull'immagine della decomposizione 
         * 
         */
        void plotPathForConvexPolygon(vector<CGAL::Segment_2<K>> path);

        /**
         * @brief stampa il path finale su un'immagine "FinalPath" evidenziando i punti in cui viene diviso e il punto di partenza
         * @param path 
         * @param pointsToPrint 
         * @param start 
         */
        void plotFinalPath(vector<CGAL::Segment_2<K>> path, vector<K::Point_2> pointsToPrint,  K::Point_2 start);

        /**
         * @brief stampa su "FinalPath" una parte del path finale 
         * @param path 
         */
        void plotPartialPath(vector<CGAL::Segment_2<K>> path);


        /**
         * @brief stampa un perimetro sull'immagine "Perimeter" e aggiorna m_perimeterImage
         * @param new_poly 
         */
        void updatePerimeterImage(shared_ptr<CGAL::Polygon_2<K>> new_poly);

        /**
         * @brief stampa su "FinalPath" in blu un singolo punto 
         * @param point 
         */
        void plotPoint(K::Point_2 point); 

        /**
         * @brief calcola il valore in pixel dati x metri in base a m_resolution 
         * 
         * @param x 
         * @return float 
         */
        float pixelFromMetres (float x);

        /**
         * @brief calcola la risoluzione da usare per l'immagine 
         * @param perimeter_vertices vertici del poligono da stampare 
         */
        void calculateResolution(vector<K::Point_2>& perimeter_vertices);

    private: 

        /**
         * @brief immagine del perimetro iniziale
         * 
         */
        cv::Mat m_perimeterImage;

        /**
         * @brief immagine della decomposizione 
         * 
         */
        cv::Mat m_image_decomposition;

        /**
         * @brief nome della decomposizione
         * 
         */
        string m_decompositionName;

        /**
         * @brief rapporto pixel/metri
         * 
         */
        float m_resolution; 
        
};
