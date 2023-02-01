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
         * @brief stampa in nero perimetro di un poligono su sfondo bianco
         * 
         * @param poly 
         * @param imageName nome dell'immagine
         * @param printIndexes se è true, vengono stampati gli indici dei vertici
         */
        void plotPerimeter(shared_ptr<CGAL::Polygon_2<K>> poly, string imageName, bool printIndexes);


        /**
         * @brief stampa un sottopoligono numerato "num" , in nero su un'immagine denominata con decomposition_name
         * @param poly poligono da stampare 
         * @param points vertici 
         * @param putText se true, vengono aggiunti i nomi 
         * @param polName nome da stampare sul poligono 
         * @param decomposition_name nome della decomposizione
         */
        void plotSubPolygon(const Polygon& poly,  vector<K::Point_2>& points, string decomposition_name, bool putText, string polName);

        /**
         * @brief 
         * 
         * @param path 
         */
        void plotPathForConvexPolygon(vector<CGAL::Segment_2<K>> path);

        /**
         * @brief 
         * 
         * @param path 
         * @param pointsToPrint 
         * @param start 
         * @param sweepDistance 
         */
        void plotFinalPath(vector<CGAL::Segment_2<K>> path, vector<K::Point_2> pointsToPrint,  K::Point_2 start, float sweepDistance);

        /**
         * @brief stampa su "FinalPath" una parte del path finale 
         * @param path 
         * @param cont se != -1 stampa cont in corrispondenza dell'ultimo punto del path 
         */
        void plotPartialPath(vector<CGAL::Segment_2<K>> path, int cont);


        /**
         * @brief stampa un perimetro sull'immagine "Perimeter" e aggiorna m_perimeterImage
         * @param new_poly 
         */
        void updatePerimeterImage(shared_ptr<CGAL::Polygon_2<K>> new_poly, bool printIndexes);

        /**
         * @brief stampa su "FinalPath" in blu un singolo punto 
         * @param point 
         * @param cont se cont != -1 , stampa il numero in corrispondenza del punto 
         */
        void plotPoint(K::Point_2 point, char color, int cont); 

        void plotLineForTest(CGAL::Line_2<K> line, string imageName);

        void plotPerimeterForTest(shared_ptr<CGAL::Polygon_2<K>> poly, string imageName, bool printIndexes);

        void clearAllImages(); 


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
         * @brief 
         * 
         */
        cv::Mat m_testImage; 

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


        
};
