#pragma once 

#include <stdlib.h>
#include <vector>
#include <string>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/partition_2.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/property_map.h>
#include <CGAL/intersections.h>
#include <CGAL/Polyline_simplification_2/simplify.h>
#include "utils.hpp"
#include "CoveragePlotHelper.h"
#include "MyDecomposition.h"

namespace PS = CGAL::Polyline_simplification_2;
typedef PS::Stop_above_cost_threshold Stop;


class CoveragePathCreator {
    public:
    
      CoveragePathCreator();

      ~CoveragePathCreator();

      /**
       * @brief
       * 
       * @param points, coppie di coordinate dei punti del perimetro iniziale su cui fare coverage
       * @param sweepDistance, distanza tra le strisciate (raggio del veicolo)
       * @param decompositionType, tipo di decomposizione da usare 
       * @return true 
       * @return false 
       */
      bool init( vector<pair<float,float>> points, float sweepDistance, int decompositionType);

      bool run();

      /**
       * @brief Restituisce il percorso finale in punti <float,float> a distanza m_rangeToReturn
       * 
       * @return vector<pair<float,float>> 
       */
      vector<pair<float,float>> getFinalPath();
      

      /**
       * @brief setta m_addPerimeterToPath. Se è true, la prima parte del path è l'intero perimetro
       * 
       * @param b 
       */
      void setAddPerimeterToPath(bool b); 
      
      /**
       * @brief se range>0  setta m_rangeToReturn
       * 
       * @param range 
       */
      void setRangeToReturn(float range); 

      /**
       * @brief Set the Plot Restricted Polygons object
       * 
       * @param b 
       */
      void setPlotRestrictedPolygons(bool b);

      /**
       * @brief Set the Reduce Subpolygons object
       * 
       * @param b 
       */
      void setReduceSubpolygons(bool b); 

      /**
       * @brief Set the Plot Index Of Path object
       * 
       * @param b 
       */
      void setPlotIndexOfPath(bool b);

    private:
  
        /**
         * @brief distanza tra i punti che compongono il path finale 
         * 
         */
        float m_rangeToReturn; 

        /**
         * @brief vertici del perimetro iniziale 
         * 
         */
        vector<K::Point_2> m_initialPerimeterVertices; 

        /**
         * @brief vertici del perimetro dopo la semplificazione 
         * 
         */
        vector<K::Point_2> m_simplyfiedVertices;

        /**
         * @brief vertici del perimetro dopo la decomposizione (possono essere di più)
         * 
         */
        vector<K::Point_2> m_decomposedVertices;

        /**
         * @brief poligono iniziale (perimetro)
         * 
         */
        shared_ptr<CGAL::Polygon_2<K>> m_initialPolygon;

        /**
         * @brief poligono dopo la semplificazione (perimetro)
         * 
         */
        shared_ptr<CGAL::Polygon_2<K>> m_simplyfiedPolygon;

        /**
         * @brief lista di sottopoligoni di indici dopo la decomposizione (non ordinati)
         * 
         */
        Polygon_list m_decomposedPolysOfIndices;

        /**
         * @brief numero di sottopoligoni in cui è stato decomposto il perimetro iniziale 
         * 
         */
        size_t m_numberOfDecomposedSubPolygons; 
         
         /**
         * @brief matrice N*N*2 di adiacenza dei sottopoligoni in ordine di decomposizione (non per il path). m_adj[i][j][0] e m_adj[i][j][1] sono i due vertici dell'adiacenza 
         * tra i e j. Se sono entrambi -1 i e j non sono adiacenti. 
         * 
         */
        vector<vector<vector<int>>> m_adj; 

        /**
         * @brief matrice N*N. Alla posizione (i,j) c'è il costo minimo del tragitto tra il sottopoligono i e il sottopoligono j .
         * Costo =  numero di sottopoligoni da attraversare  
         * (non è ordinata)
         */
        vector<vector<float>> m_adjWeigthMatrix; 

        /**
         * @brief intersezioni tra le griglia e i sottopoligoni. m_intersections[i][j] è la j-esima intersezione tra la i-esima griglia e l'i-esimo sottopoligono
         * 
         * m_intersections è ordinato rispetto all'ordine di percorrenza 
         */
        vector<vector<K::Point_2>> m_intersections;

        /**
         * @brief vector di polygoni per cui costruire il path, ordinati secondo tsp
         * 
         */
        vector<shared_ptr<CGAL::Polygon_2<K>>> m_ordinatedSubpolygonsForPath; 

        /**
         * @brief m_polygonsSorted[i] è il numero del polygono da attraversare come i-esimo nell'ordinamento   
         * 
         */
        vector<int> m_polygonsSorted; 

 
        /**
         * @brief unione ordinata del perimetro + percorso punto iniziale -> inizio primo path +  tutti i path dei sottopoligoni + i loro collegamenti
         * ordinata nell'ordine di percorrenza 
         * 
         */
        vector<CGAL::Segment_2<K>> m_finalPath;
  
        /**
         * @brief path per il return composto da punti a distanza 0.2 
         * 
         */
        vector<K::Point_2> m_pathToReturn;

        /**
         * @brief true se all'inizio del path finale si vuole inserire il perimetro iniziale
         * 
         */
        bool m_addPerimeterToPath;

        /**
         * @brief 
         * 
         */
        bool m_plotRestrictedPolygons;

        /**
         * @brief 
         * 
         */
        bool m_reduceSubpolygons; 

        /**
         * @brief true se si vogliono stampare indici alla fine di ogni parte del path (per seguire il percorso)
         * 
         */
        bool m_plotIndexOfPath;

        /**
         * @brief tipo di decomposizione del perimetro iniziale 
         * 
         */
        int m_decompositionType;

        /**
         * @brief nome della decomposizione 
         * 
         */
        string m_decompositionName;


        /**
         * @brief true se si vuole plottare il path e le varie operazioni 
         * 
         */
        bool m_doPlotting;

        /**
         * @brief distanza tra le strisciate (raggio del veicolo)
         * 
         */
        float m_sweepDistance;

        /**
         * @brief classe per fare tutti i plot 
         * 
         */
        CoveragePlotHelper m_Helper;

        /**
         * @brief 
         * 
         */
        MyDecomposition m_decomposer; 

        /**
         * @brief vertice iniziale del path (primo vertice dato in input)
         * 
         */
        K::Point_2 m_startingPoint;

        /**
         * @brief vertice del poligono decomposto preso come punto di partenza ( il più vicino allo startingPoint)  
         * 
         */
        K::Point_2 m_firstVertex;



        /**
         * @brief decomoposizione del perimetro inziale
         * 
         * @return true -> se viene decomposto correttamente 
         * @return false altrimenti 
         */
        bool decompose();

        /**
         * @brief crea la matrice di adiacenza dei sottopoligoni m_adj
         * 
         */
        void createAdjMatrix();

        /**
         * @brief ordina i sottopoligoni per la percorrenza (chiama findMinRoute(int start))
         * 
         * @return true 
         * @return false in caso di errori 
         */
        bool orderSubPolygons();

        /**
         * @brief restituisce il punto a massima distanza da un segmento e il valore della distanza 
         * 
         * @param points insieme di punti 
         * @param segment segmento 
         * @return tuple<float, K::Point_2> 
         */
        tuple<float, K::Point_2> maxDistance(vector<K::Point_2>& points, K::Segment_2& segment); 

        /**
         * @brief restituisce un segmento la cui direzione è quella di sweep e il punto più lontano 
         * 
         * @param polygon 
         * @return tuple<CGAL::Segment_2<K>, K::Point_2> 
         */
        tuple<CGAL::Segment_2<K>, K::Point_2> findSweepDirection(shared_ptr<CGAL::Polygon_2<K>> polygon);

        /**
         * @brief crea la griglia di rette parallele a sweepDirection
         * @param parallelEdge lato del poligono parallelo alla direzione di spazzata  
         * @param point punto più lontano dal segmento 
         * @return vector<CGAL::Line_2<K>> 
         */
        vector<CGAL::Line_2<K>> createGrid(CGAL::Segment_2<K> parallelEdge, K::Point_2 point);

         /**
         * @brief 
         * 
         * @param polygon 
         * @param isEdgeToReduce 
         * @param adjacences 
         */
        void areEdgesToReduce(shared_ptr<CGAL::Polygon_2<K>> polygon, vector<bool> &isEdgeToReduce, vector<CGAL::Segment_2<K>> adjacences);


        /**
         * @brief ritorna i punti di intersezione tra la griglia e il poligono ristretto nelle adiacenze
         * @param cont numero del sottopoligono
         * @param borders borders[i] == true se il lato i del sottopoligono cont ha un'adiacenza e va ristretta
         * @return vector<CGAL::Segment_2<K>> 
         */
        vector<K::Point_2> generateGridForOnePolygon(shared_ptr<CGAL::Polygon_2<K>> polygon , vector<bool>& borders);

        /**
         * @brief genera il path per un sottopoligono 
         * @param intersections punti di intersezione della griglia con il sottopoligono (eventualmente ristretto) ==> sono i punti delle "curve" del path
         * @param start punto di partenza del path
         * @return vector<CGAL::Segment_2<K>> 
         */
        vector<CGAL::Segment_2<K>> generatePathForOnePolygon(vector<K::Point_2> intersections, int start);

        /**
         * @brief inserisce in m_finalPath il collegamento tra la fine del path creato fino a quel momento e l'inizio del path di indexedPolygon
         *  , trovando di conseguenza il punto iniziale del nuvoo pezzo di path (uno dei 4 possibili) 
         * @param indexPolygon 
         * @param indexOfLastPolygonCovered
         * @return int, indice di inizio per la costruzione del nuovo path 
         */
        int generateLinkBetween(size_t indexPolygon, size_t indexOfLastPolygonCovered, int &cont); 


        /**
         * @brief trova la strada più breve a partire da start per toccare tutti i sottopoligoni 
         * @param start sottopoligono di partenza 
         * @return vector<int> 
         */
        vector<int> findMinRouteTo(int start);


        /**
         * @brief trova la strada più breve per andare dal sottopoligono start ad end
         * @param start partenza 
         * @param end arrivo 
         * @return vector<int> ordinato : alla posizione 0 c'è start , alla 1 il successivo , e così via fino ad end 
         */
        vector<int> findMinRouteBetween(int start, int end);

        /**
         * @brief restituisce l'indice i del minimo valore di dist t.c. !visited[i]
         * ==> restituisce l'indice del poligono non ancora visitato a distanza minima dalla sorgente
         * @param dist dist[i] è la distanza del sottopoligono i dallo start dell'algoritmo 
         * @param visited visited[i] == true se è già stato visitato il sottopoligono i
         * @return int 
         */
        int indexOfMinimum(vector<float>& dist, vector<bool>& visited);

        /**
         * @brief restituisce l'indice corrispondente a quale dei 4 parametri è minore. 
         * ==> da quale dei 4 punti possibili di partenza del path di un sottopoligono partire per creare il path 
         * @param a distanza tra la fine del path precedente e il primo punto possibile di partenza del path successivo  
         * @param b ...
         * @param c ...
         * @param d ...
         * @return int 
         */
        int initialIndex(float a, float b, float c, float d);  

        /**
         * @brief restituisce il numero di adiacenze del sottopoligono node 
         * @param node 
         * @return int 
         */
        int numAdiacency(int node);

        /**  
         * @brief inserisce in distances la minima distanza per andare da sorg a ogni altro nodo
         * @param graph matrice di adiacenza classica. 1 se i e j sono , 0 altrimenti  
         * @param sorg nodo start 
         * @param distances 
         */
        void Dijkstra(vector<vector<int>> &graph, int sorg , vector<float>& distances);

        /**
         * @brief riempie m_AdjWeightMatrix[i][j] con il costo del cammino da i a j, numero di sottopoligoni di distanza 
         * 
         */
        void createAdjWeigthMatrix();

        /**
         * @brief genera per ogni sottopoligono il suo path e li inserisce in m_paths.
         * m_paths[i] è il path del sottopoligono i , dove i è la sua posizione nell'ordine di percorrenza. 
         * 
         */
        void generatePathForSubpolygons();

   
        /**
         * @brief inizia l'algoritmo di copertura del perimetro iniziale, chiamando poi tutte le altre funzioni 
         * 
         */
        void cover();


        /**
         * @brief crea il path composto da punti <float, float>
         * 
         */
        void createPathToReturn(); 


        /**
         * @brief Riduce i sottopoligoni in corrispondenza delle adiacenze. 
         * crea per ogni sottopoligono una griglia composta dai punti che saranno le "curve" del path
         * 
         */
        void generateGridsForSubpolygons();

        /**
         * @brief approssima il perimetro iniziale riducendone i vertici a seconda della sweepDistance
         * 
         */
        void simplifyPerimeter();

     
        /**
         * @brief riduce un sottopoligono secondo il vector borders
         * 
         * @param polygon 
         * @param edges 
         * @return shared_ptr<CGAL::Polygon_2<K>> 
         */
        shared_ptr<CGAL::Polygon_2<K>> reduceSubPolygon(shared_ptr<CGAL::Polygon_2<K>> polygon, vector<bool> &edges );

        /**
         * @brief 
         * 
         * @param polygon
         * @param isAdjacent 
         * @return void
         */
        void eliminateExcessPoints(shared_ptr<CGAL::Polygon_2<K>>  polygon, vector<bool> &isAdjacent);
       
};
