#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/partition_2.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/property_map.h>
#include <vector>
#include <cassert>
#include <string.h>
#include <list>
#include <iterator>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/draw_polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "utils.hpp"
#include "path_generator.hpp"

//LINEA DI COMANDO: <tipocomposizione> <sweepDistance> 


using namespace std;

float sweep_distance;
string decomposition_name;

//funziona solo con la decomposizione 0
//SISTEMARE I RIFERIMENTI -> SHARED_PTR??
//SE IL RANGE È GRANDE E NON CI STA NEANCHE UNA STRISCIATA DÀ PROBLEMI 
//AGGIUNGERE UNA TOLLERANZA PER CUI SCEGLIERE IL LATO PIÙ LUNGO ANCHE SE LA SUA ALTEZZA È LEGGERMENTE MAGGIORE 
//star.txt non va 

int main(int argc, char* argv[]) {
    
    //argomenti linea di comando
    if (argc < 4) { 
        cout << "Argument Error" << endl;
        printInfo();
        return 1;
    }
    string filename = argv[1];
    sweep_distance = stof(argv[3]);
    

    //leggo input 
    vector<K::Point_2> points = readFromFile(filename);
    shared_ptr<CGAL::Polygon_2<K>> initial_polygon = createPolygon(points);
    
    //immagine su cui plottare
    cv::Mat initial_image(1000,1000, CV_8UC3, cv::Scalar(255,255,255));
    cv::Mat image_decomposition(1000,1000, CV_8UC3, cv::Scalar(255,255,255));
    cv::Mat image_path(1000,1000, CV_8UC3, cv::Scalar(255,255,255));

    if (initial_image.empty() || image_decomposition.empty() || image_path.empty()) {
        cout << "Could not open or find the image" << endl;
    }


    //calcolo rapporto pixel/metri
    float resolution = calculateresolution(points);

    //plot poligono iniziale
    plotPolygon(initial_image, "Initial Polygon", initial_polygon, resolution);

    //decomposizione
    Partition_traits_2 traits(CGAL::make_property_map(points));
    Polygon polygon;
    for (int i = 0; i < points.size(); i++) {
        polygon.push_back(i);
    }
    Polygon_list partition_polys;
  
    switch (stoi(argv[2]))
    {
    case 0:
        CGAL::optimal_convex_partition_2(polygon.vertices_begin(),
                                 polygon.vertices_end(),
                                 back_inserter(partition_polys), 
                                 traits);
        decomposition_name = "Optimal convex partition";
        break;
    case 1: 
        CGAL::y_monotone_partition_2(polygon.vertices_begin(),
                                 polygon.vertices_end(),
                                 back_inserter(partition_polys), 
                                 traits);
        decomposition_name = "Monotone partition";
        break;
    case 2:
        CGAL::approx_convex_partition_2(polygon.vertices_begin(),
                                 polygon.vertices_end(),
                                 back_inserter(partition_polys), 
                                 traits);
        decomposition_name = "Approx convex partition";
        break;
    case 3:
        cout << "green approx convex partition" << endl;
        CGAL::greene_approx_convex_partition_2(polygon.vertices_begin(),
                                 polygon.vertices_end(),
                                 back_inserter(partition_polys), 
                                 traits);
        break;
    default:
        cout << "Argument Error" << endl;
        return 1;
        break;
    }

    //controllo decomposizione
    assert(CGAL::partition_is_valid_2(polygon.vertices_begin(),
                                        polygon.vertices_end(),
                                        partition_polys.begin(),
                                        partition_polys.end(),
                                        traits));





    //plot dei sottopoligoni con openCV
    int k = 0;
    for (const Polygon& poly : partition_polys){
        k++;
        plotSubPolygon(image_decomposition,decomposition_name, poly, points, resolution) ;
    }
    cv::waitKey(0);



    //generazione path per ogni poligono
    vector<vector<CGAL::Segment_2<K>>> grids; 
    // vector<vector<K::Point_2>> grids;
    vector<shared_ptr<CGAL::Polygon_2<K>>> polygons_for_path;



    //COSTRUZIONE MATRICE DI ADIACENZA
    k = partition_polys.size(); 
    //le adiacenze sono rappresentate da una coppia di indici di punti perché suppongo che tutti i poligoni siano convessi 
    vector<vector<vector<int>>> adj;
    adj.resize(k);
    for (int i = 0; i < k; i++) {
        adj.at(i).resize(k);
        for (int j = 0; j < k; j++) {
            adj.at(i).at(j).resize(2);
        }
    }

    int cont_i = 0;
    int cont_j = 0;

    for (const Polygon& poly1 : partition_polys){ 
        for (const Polygon& poly2 : partition_polys){
            if (cont_i == cont_j) { 
                adj[cont_i][cont_j][0] = -1;
                adj[cont_i][cont_j][1] = -1;
            }

            else {
                int vert_i, vert_j;
                adjacency (poly1.container(), poly2.container(), vert_i, vert_j);
                adj[cont_i][cont_j][0] = vert_i;
                adj[cont_i][cont_j][1] = vert_j;
            }
            cont_j++;
        }
        cont_i++;
        cont_j = 0;
    }


    vector<int> polygonSorted = sortPolygons(adj); //polygons in ordine di visita

   




    // //stampo ordine poligoni ==> per adesso ordina semplicemente in base alle distanze minime, non va bene 
    // cout << "Ordine poligoni nel path: " << endl;
    // for (int i = 0; i < polygonSorted.size(); i++ ) {
    //     cout << polygonSorted.at(i) << endl;
    // }

  



//    // MATRICE ADJ OK !
//     cout << "MATRICE ADIACENZA" << endl;
//     for (int i = 0; i < adj.size(); i++) {

//         cout << "Poligono " << i << " :" << endl;
//         for (int j = 0; j < adj.size(); j++) {
//             if (adj[i][j][0] != -1  && adj[i][j][1] != -1 ) {
//                 cout << "adiacenza con poligono " << j << " in " << adj[i][j][0] << " , " << adj[i][j][1] << endl;
//             }
//             else if (adj[i][j][0] != -1  && adj[i][j][1] == -1) {
//                 cout << "adiacenza con poligono " << j << " in " << adj[i][j][0] <<  endl;
//             }
//             else if (adj[i][j][0] == -1  && adj[i][j][1] != -1) {
//                 cout << "adiacenza con poligono " << j << " in " << adj[i][j][1] <<  endl;
//             }
//             else {
//                 cout << "Nessuna adiacenza con poligono " << j << endl;
//             }
//         } 
//     }

    



    int cont = 0;
    for (const Polygon& poly : partition_polys) { 
        
        vector<K::Point_2> tmp;
        for (Point p: poly.container()) {
            tmp.push_back(K::Point_2 (points[p].hx(), points[p].hy()));
        }
        polygons_for_path.push_back(createPolygon(tmp));

        //borders indica quali lati sono in comune e quindi bisogna lasciare lo spazio , inizializzo tutto a 0
        vector<bool> borders; 
        borders.resize(polygons_for_path.at(cont)->edges().size(),false);       

       //metto a 1 i lati che hanno adiacenza
        for ( int i = 0; i < adj[cont].size(); i++) {
            if (adj[cont][i][0] != -1 && adj[cont][i][1] != -1) {
                
                CGAL::Segment_2<K> seg (points[adj[cont][i][0]],  points[adj[cont][i][1]]);

                for (int j = 0; j < polygons_for_path.at(cont)->edges().size(); j++ ) {

                    if ( (seg.source() == polygons_for_path.at(cont)->edge(j).source() && seg.target() == polygons_for_path.at(cont)->edge(j).target() )
                    || ( seg.source() == polygons_for_path.at(cont)->edge(j).target() && seg.target() == polygons_for_path.at(cont)->edge(j).source())  ) {
                        borders[j] = 1;
                    }
                }
            }
        }

        grids.push_back(generatePathForOnePolygon(polygons_for_path.at(cont), sweep_distance, borders )); 
        plotPathForConvexPolygon(grids.at(cont), polygons_for_path.at(cont), image_path, "path", resolution);
        cont++;

    }

    cv::waitKey(0);


    return 0;

}