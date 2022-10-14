#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/partition_2.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/property_map.h>
#include <vector>
#include <cassert>
#include <string.h>
#include <list>
#include <CGAL/draw_polygon_2.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "utils.hpp"
#include "path_generator.hpp"

using namespace std;
// string decomposition;
#define resolution (0.05)
#define range (10)

//SISTEMARE I RIFERIMENTI -> SHARED_PTR??
//SE IL RANGE È GRANDE E NON CI STA NEANCHE UNA STRISCIATA DÀ PROBLEMI 
//AGGIUNGERE IL NOME DELLA DECOMPOSIZIONE ALL'IMMAGINE 



int main(int argc, char* argv[]) {
    
    //argomenti
    if (argc < 3) { 
        cout << "Argument Error" << endl;
        printInfo();
        return 1;
    }
    string filename = argv[1];
    

    
    //leggo input 
    vector<K::Point_2> points = readFromFile(filename);
    shared_ptr<CGAL::Polygon_2<K>> initial_polygon = createPolygon(points);
    
    //immagine su cui plottare
    cv::Mat initial_image(500,500, CV_8UC3, cv::Scalar(255,255,255));
    cv::Mat image_decomposition(500,500, CV_8UC3, cv::Scalar(255,255,255));
    cv::Mat image_path(500,500, CV_8UC3, cv::Scalar(255,255,255));

    if (initial_image.empty() || image_decomposition.empty() || image_path.empty()) {
        cout << "Could not open or find the image" << endl;
    }

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
        cout << "optimal convex partition" << endl;
        CGAL::optimal_convex_partition_2(polygon.vertices_begin(),
                                 polygon.vertices_end(),
                                 back_inserter(partition_polys), 
                                 traits);
        break;
    case 1: 
        cout << "monotone partition" << endl;
        CGAL::y_monotone_partition_2(polygon.vertices_begin(),
                                 polygon.vertices_end(),
                                 back_inserter(partition_polys), 
                                 traits);
        break;
    case 2:
        cout << "approx convex partition" << endl;
        CGAL::approx_convex_partition_2(polygon.vertices_begin(),
                                 polygon.vertices_end(),
                                 back_inserter(partition_polys), 
                                 traits);
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
        plotSubPolygon(image_decomposition,"decomposizione", poly, points, resolution) ;
    }
    cv::waitKey(0);


    //generazione path per ogni poligono
    vector<vector<K::Point_2>> grids;
    vector<shared_ptr<CGAL::Polygon_2<K>>> polygons_for_path;
    int cont = 0;

    for (const Polygon& poly : partition_polys){ 
        
        vector<K::Point_2> tmp;
        for (Point p: poly.container()) {
            tmp.push_back(K::Point_2 (points[p].hx(), points[p].hy()));
        }
        polygons_for_path.push_back(createPolygon(tmp));
        grids.push_back(generatePath(polygons_for_path.at(cont), range)); //a due a due sono i punti da collegare per creare la griglia 
        plotPathForConvexPolygon(grids.at(cont), polygons_for_path.at(cont), image_path, "path", resolution);
        cont++;

    }

    cv::waitKey(0);


    return 0;
}