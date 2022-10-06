#include <iostream>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <math.h>
#include <CGAL/Point_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Simple_cartesian.h>
#include "utils.hpp"
#include "generatorPath.hpp"

//come fare in modo che il make si possa rifare anche quando viene modificato utils.hpp e non solo decomposer.cpp?
//readFromFile chiama invalid argument se in input.txt metti una riga vuota in più
//come faccio a fare tutti i controlli?

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point;
typedef CGAL::Polygon_2<K> Polygon;

using namespace std;
#define resolution (0.05)


vector<Polygon> decompose(Polygon poly) {
    vector<Point> points = extractVertices(poly);
    vector<Point> p1;
    vector<Point> p2;
    for (int i = 0; i < points.size()/2; i++)   p1.push_back(points.at(i));
    for (int i = points.size()/2; i < points.size(); i++) p2.push_back(points.at(i));
    vector<Polygon> polygons;
    polygons.push_back(createPolygon(p1));
    polygons.push_back(createPolygon(p2));
    return polygons;
}

int main (int argc, char** argv) { 

    vector<Point> points = readFromFile();
    // cout << "Punti letti:" << endl;
    // for (int i = 0; i < points.size(); i++) printPoint(points[i]);
    Polygon poly = createPolygon(points);
    cv::Mat initial = plotPolygon("Poligono Iniziale", poly,resolution);
    vector<Polygon> decomposition = decompose(poly);
    vector<cv::Mat> images; 
    
    // plot images
    for (int i = 0; i < decomposition.size(); i++) {
        images.push_back(plotPolygon( ( "Poligono "+to_string(i)),decomposition.at(i),resolution));
    }

    vector<vector<Point>> paths;
    for (int i = 0 ; i < decomposition.size(); i++) {
            cout << "Generate path for area "<< i << endl;
            vector <Point> path_i;
            generatePath(decomposition.at(i),path_i);
            // cout << "punti che compongono il path: " << endl;
            // for (int i = 0; i < path_i.size(); i++) printPoint(path_i.at(i));
            paths.push_back(path_i);
    }

    //bisogna disegnare i punti sul poligono
    //avrò 2 immagini e 2 path corrispondenti 
    for (int i = 0; i < paths.size(); i++) {
        plotPoints("Poligono "+ to_string(i), paths.at(i), images.at(i));
    }
    return 0;
}
