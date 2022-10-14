#include "decomposer.hpp"
using namespace std; 

vector<Point> findCriticalPoints(vector<shared_ptr<Polygon>> polygons){ //i punti critici saranno tutti i vertici degli ostacoli + i vertici concavi del perimetro
    vector<Point> criticals,tmp;
    for (int i = 0; i< polygons.size(); i++) {
        if (i==0 && polygons.at(i)->is_convex()) continue; //Non ci sono punti critici nel perimetro
        //se non lo è non funziona 
        //per ora considero di essere già al primo interno 
        tmp = extractVertices(polygons.at(i));
        criticals.insert(criticals.end(), tmp.begin(), tmp.end());
    }
    return criticals;
}

// vector<shared_ptr<Polygon>> decompose(vector<shared_ptr<Polygon>> polygons, vector<Point> criticalPoints) {
//     shared_ptr<Polygon> tmp;
//     for (int i = 0; i< polygons.size(); i++) {
//         tmp = polygons.at(i);
//         if (i==0 && tmp->is_convex()) continue; //Non ci sono punti critici nel perimetro
//         //per ogni coppia di punti del poligono => segmento (sto assumendo che siano ordinati)
//         vector<Point> vertices = extractVertices(tmp);
//         for (int j = 0; j < vertices.size(); j++) {
//             if (j == 0) {
//                 Line l(vertices.at(j), vertices.at(j+1));
//                 //intersezione tra l e il poligono

//                 const auto = intersection(l, tmp); 
//                 // printPoint(i);



                
//             }
//             if (j == vertices.size()-1) {

//             }
//         }
//     }
// }

    // vector<Point> points = extractVertices(poly);
    // vector<Point> p1;
    // vector<Point> p2;
    // for (int i = 0; i < points.size()/2; i++)   p1.push_back(points.at(i));
    // for (int i = points.size()/2; i < points.size(); i++) p2.push_back(points.at(i));
    // vector<shared_ptr<Polygon>> polygons;
    // polygons.push_back(createPolygon(p1));
    // polygons.push_back(createPolygon(p2));
    // return polygons;




// int main (int argc, char** argv) { 

//     vector<Point> points = readFromFile();
//     // cout << "Punti letti:" << endl;
//     // for (int i = 0; i < points.size(); i++) printPoint(points[i]);
//     Polygon poly = createPolygon(points);
//     cv::Mat initial = plotPolygon("Poligono Iniziale", poly,resolution);
//     vector<Polygon> decomposition = decompose(poly);
//     vector<cv::Mat> images; 
    
//     // plot images
//     for (int i = 0; i < decomposition.size(); i++) {
//         images.push_back(plotPolygon( ( "Poligono "+to_string(i)),decomposition.at(i),resolution));
//     }

//     vector<vector<Point>> paths;
//     for (int i = 0 ; i < decomposition.size(); i++) {
//             cout << "Generate path for area "<< i << endl;
//             vector <Point> path_i;
//             generatePath(decomposition.at(i),path_i);
//             // cout << "punti che compongono il path: " << endl;
//             // for (int i = 0; i < path_i.size(); i++) printPoint(path_i.at(i));
//             paths.push_back(path_i);
//     }

//     //bisogna disegnare i punti sul poligono
//     //avrò 2 immagini e 2 path corrispondenti 
//     for (int i = 0; i < paths.size(); i++) {
//         plotPoints("Poligono "+ to_string(i), paths.at(i), images.at(i));
//     }
//     return 0;
// }




    // cv::Mat initial = plotPolygon("Poligono Iniziale", poly,resolution);
    // vector<Polygon> decomposition = decompose(poly);
    // vector<cv::Mat> images; 
    
    // // plot images
    // for (int i = 0; i < decomposition.size(); i++) {
    //     images.push_back(plotPolygon( ( "Poligono "+to_string(i)),decomposition.at(i),resolution));
    // }

    // vector<vector<Point>> paths;
    // for (int i = 0 ; i < decomposition.size(); i++) {
    //         cout << "Generate path for area "<< i << endl;
    //         vector <Point> path_i;
    //         generatePath(decomposition.at(i),path_i);
    //         // cout << "punti che compongono il path: " << endl;
    //         // for (int i = 0; i < path_i.size(); i++) printPoint(path_i.at(i));
    //         paths.push_back(path_i);
    // }

    // //bisogna disegnare i punti sul poligono
    // //avrò 2 immagini e 2 path corrispondenti 
    // for (int i = 0; i < paths.size(); i++) {
    //     plotPoints("Poligono "+ to_string(i), paths.at(i), images.at(i));
    // }
    // return 0;
// }
