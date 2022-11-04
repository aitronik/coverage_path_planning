#include "utils.hpp"


using namespace std; 

vector<K::Point_2> readFromFile(string name){
    string filename("../" + name);
    ifstream input_file(filename);
    vector<string> lines;
    string line;
    vector<vector<K::Point_2>> perimeters; //il primo sarà il perimetro esterno, gli altri gli ostacoli 
    if (!input_file.is_open()) {
        cerr << "Could not open the file -'" << filename <<"'"<< endl;
        exit(-1);
    }
    while (getline(input_file, line)) {
        lines.push_back(line);
    }
    string token, tmp;
    string delimiter = ",";
    size_t pos;
    vector<K::Point_2> tmp_polygon;
    int cont = 0;
    for (int i = 0; i < lines.size(); i++) {
        tmp = lines.at(i);
        if (tmp.compare("--") == 0) {
            cont ++;
            if (cont != 1) perimeters.push_back(tmp_polygon);
            tmp_polygon.clear();
            continue;
        }
        pos = tmp.find(delimiter);
        token = tmp.substr(0,pos); //primo pezzo
        tmp.erase(0, delimiter.size() + token.size()) ;
        K::Point_2 p (stof(token),stof(tmp));
        tmp_polygon.push_back(p);
    }   
    input_file.close();
    return perimeters.at(0);
}

// template <class InputIterator, class OutputIterator, class Traits>
// inline
// OutputIterator optimal_convex_partition_2(InputIterator first,
//                                           InputIterator beyond,
//                                           OutputIterator result,
//                                           const Traits& traits)
// {
//    return partition_optimal_convex_2(first, beyond, result, traits);
// }

void printPoint(K::Point_2 p) {
    cout << p.hx() << "," << p.hy() << endl;
}

shared_ptr<CGAL::Polygon_2<K>> createPolygon(vector<K::Point_2> points) {
    size_t sz = points.size();
    K::Point_2 array[sz]; 
    for (int i = 0; i < sz; i++) array[i] = points.at(i); 
    CGAL::Polygon_2<K> p (array, array+sz);
    shared_ptr<CGAL::Polygon_2<K>> poly = make_shared<CGAL::Polygon_2<K>>(p);
    return poly;
}


// void plotPoints(const string& name, const vector<Point>& points,  cv::Mat image) {
//     vector<cv::Point> points;
//     cout << "num points : " << points.size() << endl;;
//     for (int i = 0; i < points.size(); i++) {
//         cv::Point p (points.at(i).hx(), points.at(i).hy());
//         //cout << points.at(i).hx() << " " << points.at(i).hy() << endl;
//         cv::circle(image,p,5,cv::Scalar(255, 0, 0), 1, 8, 0 );
//     }
//     cv::imshow(name, image);
//     cv::waitKey(0);  
// }

void plotSubPolygon(cv::Mat image, const string name, /*shared_ptr<Polygon>& p*/ Polygon poly,  vector<K::Point_2> points, float resolution){ //sulla stessa immagine se sono più di uno
   
    if (image.empty()) {
        cout << "Could not open or find the image" <<endl;
    }

    cv::Point p_old;
    cv::Point first;
    cv::Point last;
    int cont = 0;
    size_t sz = poly.container().size();
    for (Point p: poly.container()) {
        cv::Point point (points[p].hx(), points[p].hy());
        if (cont == 0) {
            p_old = point;
            first = point;
        }
        if (cont == sz -1 ) {
            last = point;
        }
        cv::line(image, p_old, point, cv::Scalar(115, 44, 83), 1, 8, 0);
        p_old = point;
        cont++;
    }

    cv::line(image, last, first, cv::Scalar(115, 44, 83), 1, 8, 0);
    cv::namedWindow(name, 1);
    cv::imshow(name, image);
    cv::waitKey(0);    
}

void plotPolygon(cv::Mat image, const string name, shared_ptr<CGAL::Polygon_2<K>> poly, float resolution){ 
    //creating a blank image with white background
    if (image.empty()) cout << "Could not open or find the image" <<endl;
    //points di cgal ==> points di opencv
    vector<K::Point_2> points;
    for(const K::Point_2& pt : poly->vertices()) points.push_back(pt);
    //creo points di opencv 
    cv::Point p_old;
    cv::Point first;
    cv::Point last;
    for (int i = 0; i < points.size(); i++) {
        cv::Point point (points.at(i).hx(), points.at(i).hy());
        if (i == 0) {
            p_old = point;
            first = point;
        }
        if (i == points.size()-1 ) last = point;
        cv::line(image, p_old, point, cv::Scalar(115, 44, 83), 1, 8, 0);
        p_old = point;
    }
    cv::line(image, last, first, cv::Scalar(115, 44, 83), 1, 8, 0);
    cv::namedWindow(name, 1);
    cv::imshow(name, image);
    cv::waitKey(0);    
}

void printInfo() {
    cout << "0 -> optimal convex partition" << endl;
    cout << "1 -> monotone partition" << endl;
    cout << "2 -> approx convex partition" << endl;
    cout << "3 -> greene approx convex partition" << endl;
}


void plotPathForConvexPolygon(vector<CGAL::Segment_2<K>> grid , shared_ptr<CGAL::Polygon_2<K>> poly, cv::Mat image, const string name, float resolution) {
    
    plotPolygon(image, name ,poly,resolution );
    int k = 0; 
    while (k < grid.size()) {
        K::Point_2 p = grid.at(k).source(); 
        K::Point_2 q = grid.at(k).target();
        cv::line(image, cv::Point(p.hx(), p.hy()), cv::Point(q.hx(), q.hy()), cv::Scalar(0, 0, 255), 1, 8, 0);
        k++;
    }
    cv::namedWindow(name, 1);    
    cv::imshow(name , image);
    cv::waitKey(0);    
}


//suppongo che i punti in comune non possano essere più di due
bool adjacency(list<size_t> container1, list<size_t> container2, int& vertex_i, int& vertex_j ) {
    int adj[2] = {-1,-1};
    int cont = 0;
    // per ogni lista di vertici di un poligono
    for (size_t p: container1) {
        // guardi tutti i vertici
        for (size_t q:container2) {
            // se sono uguali
            if (p == q) {
                adj[cont] = p;
                cont++;
            }
        }
    }
    vertex_i = adj[0];
    vertex_j = adj[1];
    // vertex_i and j can be -1 if there is no adjacency
    // or have a value containing the vertex in which they are adjacent
    // return true if the two polygons have at least one vertex in common
    return (vertex_i != -1);
}


double calculateAngle (CGAL::Vector_2<K> v, CGAL::Vector_2<K> w) {


    double theta = CGAL::scalar_product(v,w);
    double len1, len2;
    len1 = sqrt(v.squared_length());
    len2 = sqrt(w.squared_length());
    return theta/ (len1*len2);
}
