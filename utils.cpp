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


void plotPathForConvexPolygon(vector<K::Point_2> grid , shared_ptr<CGAL::Polygon_2<K>> poly, cv::Mat image, const string name, float resolution) {
    
    plotPolygon(image, name ,poly,resolution );
    int k = 0; 
    while (k < grid.size()) {
        K::Point_2 p = grid.at(k); 
        K::Point_2 q = grid.at(k+1);
        cv::line(image, cv::Point(p.hx(), p.hy()), cv::Point(q.hx(), q.hy()), cv::Scalar(0, 0, 255), 1, 8, 0);
        k = k+2;    
    }
    cv::namedWindow(name, 1);    
    cv::imshow(name , image);
    cv::waitKey(0);    
}

// vector<Point> extractVertices(shared_ptr<Polygon> poly) {
//     vector<Point> points;
//     //estraggo i vertici
//     for(const Point& p : poly->vertices()) points.push_back(p); //dovrebbero essere in ordine? 
//     return points;
// }