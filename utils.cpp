#include "utils.hpp"
using namespace std; 

vector<Point> readFromFile() {
    string filename("../input.txt");
    vector<string> lines;
    string line;
    ifstream input_file(filename);
    vector<Point> points;
    if (!input_file.is_open()) {
        cerr << "Could not open the file -'" << filename <<"'"<< endl;
        exit(-1);
    }
    while (getline(input_file, line)) {
        lines.push_back(line);
    }
    string delimiter = ",";
    string token;
    string tmp;
    float x;
    float y;
    size_t pos;
    for (int i = 0; i < lines.size(); i++) {
        tmp = lines.at(i);
        pos = tmp.find(delimiter);
        token = tmp.substr(0,pos); //primo pezzo
        x = stof(token);
        tmp.erase(0, delimiter.size() + token.size()) ;
        y = stof(tmp);
        Point p (x,y);
        points.push_back(p);
    }
    input_file.close();
    return points;
}

void printPoint(Point p) {
    cout << p.hx() << "," << p.hy() << endl;
}

Polygon createPolygon(vector<Point> points) {
    size_t sz = points.size();
    Point array[sz]; 
    for (int i = 0; i < sz; i++) array[i] = points.at(i); 
    Polygon poly(array, array+sz );
    return poly;
}

void plotPoints(string name, vector<Point> inPoints,  cv::Mat image) {
    vector<cv::Point> points;
    cout << "num points : " << inPoints.size() << endl;;
    for (int i = 0; i < inPoints.size(); i++) {
        cv::Point p (inPoints.at(i).hx(), inPoints.at(i).hy());
        //cout << inPoints.at(i).hx() << " " << inPoints.at(i).hy() << endl;
        cv::circle(image,p,5,cv::Scalar(255, 0, 0), 1, 8, 0 );
    }
    cv::imshow(name, image);
    cv::waitKey(0);  
}

cv::Mat plotPolygon(string name, Polygon p, float resolution){
    //creating a blank image with white background
    cv::Mat image(500,500, CV_8UC3, cv::Scalar(255,255,255));
    if (image.empty()) cout << "Could not open or find the image" <<endl;
    //points di cgal ==> points di opencv
    vector<Point> inPoints;
    for(const Point& pt : p.vertices()) inPoints.push_back(pt);
    //credo points di opencv 
    cv::Point p_old;
    cv::Point first;
    cv::Point last;
    for (int i = 0; i < inPoints.size(); i++) {
        cv::Point p (inPoints.at(i).hx(), inPoints.at(i).hy());
        if (i == 0) {
            p_old = p;
            first = p;
        }
        if (i == inPoints.size()-1 ) last = p;
        cv::line(image, p_old, p, cv::Scalar(115, 44, 83), 1, 8, 0);
        p_old = p;
    }
    cv::line(image, last, first, cv::Scalar(115, 44, 83), 1, 8, 0);
    cv::namedWindow(name, 1);
    cv::imshow(name, image);
    cv::waitKey(0);    
    return image; 
}


vector<Point> extractVertices(Polygon poly) {
    vector<Point> points;
    //estraggo i vertici
    for(const Point& p : poly.vertices()) points.push_back(p); //dovrebbero essere in ordine? 
    return points;
}