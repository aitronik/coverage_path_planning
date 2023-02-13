#include "CoveragePlotHelper.h"

//rivedere la conversione metri-pixel, se i metri sono tanti tanti non va ? 


CoveragePlotHelper::CoveragePlotHelper(){

}

/*************************************/

CoveragePlotHelper::~CoveragePlotHelper(){


}

/*************************************/


bool CoveragePlotHelper::init(vector<K::Point_2>& perimeter_vertices){
#ifdef ENABLE_OPENCV
    m_perimeterImage = cv::Mat(1000,1000, CV_8UC3, cv::Scalar(255,255,255));
    m_image_decomposition = cv::Mat(1000,1000, CV_8UC3, cv::Scalar(255,255,255));
    m_testImage = cv::Mat(1000, 1000, CV_8UC3, cv::Scalar(255,255,255));

    calculateResolution(perimeter_vertices);

    if (m_perimeterImage.empty() || m_image_decomposition.empty() || m_testImage.empty() ) {
        cout << "CoveragePlothHelper: Could not open or find the image" << endl;
        return false;
    }
#endif
    return true;

}


/***********************************/

float CoveragePlotHelper::pixelFromMetres (float x) {
    return x*m_resolution;
}



/***********************************/

//resolution non è il termine giusto
void CoveragePlotHelper::calculateResolution(vector<K::Point_2>& perimeter_vertices) {

    float x_max = perimeter_vertices.at(0).x();
    float y_max = perimeter_vertices.at(0).hy();
    for (size_t i = 0; i < perimeter_vertices.size(); i++ ){
        if (perimeter_vertices.at(i).x() > x_max) {
            x_max = perimeter_vertices.at(i).x();
        }
        if (perimeter_vertices.at(i).hy() > y_max) {
            y_max = perimeter_vertices.at(i).hy();
        }
    }
    if (y_max >= x_max) {
        m_resolution = 980/y_max;
    }
    else {
        m_resolution = 980/x_max;
    }
}



/***********************************/


void CoveragePlotHelper::plotPerimeter(shared_ptr<CGAL::Polygon_2<K>> poly, string imageName, bool printIndexes) {
#ifdef ENABLE_OPENCV
    //trasformo i point di cgal in points di opencv
    vector<K::Point_2> points;
    for(const K::Point_2& pt : poly->vertices()) points.push_back(pt);
    
    
    //creo points di opencv ==> trasformando da metri a pixel
    cv::Point p_old;
    cv::Point first;
    cv::Point last;
    for (size_t i = 0; i < points.size(); i++) {
        cv::Point point ( pixelFromMetres(points.at(i).x()) , pixelFromMetres(points.at(i).hy()) );
        if (i == 0) {
            p_old = point;
            first = point;
        }
        if (i == points.size()-1 ) last = point;
        cv::line(m_perimeterImage, p_old, point, cv::Scalar(0,0,0) , 2, 8, 0);
        
        if (printIndexes)  {
            cv::putText(m_perimeterImage, std::to_string(i) ,point , cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 255),2);
        }
           
        p_old = point;
    }

    cv::line(m_perimeterImage, last, first, cv::Scalar(0,0,0), 2, 8, 0);
    
    cv::namedWindow(imageName, 1);
    cv::imshow(imageName, m_perimeterImage);
    cv::waitKey(0);    
#endif
}

/***********************************/

void CoveragePlotHelper::plotPerimeterForTest(shared_ptr<CGAL::Polygon_2<K>> poly, string imageName, bool printIndexes) {
#ifdef ENABLE_OPENCV
    //trasformo i point di cgal in points di opencv
    vector<K::Point_2> points;
    for(const K::Point_2& pt : poly->vertices()) points.push_back(pt);
 
    
    //creo points di opencv ==> trasformando da metri a pixel
    cv::Point p_old;
    cv::Point first;
    cv::Point last;
    for (size_t i = 0; i < points.size(); i++) {
        cv::Point point ( pixelFromMetres(points.at(i).x()) , pixelFromMetres(points.at(i).hy()) );
        if (i == 0) {
            p_old = point;
            first = point;
        }
        if (i == points.size()-1 ) last = point;
        cv::line(m_testImage, p_old, point, cv::Scalar(0,0,0) , 2, 8, 0);           
        p_old = point;
        if (printIndexes)  {
            cv::putText(m_testImage, std::to_string(i) ,point , cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 255),2);
        }
    }

    cv::line(m_testImage, last, first, cv::Scalar(0,0,0), 2, 8, 0);
    
    cv::namedWindow(imageName, 1);
    cv::imshow(imageName, m_testImage);
    cv::waitKey(0);    
#endif
}

/***********************************/
//stampa il sottopoligono e un numero che indica la sua posizione nell'ordinamento 
void CoveragePlotHelper::plotSubPolygon(const Polygon& poly,  vector<K::Point_2>& points,  string decomposition_name, bool putPolygonText, string polName, bool putVertexText) {
#ifdef ENABLE_OPENCV
    // cout << "CoveragePlotHelper: plotSubPolygon" << endl;
    cv::Point p_old;
    cv::Point first;
    cv::Point last;
    size_t cont = 0;
    m_decompositionName = decomposition_name;
    size_t sz = poly.container().size();
    cv::Point p_label;
    // int vertexIndex = 0; 
    for (Point p: poly.container()) {
        cv::Point point ( pixelFromMetres(points[p].x()), pixelFromMetres(points[p].hy()) );
        if (putVertexText) {
            cv::putText(m_image_decomposition, to_string(p), point, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 255), 2);
            // vertexIndex++; 
        }
        p_label += point;
        if (cont == 0) {
            p_old = point;
            first = point;
        }
        if (cont == sz -1 ) {
            last = point;
        }
        cv::line(m_image_decomposition, p_old, point, cv::Scalar(0,0,0), 2, 8, 0);

        p_old = point;
        cont++;
    }

    p_label /= (int)cont;

    cv::line(m_image_decomposition, last, first, cv::Scalar(0,0,0), 2, 8, 0); 
    if (putPolygonText) {
        cv::putText(m_image_decomposition, polName , p_label, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 255),2);
    }
    cv::namedWindow(m_decompositionName, 1); 
    cv::imshow(m_decompositionName, m_image_decomposition);
    cv::waitKey(0);    
#endif
}




/***********************************/

void CoveragePlotHelper::plotPathForConvexPolygon(vector<CGAL::Segment_2<K>> path) {
#ifdef ENABLE_OPENCV
    // cout << "CoveragePlotHelper: plotPathForConvexPolygon" << endl;
    size_t k = 0; 

    while (k < path.size()) {
        K::Point_2 p = path.at(k).source(); 
        K::Point_2 q = path.at(k).target();
        cv::line(m_image_decomposition, cv::Point( pixelFromMetres(p.x()), pixelFromMetres(p.hy()) ) , 
            cv::Point(pixelFromMetres(q.x()), pixelFromMetres(q.hy()) ), cv::Scalar(0, 0, 255), 1, 8, 0);
        k++;
    }
    cv::namedWindow(m_decompositionName, 1);    
    cv::imshow(m_decompositionName, m_image_decomposition);
    cv::waitKey(0);    
#endif
}

/***********************************/
void CoveragePlotHelper::plotFinalPath(vector<CGAL::Segment_2<K>> path, vector<K::Point_2> pointsToPrint, K::Point_2 start, float sweepDistance) {
#ifdef ENABLE_OPENCV
    // cout << "CoveragePlotHelper: plotFinalPath" << endl;
    for (size_t i = 0; i < path.size(); i++) {
        K::Point_2 p = path.at(i).source();
        K::Point_2 q = path.at(i).target(); 
        if (p != q){
            cv::line(m_perimeterImage, cv::Point( pixelFromMetres(p.x()), pixelFromMetres(p.hy()) ) , 
                cv::Point(pixelFromMetres(q.x()), pixelFromMetres(q.hy()) ), cv::Scalar(255,20,147), 1, 8, 0); 
        }
    }

    //stampo spessore robot 
    for (size_t i = 0; i < path.size(); i++) {
        K::Point_2 p = path.at(i).source();
        K::Point_2 q = path.at(i).target();
        if (p != q){
            cv::line(m_perimeterImage, cv::Point( pixelFromMetres(p.x()), pixelFromMetres(p.hy()) ) , 
                cv::Point(pixelFromMetres(q.x()), pixelFromMetres(q.hy()) ), cv::Scalar(250,235,215), pixelFromMetres(sweepDistance), 8, 0);
        }
    }


    //stampa i punti 
    for (size_t i = 0; i < pointsToPrint.size(); i++) {
        if (pointsToPrint.at(i) == start) {
            cv::circle(m_perimeterImage, cv::Point(pixelFromMetres(pointsToPrint.at(i).x()) ,  pixelFromMetres(pointsToPrint.at(i).hy())),
            2, cv::Scalar(255,0,0), 3 ); 
        }
    }   
    // cv::namedWindow("FinalPath", 1);    
    cv::imshow("FinalPath" , m_perimeterImage);
    cv::waitKey(0);    
#endif
}

/***********************************/
void CoveragePlotHelper::plotPartialPath(vector<CGAL::Segment_2<K>> path, int cont) {
#ifdef ENABLE_OPENCV

    // cout<< "CoveragePlotHelper: plotPartialPath"<< endl;

    cv::Point last( pixelFromMetres(path.at(path.size()-1).target().x()), pixelFromMetres(path.at(path.size()-1).target().hy())) ; 

    for (size_t i = 0; i < path.size(); i++) {
        K::Point_2 p = path.at(i).source();
        K::Point_2 q = path.at(i).target(); 
        cv::line(m_perimeterImage, cv::Point( pixelFromMetres(p.x()), pixelFromMetres(p.hy()) ) , 
            cv::Point(pixelFromMetres(q.x()), pixelFromMetres(q.hy()) ), cv::Scalar(0,0,255) , 1.5, 8, 0);
    }

    if (cont != -1) {
        cv::putText(m_perimeterImage, to_string(cont), last,  cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 2 );
    }

    cv::namedWindow("FinalPath", 1);    
    cv::imshow("FinalPath" , m_perimeterImage);
    cv::waitKey(0);  

#endif
}


/***********************************/

//stampa il nuovo perimetro e aggiorna il perimetro da disegnare
void CoveragePlotHelper::updatePerimeterImage(shared_ptr<CGAL::Polygon_2<K>> new_poly, bool printIndexes) {
#ifdef ENABLE_OPENCV
    cv::Mat new_image(1000,1000, CV_8UC3, cv::Scalar(255,255,255)); 
    m_perimeterImage = new_image; 
    plotPerimeter(new_poly, "Simplifyed Perimeter", printIndexes); 
#endif
}

/***********************************/

void CoveragePlotHelper::plotPoint(K::Point_2 point, char color, int cont) {
#ifdef ENABLE_OPENCV
    // cout << "CoveragePlotHelper: plotPoint" << endl;
    cv::Scalar c;
    if (color == 'g') {
        c = cv::Scalar(0,255,0); //giallo
    }
    else if (color == 'r'){
        c = cv::Scalar(0,0,255);
    }
    else {
        c = cv::Scalar(255,0,0);
    }
    cv::Point p(pixelFromMetres(point.x()), pixelFromMetres(point.hy())); 
    cv::circle(m_perimeterImage, p , 2, c, 2 ); 

    if (cont != -1) {
        cv::putText(m_perimeterImage, to_string(cont), p,  cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 2 );
    }

    cv::namedWindow("FinalPath", 1);    
    cv::imshow("FinalPath" , m_perimeterImage);
    cv::waitKey(0);  
#endif  
}

/***********************************/
void CoveragePlotHelper::plotLineForTest(CGAL::Line_2<K> line, string imageName) {
#ifdef ENABLE_OPENCV
    cout << "CoveragePlotHelper: plotLineForTest() " << endl; 
    double a = line.a(); 
    double b = line.b(); 
    double c = line.c(); 

    if (a == 0 ) {
         cv::line(m_image_decomposition, cv::Point(pixelFromMetres(0), pixelFromMetres(-(c/b))), cv::Point(pixelFromMetres(1), pixelFromMetres(-(c/b))),   
        cv::Scalar(0, 0, 255), 1, 8, 0   );
    }
    else if (b == 0) {
         cv::line(m_image_decomposition, cv::Point(pixelFromMetres(-(c/a)), pixelFromMetres(0)), cv::Point(pixelFromMetres(-(c/a)), pixelFromMetres(1)),   
        cv::Scalar(0, 0, 255), 1, 8, 0   );
    }
    else {
        cv::line(m_image_decomposition, cv::Point(pixelFromMetres(0), pixelFromMetres(-(c/b))), cv::Point(pixelFromMetres(-(c/a)), pixelFromMetres(0)),   
            cv::Scalar(0, 0, 255), 1, 8, 0   );
    }
    cv::namedWindow(imageName, 1);    
    cv::imshow(imageName , m_testImage);
    cv::waitKey(0);  
#endif
}

/***********************************/
void CoveragePlotHelper::clearAllImages() {
#ifdef ENABLE_OPENCV
    m_perimeterImage = cv::Mat(1000,1000, CV_8UC3, cv::Scalar(255,255,255));
    m_image_decomposition = cv::Mat(1000,1000, CV_8UC3, cv::Scalar(255,255,255));
    m_testImage = cv::Mat(1000,1000, CV_8UC3, cv::Scalar(255,255,255));
#endif
}

/***********************************/
// void plotSubPolygons(Polygon_list decomposition, vector<K::Point_2>& vertices, bool putVertexText){
    
//     for (const Polygon &poly : decomposition) {
//         for (const &p : poly.container() ) {

//         }
//     }
// }
