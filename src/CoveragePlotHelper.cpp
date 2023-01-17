#include "CoveragePlotHelper.h"

CoveragePlotHelper::CoveragePlotHelper(){

}

/*************************************/

CoveragePlotHelper::~CoveragePlotHelper(){


}

/*************************************/


bool CoveragePlotHelper::init(vector<K::Point_2>& perimeter_vertices){

    m_perimeterImage = cv::Mat(1000,1000, CV_8UC3, cv::Scalar(255,255,255));
    m_image_decomposition = cv::Mat(1000,1000, CV_8UC3, cv::Scalar(255,255,255));

    calculateResolution(perimeter_vertices);

    if (m_perimeterImage.empty() || m_image_decomposition.empty()) {
        cout << "CoveragePlothHelper: Could not open or find the image" << endl;
        return false;
    }

    return true;

}


/***********************************/

float CoveragePlotHelper::pixelFromMetres (float x) {
    return x*m_resolution;
}



/***********************************/

//resolution non è il termine giusto
void CoveragePlotHelper::calculateResolution(vector<K::Point_2>& perimeter_vertices) {

    float x_max = perimeter_vertices.at(0).hx();
    float y_max = perimeter_vertices.at(0).hy();
    for (size_t i = 0; i < perimeter_vertices.size(); i++ ){
        if (perimeter_vertices.at(i).hx() > x_max) {
            x_max = perimeter_vertices.at(i).hx();
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


void CoveragePlotHelper::plotPerimeter(shared_ptr<CGAL::Polygon_2<K>> poly) {
    
    //trasformo i point di cgal in points di opencv
    vector<K::Point_2> points;
    for(const K::Point_2& pt : poly->vertices()) points.push_back(pt);
 
    
    //creo points di opencv ==> trasformando da metri a pixel
    cv::Point p_old;
    cv::Point first;
    cv::Point last;
    for (size_t i = 0; i < points.size(); i++) {
        cv::Point point ( pixelFromMetres(points.at(i).hx()) , pixelFromMetres(points.at(i).hy()) );
        if (i == 0) {
            p_old = point;
            first = point;
        }
        if (i == points.size()-1 ) last = point;
        cv::line(m_perimeterImage, p_old, point, cv::Scalar(0,0,0) , 2, 8, 0);
        // cv::putText(m_perimeterImage, std::to_string(i) ,point , cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 255),2);
           
        p_old = point;
    }

    cv::line(m_perimeterImage, last, first, cv::Scalar(0,0,0), 2, 8, 0);
    
    cv::namedWindow("Perimeter", 1);
    cv::imshow("Perimeter", m_perimeterImage);
    cv::waitKey(0);    

}

/***********************************/
//stampa il sottopoligono e un numero che indica la sua posizione nell'ordinamento 
void CoveragePlotHelper::plotSubPolygon(const Polygon& poly,  vector<K::Point_2>& points, int num, string decomposition_name) {
    
    cout << "CoveragePlotHelper: plotSubPolygon" << endl;
    cv::Point p_old;
    cv::Point first;
    cv::Point last;
    size_t cont = 0;
    m_decompositionName = decomposition_name;
    size_t sz = poly.container().size();
    cv::Point p_label;
    for (Point p: poly.container()) {
        cv::Point point ( pixelFromMetres(points[p].hx()), pixelFromMetres(points[p].hy()) );
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
    cv::putText(m_image_decomposition, std::to_string(num) , p_label, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 255),2);

    cv::namedWindow(m_decompositionName, 1); 
    cv::imshow(m_decompositionName, m_image_decomposition);
    cv::waitKey(0);    
    
}




/***********************************/

void CoveragePlotHelper::plotPathForConvexPolygon(vector<CGAL::Segment_2<K>> path) {
    
    cout << "CoveragePlotHelper: plotPathForConvexPolygon" << endl;
    size_t k = 0; 
    while (k < path.size()) {
        K::Point_2 p = path.at(k).source(); 
        K::Point_2 q = path.at(k).target();
        cv::line(m_image_decomposition, cv::Point( pixelFromMetres(p.hx()), pixelFromMetres(p.hy()) ) , 
            cv::Point(pixelFromMetres(q.hx()), pixelFromMetres(q.hy()) ), cv::Scalar(0, 0, 255), 1, 8, 0);
        k++;
    }
    cv::namedWindow(m_decompositionName, 1);    
    cv::imshow(m_decompositionName, m_image_decomposition);
    cv::waitKey(0);    

}

/***********************************/
void CoveragePlotHelper::plotFinalPath(vector<CGAL::Segment_2<K>> path, vector<K::Point_2> pointsToPrint, K::Point_2 start) {
    
    cout << "CoveragePlotHelper: plotFinalPath" << endl;
    for (size_t i = 0; i < path.size(); i++) {
        K::Point_2 p = path.at(i).source();
        K::Point_2 q = path.at(i).target(); 
        cv::line(m_perimeterImage, cv::Point( pixelFromMetres(p.hx()), pixelFromMetres(p.hy()) ) , 
            cv::Point(pixelFromMetres(q.hx()), pixelFromMetres(q.hy()) ), cv::Scalar(0, 0, 255), 1, 8, 0);
    }

    //stampa i punti 
    for (size_t i = 0; i < pointsToPrint.size(); i++) {
        if (pointsToPrint.at(i) == start) {
            cv::circle(m_perimeterImage, cv::Point(pixelFromMetres(pointsToPrint.at(i).hx()) ,  pixelFromMetres(pointsToPrint.at(i).hy())),
            3, cv::Scalar(255,0,0), 3 ); 
        }
        else {
            cv::circle(m_perimeterImage, cv::Point(pixelFromMetres(pointsToPrint.at(i).hx()) ,  pixelFromMetres(pointsToPrint.at(i).hy())),
                2, cv::Scalar(0,255,0), 1 ); 
        }
    }   
    cv::namedWindow("FinalPath", 1);    
    cv::imshow("FinalPath" , m_perimeterImage);
    cv::waitKey(0);    

}

/***********************************/
void CoveragePlotHelper::plotPartialPath(vector<CGAL::Segment_2<K>> path) {

    cout<< "CoveragePlotHelper: plotPartialPath"<< endl;
    
    for (size_t i = 0; i < path.size(); i++) {
        K::Point_2 p = path.at(i).source();
        K::Point_2 q = path.at(i).target(); 
        cv::line(m_perimeterImage, cv::Point( pixelFromMetres(p.hx()), pixelFromMetres(p.hy()) ) , 
            cv::Point(pixelFromMetres(q.hx()), pixelFromMetres(q.hy()) ), cv::Scalar(255,0,255) , 1, 8, 0);
    }

    cv::namedWindow("FinalPath", 1);    
    cv::imshow("FinalPath" , m_perimeterImage);
    cv::waitKey(0);  


}


/***********************************/

//stampa il nuovo perimetro e aggiorna il perimetro da disegnare
void CoveragePlotHelper::updatePerimeterImage(shared_ptr<CGAL::Polygon_2<K>> new_poly) {
    
    cv::Mat new_image(1000,1000, CV_8UC3, cv::Scalar(255,255,255)); 
    m_perimeterImage = new_image; 
    plotPerimeter(new_poly); 

}

/***********************************/

void CoveragePlotHelper::plotPoint(K::Point_2 point) {
    cout << "CoveragePlotHelper: plotPoint" << endl;
    cv::circle(m_perimeterImage, cv::Point(pixelFromMetres(point.hx()), pixelFromMetres(point.hy())) , 2, cv::Scalar(255,0,0), 2 ); 
    cv::namedWindow("FinalPath", 1);    
    cv::imshow("FinalPath" , m_perimeterImage);
    cv::waitKey(0);    
}

/***********************************/
void CoveragePlotHelper::plotLineForTest(CGAL::Line_2<K> line) {
    double a = line.a(); 
    double b = line.b(); 
    double c = line.c(); 
    
    cv::line(m_perimeterImage, cv::Point(pixelFromMetres(0), pixelFromMetres(-(c/b))), cv::Point(pixelFromMetres(-(c/a)), pixelFromMetres(0)),   
        cv::Scalar(0, 0, 255), 1, 8, 0   );
    cv::namedWindow("FinalPath", 1);    
    cv::imshow("FinalPath" , m_perimeterImage);
    cv::waitKey(0);    
}