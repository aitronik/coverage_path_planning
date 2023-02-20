#include "utils.hpp"

using namespace std; 

vector<pair<float,float>> readFromFile(string name){
    string filename(name);
    ifstream input_file(filename);
    vector<string> lines;
    string line;
    vector<vector<pair<float,float>>> perimeters; 
    // vector<vector<K::Point_2>> perimeters; //il primo sarà il perimetro esterno, gli altri gli ostacoli 
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
    // vector<K::Point_2> tmp_poygon; 
    vector<pair<float,float>> tmp_polygon;
    int cont = 0;
    for (size_t i = 0; i < lines.size(); i++) {
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
        pair<float,float> p(stof(token),stof(tmp)); 
        // K::Point_2 p (stof(token),stof(tmp));
        tmp_polygon.push_back(p);
    }   
    input_file.close();
    return perimeters.at(0); //se metto gli ostacoli bisognerà aggiungere
}


/*************************************/

shared_ptr<CGAL::Polygon_2<K>> createPolygon(vector<K::Point_2> points) {
    size_t sz = points.size(); 
    CGAL::Polygon_2<K> polygon(points.begin(), points.begin()+sz); 
    return make_shared<CGAL::Polygon_2<K>>(polygon); 

}

/*************************************/
//suppongo che i punti in comune non possano essere più di due
//ritorna true se a[0] != -1 
//a[0] == -1 && a[1] == -1 se non c'è adiacenza 
//a[0] != -1 && a[1] == -1 se c'è un solo punto di adiacenza 
//altrimenti a[0] e a[1] sono i due estremi del segmento di adiacenza tra i due poligoni 

bool adjacency(list<size_t> container1, list<size_t> container2, int& vertex_i, int& vertex_j, vector<K::Point_2> decomposedVertices) {
    
    int adj[2] = {-1, -1}; //punti di adiacenza (estremi del segmento)
    int cont = 0; //contatore di quanti punti sono stati trovati (da 0 a 2)
    
    // //controllo tramite la lista di indici dei poligoni che ci sia almeno un vertice in comune 
    // for (size_t p : container1) {
    //     for (size_t q : container2) {
    //         //se sono uguali 
    //         if (p == q && cont < 2) {
    //             adj[cont] = p; 
    //             cont ++; 
    //             break;
    //         } 
    //     }
    // }

    //se è solo uno, provo a calcolarne un secondo (che potrebbe non essere in comune come vertice, ma appartenere comunque anche 
    //a un lato del poligono di cui non è vertice) 
    

    size_t old, first; 
    int k; 

    //prima cerco a partire da polygon1
    for (size_t p : container1) {
        if (cont == 2) {
            break;
        }
        k = 0;
        //controllo se è parte di uno dei lati di polygon2
        for (size_t q : container2) {
            if (k == 0) { // sono al primo vertice
                old = q;
                first = q;
            }

            if (p == q && cont < 2) {
            adj[cont] = p; 
            cont ++; 
            break;
            }


            if (k == container2.size()-1) { //sono all'ultimo vertice
                if (p != adj[0] && CGAL::squared_distance(CGAL::Segment_2<K>(decomposedVertices[q],decomposedVertices[first] ), decomposedVertices[p]) <= 0.001) {
                    adj[cont] = p;
                    cont ++;    
                    break;
                }
            }
            if (p != adj[0] && CGAL::squared_distance(CGAL::Segment_2<K>(decomposedVertices[old],decomposedVertices[q] ), decomposedVertices[p]) <= 0.001) {
                adj[cont] = p;
                cont ++;    
                break;
            }
            k++;
            old = q;
        }
    }

    //al contrario
    for (size_t p : container2) {
        
        if (cont == 2) {
            break; 
        }
        k = 0;
        for (size_t q : container1) {
            
            if (k == 0) {
                first = q;
                old = q;
            }

            // if (p == q && cont < 2) {
            //     adj[cont] = p; 
            //     cont ++; 
            //     break;
            // }

            if (k == container1.size()-1) { //sono all'ultimo vertice
                if (p != adj[0] && CGAL::squared_distance(CGAL::Segment_2<K>(decomposedVertices[q],decomposedVertices[first] ), decomposedVertices[p]) <= 0.001) {
                    adj[cont] = p;
                    cont ++;    
                    break;
                }
            }
            if (p != adj[0] && CGAL::squared_distance(CGAL::Segment_2<K>(decomposedVertices[old],decomposedVertices[q] ), decomposedVertices[p]) <= 0.001) {
                adj[cont] = p;
                cont ++;    
                break;
            }
            k++;
            old = q;
        }
    } 
    
    
    
    vertex_i = adj[0];
    vertex_j = adj[1];

    return (vertex_i != -1);
}




/*************************************/
//suppongo che i punti in comune non possano essere più di due
// bool adjacency(list<size_t> container1, list<size_t> container2, int& vertex_i, int& vertex_j ) {
//     int adj[2] = {-1,-1};
//     int cont = 0;
//     // per ogni lista di vertici di un poligono
//     for (size_t p: container1) {
//         // guardi tutti i vertici
//         for (size_t q:container2) {
//             // se sono uguali
//             if (p == q && cont < 2) {
//                 adj[cont] = p;
//                 cont++;
//                 break;
//             }
//         }
//     }
//     vertex_i = adj[0];
//     vertex_j = adj[1];
//     // vertex_i and j can be -1 if there is no adjacency
//     // or have a value containing the vertex in which they are adjacent
//     // return true if the two polygons have at least one vertex in common
//     return (vertex_i != -1);
    
// }

/*************************************/
bool isPointIntoConvexPolygon(shared_ptr<CGAL::Polygon_2<K>> polygon, K::Point_2 p, float approx) {
    bool to_return = false; 
    //cerco le massime e minime x,y del polygon 
    double x_max = 0; 
    double y_max = 0;
    double x_min = INT_MAX; 
    double y_min = INT_MAX; 

    //calcolo xmax, ymax, xmin, ymin 
    for (size_t i = 0; i < polygon->vertices().size(); i++) {
        K::Point_2 corr = polygon->vertex(i);
        if (corr.x() >= x_max) {
            x_max = corr.x(); 
        }
        if (corr.x() <= x_min) {
            x_min = corr.x(); 
        }
        if (corr.y() >= y_max) {
            y_max = corr.y();
        }
        if (corr.y() <= y_min) {
            y_min = corr.y();
        }
    }

    if (p.x() >= x_min-approx && p.x() <= x_max+approx) {
        if (p.y() >= y_min-approx && p.y() <= y_max+approx) {
            to_return = true; 
        }
    }

    return to_return; 

}


/*************************************/ 

//il secondo elemento del pair è l'indice del lato  in cui c'è l'intersezione
pair<K::Point_2,int> intersect_concave_polygon_at_index(shared_ptr<CGAL::Polygon_2<K>> polygon, size_t edgeIndex, size_t vertexIndex) {

    CGAL::Line_2<K> line = polygon->edge(edgeIndex).supporting_line();
    size_t N = polygon->edges().size(); 
    
    vector<int> indexIntersections;
    vector<K::Point_2> points;
    for (size_t i = 0 ; i < polygon->edges().size() ;  i++) {
        // non confrontarlo con te stesso, precedente e successivo
        if ( i != edgeIndex && i != (edgeIndex+1)%(N) &&  i != (edgeIndex-1+N)%(N) ) {
            const auto inter = CGAL::intersection(line, polygon->edge(i));
        
            if (inter){
                if (const CGAL::Segment_2<K>* s = boost::get<CGAL::Segment_2<K>>(&*inter) ) { //se si intersecano in un segmento niente , è il lato stesso 
                cout << "trovato intersezione con segmento " << endl;
                }
                else if (const K::Point_2* p = boost::get<K::Point_2>(&*inter)){  //se si intersecano in un punto 
                    points.push_back(*p);
                    indexIntersections.push_back(i); 
                }
            }
        }
    }

    K::Point_2 vertex =  polygon->vertex(vertexIndex);
    K::Point_2 other; // l'altro estremo (es. target del lato se il vertex ne è il source)
    int index_intersection = -1;
    float minDistance = MAXFLOAT;  
    K::Point_2 closestPoint(-1,-1);

    if (vertexIndex == edgeIndex) { //il vertex è il source dell'edge 
        other = polygon->edge(edgeIndex).target(); 
    }
    else { // il vertex è il target 
        other = polygon->edge(edgeIndex).source();
    }
   
    for (size_t i = 0; i < points.size(); i++){
        // prendo il più vicino al vertex ma che sia più vicino al vertex rispetto all'altro estremo del lato
        const float dist = CGAL::squared_distance(points[i], vertex);
        if ( (dist < minDistance) && (dist < CGAL::squared_distance(points[i], other)) ) {
            closestPoint = points[i]; 
            index_intersection = indexIntersections[i]; 
            minDistance = dist;
        }
    
    }

    return make_pair(closestPoint, index_intersection);
}

/*************************************/
vector<K::Point_2> intersect_lines(CGAL::Line_2<K> line1, CGAL::Line_2<K> line2, float approximation) {
    //se sono la stessa retta restituisce due punti p(-1,-1)
    //se sono parallele e non sono la stessa retta restituisce p(-1,-1)
    //altrimenti restituisce il punto di intersezione 
    vector<K::Point_2> to_return; 
    double Px, Py;

    double a1 = line1.a(); 
    double b1 = line1.b(); 
    double c1 = line1.c(); 
    double a2 = line2.a(); 
    double b2 = line2.b();
    double c2 = line2.c();  

    double k = (( a2 * b1) - (a1 * b2)); 
    if (k != 0) { 
        if (b1 != 0) {
            Px = ( (b2*c1) - (b1*c2) ) /k; 
            Py =  ( (a1*c2) - (a2*c1) )/ k; 
        }
        else { //a1 e b2 sono sicuramente != 0 perché k != 0 
            Px = -(c1/a1); 
            Py = ( (a2*c1) - (a1*c2)) / (a1*b2); 
        }
        to_return.push_back(K::Point_2(Px,Py)); 
    } 
    else { //sono parallele 
        if (CGAL::squared_distance(line1, line2) <= approximation) { //coincidenti
            to_return.push_back(K::Point_2(-1,-1)); 
            to_return.push_back(K::Point_2(-1,-1)); 
        }
        else {
            //quindi non si intersecano
            to_return.push_back(K::Point_2(-1,-1));
        }
    }
    return to_return;
}
/*************************************/

bool areParallel(CGAL::Line_2<K> l1, CGAL::Line_2<K> l2, float approx) {
    
    bool to_ret = false; 
    vector<K::Point_2> inters = intersect_lines(l1,l2,approx); 
    if (inters[0] == K::Point_2(-1,-1)) {
        to_ret = true;
    }
    return to_ret; 

}


/*************************************/
//si suppone che polygon sia convesso
vector<K::Point_2> intersect_convex_polygon_line(shared_ptr<CGAL::Polygon_2<K>> polygon, CGAL::Line_2<K> line, float approx) {
    
    vector<K::Point_2> a; 
    size_t N = polygon->edges().size(); 

    for (size_t i = 0; i < N ; i++) {


        vector<K::Point_2> inters = intersect_lines(line, polygon->edge(i).supporting_line(), approx); 
        
        if (a.size() < 2) { //se non ho ancora trovato due punti di intersezione

            if (inters.size() == 2) { //coincidenti ==> l'intersezione è il lato stesso 
                a.clear(); //se ne avevo già trovato uno lo rimuovo 
                a.push_back(polygon->edge(i).source()); 
                a.push_back(polygon->edge(i).target());
            }

            else if (inters[0] != K::Point_2(-1,-1)) { //si intersecano in un punto 
                if (CGAL::squared_distance(inters[0], polygon->edge(i)) <= approx ) {
                    a.push_back(inters[0]); 
                }
            }

           
        }        
    }

    return a; 

}



/*************************************/

vector<K::Point_2> divideSegment(CGAL::Segment_2<K> segment, float distance) {

    //creazione del path 
    vector<K::Point_2> path;

    //creo il vettore 
    K::Point_2 source = segment.source();
    K::Point_2 target = segment.target();
    CGAL::Vector_2<K> v(source, target);
   

    //calcolo la distanza tra le righe 
    float length = sqrt(v.squared_length());
    float mySweepDistance = distance;

    if (length <= mySweepDistance || mySweepDistance == 0) {
        path.push_back(segment.source());
        path.push_back(segment.target());
        return path;
    }


    int num_spaces = std::ceil(length/mySweepDistance);      
    mySweepDistance = length/num_spaces;
    int num_lines = num_spaces + 1; // init and ending point

    K::Point_2 next/* = source*/; 
    
    for (int i = 0; i < num_lines; i++) {
        next = source + ( (v/length)  * mySweepDistance * i);  //  v/length dovrebbe essere il vettore direzione 
        path.push_back(next); 
    }

    return path;
}


/*******************************************************/

int isLeft(K::Point_2 a, K::Point_2 b, K::Point_2 c){ //se l'angolo è di 180 viene circa 0 (perché non 0 ? ==> capire cosa fa questa funzione)
    

    float k = (b.x() - a.x())*(c.hy() - a.hy()) - (b.hy() - a.hy())*(c.x() - a.x());

    if ( k > 0.000001) {
        return 1;
    }
    else if( k < -0.000001){
        return -1;
    } 
    else{
        return 0;
    }

}



/*******************************************************/

bool arePointsEqual(K::Point_2 a, K::Point_2 b, float approx) {

    bool to_ret = false;
    if (CGAL::squared_distance(a,b) <= approx) {
        to_ret = true; 
    }
    return to_ret;
}

/*******************************************************/
bool areSegmentsEqual(CGAL::Segment_2<K> s1, CGAL::Segment_2<K> s2, float approx) { 
    bool to_ret = false; 
    if (arePointsEqual(s1.source(), s2.source(), approx) && arePointsEqual(s1.target(), s2.target(), approx)) {
        to_ret = true; 
    }
    else if (arePointsEqual(s1.source(), s2.target(), approx) && arePointsEqual(s1.target(), s2.source(), approx)) { 
        to_ret = true;
    }
    return to_ret; 

} 

/*******************************************************/
K::Point_2 nearestPoint(K::Point_2 p, vector<K::Point_2> points) {
    
    float minDistance = MAXFLOAT; 
    K::Point_2 nearest = points[0]; 
    for (size_t i = 0; i < points.size(); i++) {
        if (CGAL::squared_distance(p, points[i]) < minDistance) {
            minDistance = CGAL::squared_distance(p, points[i]); 
            nearest = points[i]; 
        }
    }
    return nearest; 
}

/*******************************************************/
pair<bool, CGAL::Segment_2<K>> concatenateSegments(CGAL::Segment_2<K> segment1, CGAL::Segment_2<K> segment2, float approx) {

    bool to_ret = false; 
    CGAL::Segment_2<K> resultingSegment(K::Point_2(-1,-1), K::Point_2(-1,-1)); 

    K::Point_2 s1 = segment1.source();
    K::Point_2 t1 = segment1.target();
    K::Point_2 s2 = segment2.source();
    K::Point_2 t2 = segment2.target(); 

    if (areParallel(segment1.supporting_line(), segment2.supporting_line(), 0.001)) {

        if (arePointsEqual(s1,s2,0.001)) {
            to_ret = true;
            resultingSegment = CGAL::Segment_2<K>(t1,t2); 
        }
        else if (arePointsEqual(s1,t2,0.001)) {
            to_ret = true;
            resultingSegment = CGAL::Segment_2<K>(t1, s2);
        }
        else if (arePointsEqual(t1,t2,0.001)) {
            to_ret = true;
            resultingSegment = CGAL::Segment_2<K>(s1,s2); 
        }
        else if (arePointsEqual(t1, s2, 0.001)) {
            to_ret = true;
            resultingSegment = CGAL::Segment_2<K>(s1,t2); 
        }
    }

    return make_pair(to_ret, resultingSegment);

}


/*******************************************************/
