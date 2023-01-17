#include "utils.hpp"

using namespace std; 

vector<pair<float,float>> readFromFile(string name){
    string filename("../" + name);
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
    K::Point_2 array[sz]; 
    for (size_t i = 0; i < sz; i++) array[i] = points.at(i); 
    CGAL::Polygon_2<K> p (array, array+sz);
    shared_ptr<CGAL::Polygon_2<K>> poly = make_shared<CGAL::Polygon_2<K>>(p);
    return poly;
}


/*************************************/


// void printInfo() {
//     cout << "0 -> optimal convex partition" << endl;
//     cout << "1 -> monotone partition" << endl;
//     cout << "2 -> approx convex partition" << endl;
//     cout << "3 -> greene approx convex partition" << endl;
// }




/*************************************/


double calculateAngle (CGAL::Vector_2<K> v, CGAL::Vector_2<K> w) {

    double theta = CGAL::scalar_product(v,w);
    double len1, len2;
    len1 = sqrt(v.squared_length());
    len2 = sqrt(w.squared_length());
    // cout << theta/ (len1*len2) << endl;
    return (theta/ (len1*len2));

}




/*************************************/


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

/*************************************/ 
//il secondo elemento del pair è l'indice del lato  in cui c'è l'intersezione
pair<vector<K::Point_2>,int> intersect_polygon_line_2(shared_ptr<CGAL::Polygon_2<K>> polygon, int edgeIndex) {

    CGAL::Line_2<K> line = polygon->edge(edgeIndex).supporting_line();
    size_t N = polygon->edges().size(); 
    int indexIntersection = -1;

    vector<K::Point_2> a;
    for (size_t i = 0 ; i < polygon->edges().size() ;  i++) {

        if ( i !=edgeIndex && i != (edgeIndex+1)%(N) &&  i != (edgeIndex-1+N)%(N) ) {
            const auto inter = CGAL::intersection(line, polygon->edge(i));
        
            if (inter){
                if (const CGAL::Segment_2<K>* s = boost::get<CGAL::Segment_2<K>>(&*inter) ) { //se si intersecano in un segmento niente , è il lato stesso 
                cout << "trovato intersezione con segmento " << endl;
                }
                else if (const K::Point_2* p = boost::get<K::Point_2>(&*inter)){  //se si intersecano in un punto 
                    a.push_back(*p);
                    indexIntersection = i; 
                    break;
                }
            }
        }
    }

    return make_pair(a,indexIntersection);
}


/*************************************/

//si suppone che sia convesso
K::Point_2* intersect_polygon_line(shared_ptr<CGAL::Polygon_2<K>> polygon, CGAL::Line_2<K> line) {

    static K::Point_2 a[2];
    int cont = 0;

    for (size_t i = 0 ; i < polygon->edges().size();  i++) {
      
        const auto inter = CGAL::intersection(line, polygon->edge(i));

        if (inter){
            // K::Point_2 tmp;
            if (const CGAL::Segment_2<K>* s = boost::get<CGAL::Segment_2<K>>(&*inter) ) { //se si intersecano in un segmento il source del segmento è il punto più "interno" del poligono?
                a[0] = polygon->edge(i).target();
                a[1] = polygon->edge(i).source();
                return a;
            }    
            else if (const K::Point_2* p = boost::get<K::Point_2>(&*inter)){ 
                a[cont] = *p;
                cont++;
                if (cont == 2) {
                    return a;
                }
            }
            else {
                cout << "Error" << endl;
            }
        }
    }
    if (cont == 0) {
        a[0] = K::Point_2(-1,-1);
        a[1] = a[0];
    }
    if (cont == 1) {
        a[1] = K::Point_2(-1,-1);
    } 
    return a;  
}
/*************************************/
//restituisce l'indice di v nel quale c'è l'elemento x // -1 se non lo trova 
int indexOf(vector<int> v, int x) {
    for (size_t i = 0; i < v.size(); i++) {
        if (v.at(i) == x) {
            return i;
        }
    }
    return -1;

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
    // float distance = sqrt(CGAL::squared_distance(sweepDirection, point));
    float num_lines = length/mySweepDistance; 
    if (num_lines != (int)num_lines) {
        num_lines = (int)num_lines++;
        mySweepDistance = length/num_lines;
    }

    if (length <= mySweepDistance || mySweepDistance == 0) {
        path.push_back(segment.source());
        path.push_back(segment.target());
        return path;
    }
   

    K::Point_2 next = source;
    int i = 1;
    
    while (segment.collinear_has_on(next)) {
        path.push_back(next);
        next = source + ( (v/length)  * mySweepDistance * i);  //  v/length dovrebbe essere il vettore direzione 
        i++;
    }

    return path;
}


/*************************************/

bool isLeft(K::Point_2 a, K::Point_2 b, K::Point_2 c){ //se l'angolo è di 180 viene 0
     return ((b.hx() - a.hx())*(c.hy() - a.hy()) - (b.hy() - a.hy())*(c.hx() - a.hx())) >= 0;
}

/*************************************/