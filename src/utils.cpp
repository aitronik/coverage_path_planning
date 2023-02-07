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
    K::Point_2 array[sz]; 
    for (size_t i = 0; i < sz; i++) array[i] = points.at(i); 
    CGAL::Polygon_2<K> p (array, array+sz);
    shared_ptr<CGAL::Polygon_2<K>> poly = make_shared<CGAL::Polygon_2<K>>(p);
    return poly;
}


/*************************************/



double calculateAngle (CGAL::Vector_2<K> v, CGAL::Vector_2<K> w) {

    double theta = CGAL::scalar_product(v,w);
    double len1, len2;
    len1 = sqrt(v.squared_length());
    len2 = sqrt(w.squared_length());

    // cout << "theta: " << theta << endl; 
    // cout << "len1: " << len1 << endl; 
    // cout << "len2: " << len2 << endl;

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
pair<K::Point_2,int> intersect_concave_polygon_at_index(shared_ptr<CGAL::Polygon_2<K>> polygon, int edgeIndex, int vertexIndex) {

    CGAL::Line_2<K> line = polygon->edge(edgeIndex).supporting_line();
    size_t N = polygon->edges().size(); 
    
    vector<int> indexIntersections;
    vector<K::Point_2> points;
    for (size_t i = 0 ; i < polygon->edges().size() ;  i++) {
        // non confrontarlo con te stesso, precedente e successivo
        if ( i !=edgeIndex && i != (edgeIndex+1)%(N) &&  i != (edgeIndex-1+N)%(N) ) {
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
    K::Point_2 closestPoint(-1,-1);
    int index_intersection = -1;
    float minDistance = MAXFLOAT;  
    for (size_t i = 0; i < points.size(); i++){
        // prendo il più vicino al vertex
        const float dist = CGAL::squared_distance(points[i], vertex);
        if ( dist < minDistance ) {
            closestPoint = points[i]; 
            index_intersection = indexIntersections[i]; 
            minDistance = dist;
        }
    
    }


    return make_pair(closestPoint, index_intersection);
}


/*************************************/

//si suppone che sia convesso
vector<K::Point_2> intersect_convex_polygon_line(shared_ptr<CGAL::Polygon_2<K>> polygon, CGAL::Line_2<K> line) {
    
    vector<K::Point_2> a; 
    size_t N = polygon->edges().size(); 

 

    for (size_t i = 0; i < N ; i++) {
        
        const auto inter = CGAL::intersection(line, polygon->edge(i)); 

        if (inter) {
            
            if (a.size() < 2) {
                // se l'intersezione è un segmento 
                if (const CGAL::Segment_2<K>* s = boost::get<CGAL::Segment_2<K>>(&*inter)) { 
                    a.clear();
                    a.push_back(s->source()); 
                    a.push_back(s->target()); 
                }
                else if (const K::Point_2* p = boost::get<K::Point_2>(&*inter)){ 
                    a.push_back(*p); 
                }

            }

        }
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


/*************************************/

int isCollinear(shared_ptr<CGAL::Polygon_2<K>> polygon, int edgeIndex) {
    
 int to_return;
    int N = polygon->edges().size(); 


    if ( isLeft( polygon->edge((edgeIndex-1+N)%(N)).source(), polygon->edge(edgeIndex).source(), polygon->edge(edgeIndex).target()) == 0 ) {
        to_return = (edgeIndex-1+N)%N; 

    }
    else if (isLeft(polygon->edge(edgeIndex).source(), polygon->edge(edgeIndex).target(), polygon->edge((edgeIndex+1)%N).target() ) == 0 ){
        to_return = (edgeIndex+1)%N; 
    }
    else {
        to_return = -1;
    }
    return to_return;
}



/*******************************************************/

int isLeft(K::Point_2 a, K::Point_2 b, K::Point_2 c){ //se l'angolo è di 180 viene circa 0 (perché non 0 ? ==> capire cosa fa questa funzione)
    

    float k = (b.x() - a.x())*(c.hy() - a.hy()) - (b.hy() - a.hy())*(c.x() - a.x());
    // cout << a << "\t" << b << "\t" << c << endl;
    // cout << "k" << k << endl;

    if ( k > 0.000001) {
        return 1;
        // return k;
    }
    else if( k < -0.000001){
        return -1;
        // return k;
    } 
    else{
        return 0;
    }

}


/*******************************************************/

void printPointCoordinates(K::Point_2 p) {
    cout << p.x() << ", " << p.hy() << endl; 
}


/*******************************************************/

bool areEqual(K::Point_2 a, K::Point_2 b) {
    bool to_ret = false;
    if (a.x() == b.x() && a.hy() == b.hy()) {
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