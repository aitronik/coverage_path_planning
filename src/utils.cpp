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
                break;
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

//si suppone che polygon sia convesso
vector<K::Point_2> intersect_convex_polygon_line(shared_ptr<CGAL::Polygon_2<K>> polygon, CGAL::Line_2<K> line) {
    
    vector<K::Point_2> a; 
    size_t N = polygon->edges().size(); 

    for (size_t i = 0; i < N ; i++) {


        const auto inter = CGAL::intersection(line, polygon->edge(i)); 

        if (inter && a.size() <2 ) {
            
            // se l'intersezione è un segmento 
            if (const CGAL::Segment_2<K>* s = boost::get<CGAL::Segment_2<K>>(&*inter)) { 
                a.clear();
                a.push_back(s->source()); 
                a.push_back(s->target()); 
            }
            //se è un punto
            else if (const K::Point_2* p = boost::get<K::Point_2>(&*inter)){ 
                a.push_back(*p); 
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