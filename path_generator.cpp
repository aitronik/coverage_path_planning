#include "path_generator.hpp"

//trova la massima distanza tra un segmento e un insieme di punti
tuple<float, K::Point_2> maxDistance(vector<K::Point_2>& points, K::Segment_2& segment) { //altitudine
    float dist = 0; 
    float tmp; 
    K::Point_2 point; //puinto alla max distanza
    for (int i = 0; i < points.size(); i++) {
        tmp = CGAL::squared_distance(points.at(i), segment.supporting_line());
        if (tmp > dist) {
            dist = tmp;
            point = points.at(i);
        }
    }

    return make_tuple(dist,point);
}



//trova il lato parallelo alla direzione di spazzata e il punto più lontano
tuple<CGAL::Segment_2<K>, K::Point_2> findSweepDirection(shared_ptr<CGAL::Polygon_2<K>> polygon) {
    int n_edges = polygon->edges().size();
    K::Segment_2 edge;
    vector<float> altitudes;
    vector<K::Point_2> corrispondent_points;
    vector<K::Point_2> vertices = polygon->vertices();
    // float dist;
    // K::Point_2 point;
    //altitudine e lato hanno indice corrispondente nei due vector
    for (int i = 0; i < n_edges; i++) {
        edge = polygon->edge(i);
        // tuple<float, K::Point_2>
        auto[dist, point] = maxDistance(vertices, edge);
        altitudes.push_back(dist);
        corrispondent_points.push_back(point);
    }
    //cerco la minima altitudine 
    float weigth = altitudes.at(0);
    int index = 0; //indice del lato corrispondente alla minima altitudine
    for (int i = 1; i < altitudes.size(); i++) {
        if (altitudes.at(i) < weigth ) {
            weigth = altitudes.at(i);
            index = i;
        }
    }

    //la direzione di spazzata è la direzione di polygon.edge(index) 
    return make_tuple(polygon->edge(index), corrispondent_points.at(index));
}   


//divido il poligono perpendicolarmente alla direzione di sweep con ampiezza la larghezza del robot 
vector<K::Point_2> divideSegment(CGAL::Segment_2<K> segment, float range) {
    
    //creazione del path 
    vector<K::Point_2> path;

    //creo il vettore 
    K::Point_2 source = segment.source();
    K::Point_2 target = segment.target();
    CGAL::Vector_2<K> v(source, target);
   

    float length = sqrt(v.squared_length());
    if (length <= range || range == 0) {
        path.push_back(segment.source());
        path.push_back(segment.target());
        return path;
    }
    // //in quanti punti lo divido
    // int num_points = (length/range) +1 ;

    

    K::Point_2 next = source;
    int i = 0;
    

    while (segment.collinear_has_on(next)) {
        // printPoint(next);
        /*if (i != 0)*/ 
        next = source + ( (v/length)  * range * i);  //  v/length dovrebbe essere il vettore direzione 
        path.push_back(next); //idem a riga 86
        i++;

    }
    
    path.push_back(target); //se non voglio i bordi questo lo tolgo

    return path;
}


//restituisce un vector di linee che creano una griglia sul poliigono 
vector<CGAL::Line_2<K>> createGrid(shared_ptr<CGAL::Polygon_2<K>> polygon,  CGAL::Segment_2<K> sweepDirection, K::Point_2 point, float range) {
    vector<CGAL::Line_2<K>> grid;
    float distance = sqrt(CGAL::squared_distance(sweepDirection, point));
    float num_lines = distance/range; 
    if (num_lines != (int)num_lines) {
        num_lines = (int)num_lines++;
        range = distance/num_lines;
    }
    CGAL::Line_2<K> ortogonal = sweepDirection.supporting_line().perpendicular(point); //linea perpendicolare alla direzione di spazzata e passante per il punto più lontano del poligono 
    auto projection = CGAL::intersection(ortogonal, sweepDirection); //punto di proiezione del punto più lontano sul lato della direzione ==> è per forza un punto si ? 
    // CGAL::Vector_2<K> v (projection, point); //"altezza " del poligono 
    const K::Point_2* p = boost::get<K::Point_2>(&*projection);
    CGAL::Segment_2<K> h ( *p,point); 
    vector<K::Point_2> inters = divideSegment(h,range);
    //genero una linea con direzione sweepDirection.direction e che passa per il punto i-esimo 
    for (int i = 0; i < inters.size(); i++ ) {
        grid.push_back(CGAL::Line_2<K> (inters.at(i), sweepDirection.direction()));
    }
    return grid;
}


//POSSO SUPPORRE CHE SIA CONVESSO? 
K::Point_2* intersect_polygon_line(shared_ptr<CGAL::Polygon_2<K>> polygon, CGAL::Line_2<K> line) {
    static K::Point_2 a[2];
    int cont = 0;
    cout << "boh: " << polygon->edges().size() << endl;
    for (int i = 0 ; i < polygon->edges().size();  i++) {
        if (/*CGAL::do_intersect(line, polygon.edge(i))*/auto inter = CGAL::intersection(line, polygon->edge(i))){
            if ( const CGAL::Segment_2<K>* s = boost::get<CGAL::Segment_2<K>>(&*inter) ) { //se si intersecano in un segmento il source del segmento è il punto più "interno" del poligono?
               cout << "line" << endl;
                a[cont] = s->source();
                a[cont+1] = s->target();
                cout << a[cont].hx() << " " << a[cont].hy() << endl;
                cout << a[cont+1].hx() << " " << a[cont+1].hy() << endl;
                cont = cont++;
                continue;
            }    
            const K::Point_2* p = boost::get<K::Point_2>(&*inter); 
            a[cont] = *p;
            cont++;
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
 


vector<CGAL::Segment_2<K>> generatePathForOnePolygon( shared_ptr<CGAL::Polygon_2<K>> polygon, float range) { //poi dovrà ritornare un path , ma per ora ritorna un vector di linee 

    auto[edge, point] = findSweepDirection(polygon);
    //edge è il lato parallelo alla direzione di spazzata 
    //point è il punto più lontano a quel lato 


    //creo griglia 
    vector<CGAL::Line_2<K>> grid = createGrid(polygon, edge, point , range);
    vector<K::Point_2> intersections; //intersezioni tra le sweep lines e il poligono
    
    for (int i = 0; i < grid.size(); i++) {
        K::Point_2* a = intersect_polygon_line(polygon, grid.at(i));
        
        //o non interseca oppure interseca solo in un punto
        if (a[0].hx() == -1 || a[1].hx() == -1) continue;

        intersections.push_back(a[0]);
        intersections.push_back(a[1]);
        
    }   

    intersections.resize(intersections.size());

    //creazione path vero e proprio 
    vector<CGAL::Segment_2<K>> path; 
    CGAL::Segment_2<K> old;
    int cont = 0;

    cout << intersections.size() << endl;
    for (int i = 0; i < intersections.size(); i= i+2 ){
        cout << intersections.at(i).hx() << " ,"  << intersections.at(i).hy() << "          " << intersections.at(i+1).hx() << " ,"  << intersections.at(i+1).hy() << endl;
    }
    for (int i = 0; i < intersections.size() ;i = i+2) {
        CGAL::Segment_2<K> tmp (intersections[i], intersections[i+1]);
        // CGAL::Line_2<K> line (intersections[i], intersections[i+1]);
        vector< K::Point_2> p = divideSegment(tmp, 0); 

        if (p.size() == 1 ) { //è troppo piccolo, non ci passa => BISOGNA VALUTAREs
            return path;
        }


        CGAL::Segment_2<K> seg (p.at(0), p.at(p.size()-1)); //p.size()-2 nel caso in non cui voglio arrivare fino al bordo da un lato 


        if (cont == 0) {
            path.push_back(seg);
            cont++;
            old = path.at(cont-1);
        }
        if (i == intersections.size()-2 || i == intersections.size()-1 ) {
            CGAL::Segment_2<K> link (old.target(), seg.source());
            path.push_back(link); 
            cont++;
            CGAL::Segment_2<K> l(seg.source(), seg.target());
            path.push_back(l);
            cont++;
            old = path.at(cont-1);
        }
        // if (i == intersections.size()-2) {
        //     CGAL::Segment_2<K> link (old.target(), seg.source());
        //     path.push_back(link); 
        //     cont++;
        //     CGAL::Segment_2<K> l(seg.target(), seg.source());
        //     path.push_back(l);
        //     cont++;
        //     old = path.at(cont-1); 
        // }
        if (i%4==2) {
            CGAL::Segment_2<K> link (old.target(), seg.target());
            path.push_back(link); 
            cont++;
            CGAL::Segment_2<K> l(seg.target(), seg.source());
            path.push_back(l);
            cont++;
            old = path.at(cont-1); 
        } 
        if (i%4==0) {
            CGAL::Segment_2<K> link (old.target(), seg.source());
            path.push_back(link); 
            cont++;
            CGAL::Segment_2<K> l(seg.source(), seg.target());
            path.push_back(l);
            cont++;
            old = path.at(cont-1);
        }
        
        // if (i != 0  && i%2 != 0) {
        //     CGAL::Segment_2<K> link (old.target(), seg.target());
        //     path.push_back(link); 
        //     cont++;
        //     CGAL::Segment_2<K> l(seg.target(), seg.source());
        //     path.push_back(l);
        //     cont++;
        //     old = path.at(cont-1);

        // } 
        // else if (cont!= 0 && i%2 == 0) {
        //     CGAL::Segment_2<K> link (old.target(), seg.source());
        //     path.push_back(link); 
        //     cont++;
        //     CGAL::Segment_2<K> l(seg.source(), seg.target());
        //     path.push_back(l);
        //     cont++;
        //     old = path.at(cont-1);
        // }
       

    }

    return path;

}


int indexOfMinimum(vector<float>& dist, bool* visited) {
    int index = -1;
    float min = INT_MAX;
    
    for (int i = 0; i < dist.size(); i++) {
        if (dist.at(i) < min && !visited[i]) {
            min = dist.at(i);
            index = i;
        }
    }
    return index;
}


// bool is_in( vector< int>& v, int k) {
//     for (int i = 0; i < v.size(); i++) {
//         if (v.at(i) == k) return true;
//     }
//     return false;
// }



//sorg è il nodo sorgente 
//graph il grafo di adiacenza con i pesi 
//dist array delle distanze dal nodo sorgente 
void Dijkstra( vector<vector<int>>& graph, int sorg , vector<float>& distances) { //come passare per riferimento ? 
    
    int numNodes = graph.at(0).size();
    distances.resize(numNodes);
    bool visited[numNodes];

    for (int i = 0; i < numNodes; i++) {
        distances[i] = INT_MAX;
        visited[i] = false;
    }

    distances[sorg] = 0;

    for (int i = 0; i < numNodes; i++) {
        int index = indexOfMinimum(distances, visited);
        visited[index] = true;

        for (int j = 0; j < numNodes; j++) {

            if (graph.at(index).at(j) != 0 && !visited[j] && distances[index] != INT_MAX ) {

                if ( distances[index] + graph.at(index).at(j) < distances[j] ) {
                    distances[j] = distances[index] + graph.at(index).at(j);
                }

            }

        }
    }
}





vector<int> sortPolygons (vector<vector<vector<int>>> adj){

    int k = adj.size();
     //creo matrice che abbia i pesi come elementi, al momento tutti 1 se i poligoni sono adiacenti 
    vector< vector < int> > adj_new;

    for (int i = 0; i < k; i++) {
        vector<int> tmp;
        for (int j = 0; j < k; j++) {
            if (adj[i][j][0] != -1) { //il secondo può essere -1 e il primo no , ma non il contrario
                tmp.push_back(1);
            }
            else {
                tmp.push_back(0);
            }  
        }
        adj_new.push_back(tmp);
    }


    vector<float> dist;
    
    Dijkstra(adj_new, 0, dist );
   

    bool visited[dist.size()];
    for (int i = 0; i < dist.size(); i++) {
        visited[i] = false;
    }
    
    vector<int> polygonSorted; 

    for (int i = 0; i < dist.size(); i++) {
        int index = indexOfMinimum(dist, visited);
        visited[index] = true;
        polygonSorted.push_back(index);
    }

    return polygonSorted;
     
}



// vector<K::Point_2> generatePath(Polygon_list partition_polys,  vector<K::Point_2> points , int src, int dest, )







