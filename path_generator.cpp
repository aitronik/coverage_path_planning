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
vector<K::Point_2> divideSegment(CGAL::Segment_2<K> segment, float sweep_distance) {
    
    //creazione del path 
    vector<K::Point_2> path;

    //creo il vettore 
    K::Point_2 source = segment.source();
    K::Point_2 target = segment.target();
    CGAL::Vector_2<K> v(source, target);
   

    float length = sqrt(v.squared_length());

    if (length <= sweep_distance || sweep_distance == 0) {
        path.push_back(segment.source());
        path.push_back(segment.target());
        return path;
    }
   
    K::Point_2 next = source;
    int i = 1;
    
    // path.push_back(source); //perché non rientra nel caso precedente? 

    while (segment.collinear_has_on(next)) {
        path.push_back(next);
        next = source + ( (v/length)  * sweep_distance * i);  //  v/length dovrebbe essere il vettore direzione 
        i++;
    }
    
    return path;
}


//restituisce un vector di linee che creano una griglia sul poliigono 
vector<CGAL::Line_2<K>> createGrid(shared_ptr<CGAL::Polygon_2<K>> polygon,  CGAL::Segment_2<K> sweepDirection, K::Point_2 point, float sweep_distance) {
    vector<CGAL::Line_2<K>> grid;
    float distance = sqrt(CGAL::squared_distance(sweepDirection, point));
    float num_lines = distance/sweep_distance; 
    if (num_lines != (int)num_lines) {
        num_lines = (int)num_lines++;
        sweep_distance = distance/num_lines;
    }
    // cout << "createGrid: ampiezza sweep distance: " << sweep_distance << endl;
    CGAL::Line_2<K> ortogonal = sweepDirection.supporting_line().perpendicular(point); //linea perpendicolare alla direzione di spazzata e passante per il punto più lontano del poligono 
    auto projection = CGAL::intersection(ortogonal, sweepDirection); //punto di proiezione del punto più lontano sul lato della direzione ==> è per forza un punto si ? 

    const K::Point_2* p = boost::get<K::Point_2>(&*projection);
    CGAL::Segment_2<K> h ( *p,point); 

    // cout << "createGrid: altezza: " << sqrt(h.squared_length()) << endl;

    vector<K::Point_2> inters = divideSegment(h,sweep_distance);
    // cout <<  "createGrid: numero strisciate: " << inters.size() << endl;

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
    for (int i = 0 ; i < polygon->edges().size();  i++) {
        const auto inter = CGAL::intersection(line, polygon->edge(i));
        if (inter){
            if ( const CGAL::Segment_2<K>* s = boost::get<CGAL::Segment_2<K>>(&*inter) ) { //se si intersecano in un segmento il source del segmento è il punto più "interno" del poligono?
                //Prima target perché altrimenti viene in ordine diverso dagli altri segmenti intersezione
                a[0] = polygon->edge(i).target();
                a[1] = polygon->edge(i).source();
                return a;
            }    
            else {
                const K::Point_2* p = boost::get<K::Point_2>(&*inter);
                a[cont] = *p;
                cont++;
                if (cont == 2) {
                    return a;
                }
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
 

//borders[i] == 0 se non va lasciato il bordo libero 
//PUò non essere un poligono? 
vector<CGAL::Segment_2<K>> generatePathForOnePolygon( shared_ptr<CGAL::Polygon_2<K>> polygon, float sweep_distance,vector<bool> borders/*, K::Point_2 start_point*/) { 

    //start_point deve essere uno dei 4 vertici del poligono da cui possono partire le strisciate (come fare a controllare se è uno di quei 4? )

    // if ( !count(polygon->vertices().begin(), polygon->vertices().end(), start_point)) {
    //     //errore 

    // }

    // if (borders.size() != polygon->edges().size() ) {
    //     cout << "generatePathForOnePolygon: errore" << endl;
    // }

    borders.resize(borders.size());
    vector<K::Point_2> vertices;

    CGAL::Polygon_2<K> pol= *polygon;
    shared_ptr<CGAL::Polygon_2<K>> polygon_new =  make_shared <CGAL::Polygon_2<K>>(pol);
    
    // cout << "Vertici poligono iniziale: " << endl;
    // for (int i = 0; i < polygon->vertices().size(); i++) {
    //     cout << polygon->vertex(i).hx() << ", " << polygon->vertex(i).hy() << endl;
    // }
    
    //se ci sono adiacenze 
    //creo un nuovo poligono con i lati spostati
    for (int i = 0; i < borders.size(); i++) {
        
        if (borders[i] != 0) { //lascio uno spazio diverso nelle adiacenze

            CGAL::Segment_2<K> corr = polygon_new->edge(i); //lato corrente

            CGAL::Line_2<K> perp(corr);
            perp = perp.perpendicular(corr.source()); //perpendicolare al lato 
            CGAL::Vector_2<K> v = perp.to_vector();
            

            double angle1,angle2;
            if (i == 0) {
                angle1 = calculateAngle(corr.to_vector(), polygon_new->edge(borders.size()-1).to_vector());
                angle2 = calculateAngle(corr.to_vector(), polygon_new->edge(i+1).to_vector());
            } 
            else if (i == borders.size()-1) {
                angle1 = calculateAngle(corr.to_vector(), polygon_new->edge(i-1).to_vector());
                angle2 = calculateAngle(corr.to_vector(), polygon_new->edge(0).to_vector() );
            }
            else {
                angle1 = calculateAngle(corr.to_vector(), polygon_new->edge(i-1).to_vector());
                angle2 = calculateAngle(corr.to_vector(), polygon_new->edge(i+1).to_vector());
            }
            angle1 = acos(angle1);
            angle2 = acos(angle2);
            angle1 = sin(angle1);
            angle2 = sin(angle2);
            angle1 = min(angle1, angle2);


            double offset = (sweep_distance/2) * angle1;

            K::Point_2 p = corr.source() + ( (v/(sqrt(v.squared_length()))) * offset );
            CGAL::Line_2<K> new_line(p, corr.direction() );
            K::Point_2* a = intersect_polygon_line( polygon_new, new_line);
            //interseca per forza in due punti? 
            int cont = 0;
            for (K::Point_2 &p : polygon_new->container()) {
                if (p == polygon_new->edge(i).source() || p == polygon_new->edge(i).target() ) {
                    if ( cont == 0) {
                        p = a[0];
                        cont ++;
                    }
                    else {
                        p = a[1];
                    }
                }

            }
        }
    }

    // cout << "Vertici poligono nuovo: " << endl;
    // for (int i = 0; i < polygon_new->vertices().size(); i++) {
    //     cout << polygon_new->vertex(i).hx() << ", " << polygon_new->vertex(i).hy() << endl;
    // }

    auto[edge, point] = findSweepDirection(polygon_new); //questo dovrebbe funzionare bene
    //edge è il lato parallelo alla direzione di spazzata 
    //point è il punto più lontano a quel lato 


    //creo griglia 
    vector<CGAL::Line_2<K>> grid = createGrid(polygon_new, edge, point , sweep_distance);
    // cout <<"generatePath grid.size: " << grid.size() << endl;
    vector<K::Point_2> intersections; //intersezioni tra le sweep lines e il poligono
    
    for (int i = 0; i < grid.size(); i++) {

        K::Point_2* a = intersect_polygon_line(polygon_new, grid.at(i));
        
        //o non interseca oppure interseca solo in un punto
        if (a[0].hx() == -1 || a[1].hx() == -1) {
            cout << a[0].hx() << ", " << a[0].hy() << "   " << a[1].hx() << ", " << a[1].hy() << endl;
        }
        else {
            intersections.push_back(a[0]);
            intersections.push_back(a[1]);
        }
        
    }   

    intersections.resize(intersections.size());
    // cout << "generatePath intersections.size(): " << intersections.size() << endl;
    //creazione path vero e proprio 
    vector<CGAL::Segment_2<K>> path; 
    CGAL::Segment_2<K> old;
    int cont = 0;

    for (int i = 0; i < intersections.size() ; i = i+2) {

        CGAL::Segment_2<K> seg (intersections[i], intersections[i+1]);

        // //SE IL ROBOT NON CI PASSA?? 
        // vector< K::Point_2> p = divideSegment(tmp, 0); 
        // cout << p.size() << endl;
        // if (p.size() == 1 ) { //è troppo piccolo, non ci passa => BISOGNA VALUTARE
        //     cout << "Boh" << endl;
        //     return path;
        // }

        //cont = numero di pezzi nel path compresi i raccordi 

        if (i == 0) {
            path.push_back(seg);
            old = seg;
            cont++;
        }
        else if (i%4==2) {
            CGAL::Segment_2<K> link(old.target(), seg.target());
            path.push_back(link);
            CGAL::Segment_2<K> l(seg.target(), seg.source());
            cont = cont + 2;
            path.push_back(l);
            old = l;
        }
        else { //i%4 == 0
            CGAL::Segment_2<K> link(old.target(), seg.source());
            path.push_back(link);
            CGAL::Segment_2<K> l(seg.source(), seg.target());
            cont = cont + 2;
            path.push_back(l);
            old = l;
        }
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


//sorg è il nodo sorgente 
//graph il grafo di adiacenza con i pesi 
//dist array delle distanze dal nodo sorgente 
void Dijkstra( vector<vector<int>>& graph, int sorg , vector<float>& distances) { 
    
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





vector<int> sortPolygons (vector<vector<vector<int>>> adj){ //si può aggiungere l'indice di start che al momento è sempre 0 nella chiamata a findMinRoute

    vector<int> route = findMinRoute(adj,0);
    vector<int> polygonSorted; 


    int cont = 0;
    int j = 0;
    while (cont < route.size()) {
        for (int i = 0; i < route.size(); i++) {
    
            if (route[i] == j-1) {
                polygonSorted.push_back(i);
                j = i+1;
                cont++;
                break;
            }
        }
    }

    return polygonSorted;
     
}


int numAdiacency(vector<vector<vector<int>>>&adj, int node) {
    int x = 0; 
    for (int i = 0; i < adj[node].size(); i++) {
        if (adj[node][i][0] != -1 || adj[node][i][1] != -1) {
            x++;
        }
    }
    return x;
}


vector<int> findMinRoute(vector<vector<vector<int>>>&adj, int start) {
    int N = adj.size(); 
    //creo matrice che abbia i pesi come elementi, al momento tutti 1 se i poligoni sono adiacenti 
    vector< vector < int> > adj_new;

    for (int i = 0; i < N; i++) {
        vector<int> tmp;
        for (int j = 0; j < N; j++) {
            if (adj[i][j][0] != -1) { //il secondo può essere -1 e il primo no , ma non il contrario
                tmp.push_back(1);
            }
            else {
                tmp.push_back(0);
            }  
        }
        adj_new.push_back(tmp);
    }

        //creazione matrice con i pesi ==> per adesso peso : numero di poligoni di distanza
    vector<vector<float>> matrix;

    for (int i = 0; i < N; i++ ) {
        vector<float> distances;
        Dijkstra(adj_new, i, distances);
        for (int j = 0 ; j < distances.size(); j++) {
            distances[j] = distances[j];            
        }
        matrix.push_back(distances);
    }

    
    //inizializzo il vettore dei nodi visitati e dei costi 
    //start è il nodo di partenza 
    //il costo per andare a start è 0

    vector<bool> visitedNodes;
    vector<float> cost;
    vector<int> precedent; 
    visitedNodes.resize(N);
    cost.resize(N);
    precedent.resize(N);

    for (int i = 0; i < N; i++) {
        visitedNodes[i] = false; 
        cost[i] = INT_MAX;
        precedent[i] = -1;
    }


    visitedNodes[start] = true; 
    cost[start] = 0; 
    
    int corrente = start;
    int cont = 0;
    bool all_visited = false;



    while (!all_visited) {

    

        vector<float> tmp_cost;
        tmp_cost.resize(N);
        for (int i = 0; i < N ; i++ ) {
            tmp_cost[i] = INT_MAX;
        }


        for (int i = 0; i < N; i++) {
            if (!visitedNodes[i]) { //i nodo non ancora visitato
                if (cost[i] > cost[corrente] + matrix[corrente][i]) {
                    tmp_cost[i] = cost[corrente] + matrix[corrente][i];
                }
            }
        }



        //per terminazione
        all_visited = true;
        for (int i = 0 ; i < N ; i++ ) {
            if (visitedNodes[i] == false ) {
                all_visited = false;
            }
        }


        int j_min = 0;
        int min = INT_MAX;
        for (int j = 0; j < N; j++) {
            if (tmp_cost[j] == min ){ //se costano = prendo quello che ha più adiacenze 

                if (j_min != 0 && numAdiacency(adj,j) < numAdiacency(adj,j_min) ) {
                    j_min = j;
                }
            }
            else if (tmp_cost[j] < min) {
                min = tmp_cost[j];
                j_min = j;

            }
        }       
        if (j_min != start) {
            cost[j_min] = cost[corrente] + matrix[corrente][j_min];
            precedent[j_min] = corrente;
            visitedNodes[j_min] = true;
            corrente = j_min;
        }

        // cout << "corrente: " << corrente << endl;
        // corrente = (corrente+1)%N;

        continue;

    }


    return precedent;
}





