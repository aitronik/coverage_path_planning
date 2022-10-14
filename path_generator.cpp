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
vector<K::Point_2> divideSegment(K::Segment_2 segment, float range) {
    
    //creo il vettore 
    K::Point_2 source = segment.source();
    K::Point_2 target = segment.target();
    CGAL::Vector_2<K> v(source, target);
   
    float length = sqrt(v.squared_length());
    // //in quanti punti lo divido
    // int num_points = (length/range) +1 ;

    //creazione del path 
    vector<K::Point_2> path;

    K::Point_2 next = source;
    int i = 0;

    while (segment.collinear_has_on(next)) {
        // printPoint(next);
        if (i != 0) path.push_back(next);
        i++;
        next = source + ( (v/length)  * range * i);  //  v/length dovrebbe essere il vettore direzione 
    }
    // path.push_back(target);

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
    for (int i = 0 ; i < polygon->edges().size();  i++) {
        if (/*CGAL::do_intersect(line, polygon.edge(i))*/auto inter = CGAL::intersection(line, polygon->edge(i))){
            if ( const CGAL::Segment_2<K>* s = boost::get<CGAL::Segment_2<K>>(&*inter) ) { //se si intersecano in un segmento il source del segmento è il punto più "interno" del poligono?
                a[cont] = s->source();
                cont++;
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
 


vector<K::Point_2> generatePath( shared_ptr<CGAL::Polygon_2<K>> polygon, float range) { //poi dovrà ritornare un path , ma per ora ritorna un vector di linee 


    auto[edge, point] = findSweepDirection(polygon);
    //edge è il lato parallelo alla direzione di spazzata 
    //point è il punto più lontano a quel lato 


    //creo griglia 
    vector<CGAL::Line_2<K>> grid = createGrid(polygon, edge, point , range);
    vector<K::Point_2> intersections;

    for (int i = 0; i < grid.size(); i++) {
        K::Point_2* a = intersect_polygon_line(polygon, grid.at(i));
        
        //o non interseca oppure interseca solo in un punto
        if (a[0].hx() == -1 || a[1].hx() == -1) continue;

        intersections.push_back(a[0]);
        intersections.push_back(a[1]);
        
    }   
    return intersections;

}



