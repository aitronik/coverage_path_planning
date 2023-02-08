#include "CoveragePathCreator.h"

CoveragePathCreator::CoveragePathCreator()
{
    // parametri di default
    m_doPlotting = true;
    m_addPerimeterToPath = true;
}

/*******************************************************/

CoveragePathCreator::~CoveragePathCreator() {
}

/*******************************************************/

bool CoveragePathCreator::init(vector<pair<float, float>> points, float sweepDistance, int decompositionType)
{

    // vector<K::Point_2> perimeter_vertices;
    for (size_t i = 0; i < points.size(); i++) {
        K::Point_2 p(points.at(i).first, points.at(i).second);
        m_initialPerimeterVertices.push_back(p);
    }

    // creazione del poligono iniziale
    m_initialPolygon = createPolygon(m_initialPerimeterVertices);
    m_decompositionType = decompositionType;
    m_sweepDistance = sweepDistance;
    // m_numberOfTurnes = 0;

    // tipo di decomosizione
    if (m_decompositionType != 0 && m_decompositionType != 1 && m_decompositionType != 2 && m_decompositionType != 3 && m_decompositionType != 4)
    {
        cout << "CoveragePathCreator: decomposition type error" << endl;
        return false;
    }

    m_decompositionType = m_decompositionType;

    // inizializzazione dell'plot helper
    if (m_doPlotting == true)
    {
        if (!m_Helper.init(m_initialPerimeterVertices)){
            return false;
        }
    }

    return true;
}
/*******************************************************/

// decompone il poligono iniziale in sottopoligoni convessi
bool CoveragePathCreator::decompose() {

    cout << "decompose()" << endl;

    Partition_traits_2 traits(CGAL::make_property_map(m_simplyfiedVertices));
    Polygon tmp_polygon;
    for (size_t i = 0; i < m_simplyfiedVertices.size(); i++)
    {
        tmp_polygon.push_back(i);
    }


    // in base al tipo di decomposizione
    switch (m_decompositionType) {
        case 0: {

            CGAL::optimal_convex_partition_2(tmp_polygon.vertices_begin(),
                                            tmp_polygon.vertices_end(),
                                            back_inserter(m_decomposedPolysOfIndices),
                                            traits);
            m_decompositionName = "Optimal convex partition";
            m_decomposedVertices = m_simplyfiedVertices; //i vertici in questo caso di decomposizione restano gli stessi 
            break;
        } 

        case 1: { //mydecomposition 
            
            if (!m_decomposer.init(m_simplyfiedPolygon)) {
                cout << "MyDecomposition: errore inizializzazione"; 
                return 1;
            } 

            m_decomposer.run();
            pair<Polygon_list, vector<K::Point_2> > p = m_decomposer.getDecomposition();
            m_decomposedPolysOfIndices = p.first;
            m_decomposedVertices = p.second;
            m_decompositionName = "My decomposition"; 

            break;
        }

        default: {
            cout << "CoveragePathCreator: decomposition error" << endl;
            return false;
        }
    }

    // controllo decomposizione  ==> se sono nel caso della mia decompolisione non lo controllo 
    if ( m_decompositionType != 1 && !CGAL::partition_is_valid_2(tmp_polygon.vertices_begin(), tmp_polygon.vertices_end(), m_decomposedPolysOfIndices.begin(), m_decomposedPolysOfIndices.end(), traits)) {
        cout << "CoveragePathCreator: decomposition error" << endl;
        return false;
    }

    //setto quanti sono i poligoni dopo la decomposizione
    m_NumerOfDecomposedSubPolygon = (int)m_decomposedPolysOfIndices.size();

    return true;
}

/*******************************************************/

void CoveragePathCreator::setAddPerimeterToPath(bool b) {
    m_addPerimeterToPath = b;
}

/*******************************************************/
// crea una matrice di adiacenza in cui gli elementi sono coppie di indici di punti che rappresentano un'adiacenza tra due sottopoligoni (devono essere tutti convessi)
void CoveragePathCreator::createAdjMatrix() {

    cout << "createAdjMatrix()" << endl;

    //matrice N*N*2 (numero poligoni-numero poligoni - indici dei due vertici in cui sono adiacenti) 
    int N = m_NumerOfDecomposedSubPolygon; 
    m_adj.resize(N);
    for (size_t i = 0; i < N; i++)
    {
        m_adj.at(i).resize(N);
        for (size_t j = 0; j < N; j++)
        {
            m_adj.at(i).at(j).resize(2);
        }
    }

    int cont_i = 0;
    int cont_j = 0;

    for (const Polygon &poly1 : m_decomposedPolysOfIndices)
    {
        for (const Polygon &poly2 : m_decomposedPolysOfIndices)
        {
            if (cont_i == cont_j)
            {
                m_adj[cont_i][cont_j][0] = -1;
                m_adj[cont_i][cont_j][1] = -1;
            }

            else
            {
                int vert_i, vert_j;
                adjacency(poly1.container(), poly2.container(), vert_i, vert_j);
                m_adj[cont_i][cont_j][0] = vert_i;
                m_adj[cont_i][cont_j][1] = vert_j;
            }
            cont_j++;
        }
        cont_i++;
        cont_j = 0;
    }

}

/*******************************************************/

// restituisce l'indice del poligono non ancora visitato a distanza minima dalla sorgente
int CoveragePathCreator::indexOfMinimum(vector<float> &dist, bool *visited) {

    cout << "indexOfMinimum()" << endl; 

    int index = -1;
    float min = INT_MAX;

    for (size_t i = 0; i < dist.size(); i++)
    {
        if (dist.at(i) < min && !visited[i])
        {
            min = dist.at(i);
            index = i;
        }
    }
    return index;
}

/*******************************************************/

int CoveragePathCreator::initialIndex(float a, float b, float c, float d) { // si può fare in modo più furbo

    cout << "initialIndex()"<< endl; 

    float min = a;
    int index = 0;
    if (b < min)
    {
        min = b;
        index = 1;
    }
    if (c < min)
    {
        min = c;
        index = 2;
    }
    if (d < min)
    {
        min = d;
        index = 3;
    }
    // cout << "INdex" << index << endl;
    return index;
}

/*******************************************************/

int CoveragePathCreator::numAdiacency(int node) {
    cout << "numAdiacency()" << endl; 

    int x = 0;
    for (size_t i = 0; i < m_adj[node].size(); i++)
    {
        if (m_adj[node][i][0] != -1 || m_adj[node][i][1] != -1)
        {
            x++;
        }
    }
    return x;
}

/*******************************************************/

//DEVO CONTROLLARE CHE LA MATRICE CON I PESI CHE USA QUA SIA EFFETTIVAMENTE DIVERSA DA M_ADJWEIGTHMATRIX
void CoveragePathCreator::Dijkstra(vector<vector<int>> &graph, int sorg, vector<float> &distances)
{

    cout << "Dijkstra()" << endl; 

    size_t numNodes = graph.at(0).size();
    distances.resize(numNodes);
    bool visited[numNodes];

    for (size_t i = 0; i < numNodes; i++)
    {
        distances[i] = INT_MAX;
        visited[i] = false;
    }

    distances[sorg] = 0;

    for (size_t i = 0; i < numNodes; i++)
    {
        int index = indexOfMinimum(distances, visited);
        visited[index] = true;

        for (size_t j = 0; j < numNodes; j++)
        {
            if (graph.at(index).at(j) != 0 && !visited[j] && distances[index] != INT_MAX)
            {
                if (distances[index] + graph.at(index).at(j) < distances[j])
                {
                    distances[j] = distances[index] + graph.at(index).at(j);
                }
            }
        }
    }
}

/*******************************************************/

void CoveragePathCreator::createAdjWeigthMatrix() { // crea una matrice di adiacenza che come elementi ha i costi (numero poligoni di distanza)

    cout << "createAdjWeigthMatrix()" << endl; 
    size_t N = m_adj.size();
    // creo matrice che abbia i pesi come elementi, al momento tutti 1 se i poligoni sono adiacenti
    vector<vector<int>> adj_new;

    for (size_t i = 0; i < N; i++) {
        vector<int> tmp;
        for (size_t j = 0; j < N; j++) {
            if (m_adj[i][j][0] != -1) { // il secondo può essere -1 e il primo no , ma non il contrario
                tmp.push_back(1);
            }
            else {
                tmp.push_back(0);
            }
        }
        adj_new.push_back(tmp);
    }

    for (size_t i = 0; i < N; i++)
    {
        vector<float> distances;
        Dijkstra(adj_new, i, distances);
        m_adjWeigthMatrix.push_back(distances);
    }
}

/*******************************************************/

// trova il percorso che tocca tutti i nodi minimizzando la distanza
vector<int> CoveragePathCreator::findMinRouteTo(int start)
{

    cout << "findMinRouteTo()" << endl; 
    size_t N = m_adj.size();
    // inizializzo il vettore dei nodi visitati e dei costi
    // start è il nodo di partenza
    // il costo per andare a start è 0
    vector<bool> visitedNodes;
    vector<float> cost;
    vector<int> precedent;
    visitedNodes.resize(N);
    cost.resize(N);
    precedent.resize(N);

    for (size_t i = 0; i < N; i++)
    {
        visitedNodes[i] = false;
        cost[i] = INT_MAX;
        precedent[i] = -1;
    }

    visitedNodes[start] = true;
    cost[start] = 0;

    int corrente = start;
    bool all_visited = false;

    while (!all_visited)
    {

        vector<float> tmp_cost;
        tmp_cost.resize(N);
        for (size_t i = 0; i < N; i++)
        {
            tmp_cost[i] = INT_MAX;
        }

        for (size_t i = 0; i < N; i++)
        {
            if (!visitedNodes[i])
            { // i nodo non ancora visitato
                if (cost[i] > cost[corrente] + m_adjWeigthMatrix[corrente][i])
                {
                    tmp_cost[i] = cost[corrente] + m_adjWeigthMatrix[corrente][i];
                }
            }
        }

        // per terminazione
        all_visited = true;
        for (size_t i = 0; i < N; i++)
        {
            if (visitedNodes[i] == false)
            {
                all_visited = false;
            }
        }

        int j_min = start;
        bool atLeastOne = false;
        int min = INT_MAX;

        // scelgo un nodo tra quelli con i costi "aggiornabili" come nodo successivo ==> quello con il minimo costo
        for (size_t j = 0; j < N; j++)
        {
            if (tmp_cost[j] == min)
            { // se costano = prendo quello che ha più adiacenze

                if (atLeastOne && numAdiacency(j) < numAdiacency(j_min))
                {
                    j_min = j;
                }
            }
            else if (tmp_cost[j] < min)
            {
                atLeastOne = true;
                min = tmp_cost[j];
                j_min = j;
            }
        }

        if (j_min != start)
        {
            cost[j_min] = cost[corrente] + m_adjWeigthMatrix[corrente][j_min];
            precedent[j_min] = corrente;
            visitedNodes[j_min] = true;
            corrente = j_min;
        }

        continue;
    }

    return precedent;
}

/*******************************************************/

// trova il percorso che trova la strada tra start ed end minimizzando la distanza
vector<int> CoveragePathCreator::findMinRouteBetween(int start, int end) {

    cout << "findMinRouteBetween()" << endl; 

    vector<int> precedent; 
    vector<int> route;     
    size_t N = m_adj.size(); 
    vector<bool> visitedNodes;
    visitedNodes.resize(N);
    precedent.resize(N);
    for (size_t i = 0 ; i < N ; i++) { 
        visitedNodes[i] = false; 
        precedent[i] = -1;
    }
    visitedNodes[start] = true; 
    int corr = start; 

    while (!visitedNodes[end]) {

        if (m_adjWeigthMatrix[corr][end] == 1) { //ovvero sono adiacenti 
            precedent[end] = corr; 
            visitedNodes[end] = true;
        } 
        else {
            while(!(m_adjWeigthMatrix[corr][end] == 1) ) {
                //scegli il nodo tra gli adiacenti a minor distanza 
                vector<int> adiacents; 
                //creo adiacents 
                for (size_t i = 0; i < N; i++) {
                    if (m_adjWeigthMatrix[corr][i] == 1) {
                        adiacents.push_back(i); 
                    }
                }                
                //scelgo
                int min_dist = INT_MAX; 
                int closer;
                for (size_t i = 0; i < adiacents.size(); i++) {
                    if (m_adjWeigthMatrix[adiacents[i]][end] < min_dist && !visitedNodes[adiacents[i]] ) {
                        min_dist = m_adjWeigthMatrix[adiacents[i]][end];
                        closer = adiacents[i];
                    }
                }
                
                precedent[closer] = corr;
                visitedNodes[closer] = true; 
                corr = closer;
            }
        }
    }

    // int corr = start;
    route.clear();
    route.push_back(start);
    corr = start;

    //li ordino 
    while (corr != end) {

        for (size_t i = 0; i < N ; i++ ) {
            if (precedent[i] == corr) {
                route.push_back(i); 
                corr = i; 
                break;
            }
        }

    }

    return route;
}





/*******************************************************/

bool CoveragePathCreator::orderSubPolygons()
{ // si può aggiungere l'indice di start che al momento è sempre 0 nella chiamata a findMinRoute

    cout << "orderSubPolygons()" << endl; 

    bool found = false;
    int start = -1;
    // cerco qual è il poligono start, ovvero quello che contiene il first Vertex
    
    //m_decomposedVertices sono i vertici dopo la decomposizione 

    for (const Polygon &poly : m_decomposedPolysOfIndices) {
        if (!found) {
            start++; 
            for (size_t k : poly.container()) {
                if (m_decomposedVertices[k] == m_firstVertex) {
                    found = true; 
                }
            }
        }
    }

    createAdjWeigthMatrix();
    vector<int> route = findMinRouteTo(start); // route è il vector dei precedenti -> all'indice i c'è il precedente del poligono i

    // oridnamento effettivo
    size_t cont = 0;
    int j = 0;
    while (cont < route.size())
    {
        int n = 0;
        size_t i;
        for (i = 0; i < route.size(); i++)
        {
            if (route[i] == j - 1)
            {
                m_polygonsSorted.push_back(i);
                j = i + 1;
                n++;
                cont++;
                break;
            }
        }

        if (i == route.size() && n == 0) { // non lo ha trovato
            cout << "CoveragePathCreator: ordering polygon error" << endl;
            return false;
        }
    }

    return true;
}

/*******************************************************/
// trova la massima distanza tra un segmento e un insieme di punti
tuple<float, K::Point_2> CoveragePathCreator::maxDistance(vector<K::Point_2> &points, K::Segment_2 &segment)
{ // altitudine

    cout << "maxDistance()" << endl; 

    float dist = 0;
    float tmp;
    K::Point_2 point; // puinto alla max distanza
    for (size_t i = 0; i < points.size(); i++)
    {
        tmp = CGAL::squared_distance(points.at(i), segment.supporting_line());
        if (tmp > dist)
        {
            dist = tmp;
            point = points.at(i);
        }
    }

    return make_tuple(dist, point);
}

/*******************************************************/
// trova il lato parallelo alla direzione di spazzata e il punto più lontano
tuple<CGAL::Segment_2<K>, K::Point_2> CoveragePathCreator::findSweepDirection(shared_ptr<CGAL::Polygon_2<K>> polygon)
{
    cout << "findSweeepDirection()" << endl; 

    size_t n_edges = polygon->edges().size();

    K::Segment_2 edge;
    vector<float> altitudes;
    vector<K::Point_2> corrispondent_points;
    vector<K::Point_2> vertices = polygon->vertices();
   
    // altitudine e lato hanno indice corrispondente nei due vector


    for (size_t i = 0; i < n_edges; i++) {
        edge = polygon->edge(i);
        // tuple<float, K::Point_2>
        auto [dist, point] = maxDistance(vertices, edge);
        altitudes.push_back(dist);
        corrispondent_points.push_back(point);
    }
        // cerco la minima altitudine
    float weigth = altitudes.at(0);
    int index = 0; // indice del lato corrispondente alla minima altitudine
    for (size_t i = 1; i < altitudes.size(); i++) {
        if (altitudes.at(i) < weigth) {
            weigth = altitudes.at(i);
            index = i;
        }
    }
 
        // la direzione di spazzata è la direzione di polygon.edge(index)
    return make_tuple(polygon->edge(index), corrispondent_points.at(index));
}

/*******************************************************/

// restituisce un vector di linee che creano una griglia sul poliigono
vector<CGAL::Line_2<K>> CoveragePathCreator::createGrid(CGAL::Segment_2<K> parallelEdge, K::Point_2 point) {

    cout << "createGrid()" << endl; 

    vector<CGAL::Line_2<K>> grid;

    // linea perpendicolare alla direzione di spazzata e passante per il punto più lontano del poligono
    CGAL::Line_2<K> ortogonal = parallelEdge.supporting_line().perpendicular(point);  
    // punto di proiezione del punto più lontano sul lato della direzione 
    auto projection = CGAL::intersection(ortogonal, parallelEdge.supporting_line());


    const K::Point_2 *p = boost::get<K::Point_2>(&*projection);
    
    // m_Helper.plotPoint(point, 'b', -1); 
    // m_Helper.plotPoint(*p, 'b', -1); 

    CGAL::Segment_2<K> h(*p, point);

    vector<K::Point_2> intersectionPoints = divideSegment(h, m_sweepDistance);

    // genero una linea con direzione parallelEdge.direction e che passa per il punto i-esimo
    for (size_t i = 0; i < intersectionPoints.size(); i++) {
        // m_Helper.plotPoint(intersectionPoints[i], 'r', -1); 
        CGAL::Line_2<K> l(intersectionPoints[i], parallelEdge.direction());
        grid.push_back(l);

    }

    return grid;
}


/*******************************************************/

vector<int> CoveragePathCreator::isToReduce(shared_ptr<CGAL::Polygon_2<K>> polygon, vector<bool> &isEdgeAdjacent) {
    
    cout << "isToReduce()" << endl;

    int B = isEdgeAdjacent.size();
    vector<int> to_ret;
    to_ret.resize(B); 

    for (int i = 0; i < B; i++) {

        int k = isCollinear(polygon, i); 

        if (k != -1) { //se è collineare riduco
            to_ret[i] = k ;
        }
        else if (isEdgeAdjacent[i]) {
            to_ret[i] = -1; //se non è collineare ma è c'è un'adiacenza => non riduco
        }
        else {
            to_ret[i] = -2;//non c'è adiacenza ==> riduco di metà sweepDistance (o 1/3?) 
        }
   
    }

    return to_ret; 
}

/*******************************************************/
shared_ptr<CGAL::Polygon_2<K>> CoveragePathCreator::reduceSubPolygon(shared_ptr<CGAL::Polygon_2<K>> polygon, vector<bool> &isEdgeAdjacent ) {
    
    cout << "reduceSubPolygon(), " << endl;
  
    double offset; //di quanto riduco 
    shared_ptr<CGAL::Polygon_2<K>> polygon_new = std::make_shared<CGAL::Polygon_2<K>>(*polygon); 

    size_t N = polygon->edges().size(); 

    for (int i = 0; i < N; i++) {
     

        // prendo il lato corrente
        CGAL::Segment_2<K> corr = polygon->edge(i); // lato corrente
    
        // prendo la perpendicolare
        CGAL::Line_2<K> perp(corr); 
        perp = perp.perpendicular(corr.source()); // perpendicolare al lato

        // trasformo in vettore la linea
        CGAL::Vector_2<K> v = perp.to_vector();
      
        // override!!!!!
        offset = (m_sweepDistance/2); 
        if (isEdgeAdjacent[i]) {
            continue;
            // offset = offset/2;
        }


        //spostamento del lato di offset 
        v = v / (sqrt(v.squared_length())); //vettore direzione 
        v = v * offset; //traslazione 
        K::Point_2 p = corr.source() + v; //punto a distanza corretta
        CGAL::Line_2<K> new_line(p, corr.direction()); //parallela al lato che passa per quel punto 

        vector<K::Point_2> a = intersect_convex_polygon_line(polygon_new, new_line); //interrsezione della retta con il poligono 


        //spostamento effettivo
        if (a.size() == 0) {
            cout << "reduceSubPolygon(): not found intersections" << endl; 
        }
   
        else {
            
            //aggiorno vertice i e vertice i+1
            //ho i due punti di intersezione a[0] e a[1] , devo capire quale dei due vertici spostare su quale dei due 

            // polygon_new->vertex(i) corrisponde ad edge(i).source() 
            // polygon_new->vertex(( i+1)%N) corrisponde a edge(i).target()

            double distance1 = CGAL::squared_distance(polygon_new->vertex(i), a[0]);
            double distance2 = CGAL::squared_distance(polygon_new->vertex(i), a[1]); 
            double distance3 = CGAL::squared_distance(polygon_new->vertex(( i+1)%N),a[0]); 
            double distance4 = CGAL::squared_distance(polygon_new->vertex(( i+1)%N),a[1]); 

            //una delle 4 deve essere per forza 
            if ( (distance1 <= distance2) && (distance4 <= distance3) ) { //vertex i è più vicino ad a[0] e vertex i+1 è più vicino ad a[1]
                polygon_new->vertex(i) = a[0]; 
                polygon_new->vertex((i+1)%N) = a[1]; 
            }
            else if ( (distance2 <= distance1 ) && (distance3 <= distance4) ){ //vertex i è più vicino ad a[1] e vertex i+1 è più vicino ad a[0]
                polygon_new->vertex(i) = a[1]; 
                polygon_new->vertex((i+1)%N) = a[0]; 
            }
            else if ( (distance1 <= distance2) && ( distance3 <= distance4) ) { //entrambi più vicini ad a[0] che ad a[1]
                //scelgo quello a minor distanza da a[0] per associarlo ad a[0]
                if (distance1 <= distance3) { //scelgo vertex i 
                    polygon_new->vertex(i) = a[0]; 
                    polygon_new->vertex((i+1)%N) = a[1]; 
                }
                else {
                    polygon_new->vertex(i) = a[1]; 
                    polygon_new->vertex((i+1)%N) = a[0];
                }
            }
            else { //entrambi più vicini ad a[1] che ad a[0]
                //scelgo quello a minor distanza da a[1] per associarlo ad a[1]
                if (distance2 <= distance4) {
                    polygon_new->vertex(i) = a[1]; 
                    polygon_new->vertex((i+1)%N) = a[0];
                }
                else {
                    polygon_new->vertex(i) = a[0]; 
                    polygon_new->vertex((i+1)%N) = a[1]; 
                }
            }
        }
    }
    
    m_Helper.plotPerimeterForTest(polygon_new, "ciao", true); 
    return polygon_new;

}        

/*******************************************************/

// ritorna i punti di intersezione tra la griglia e il poligono ristretto nelle adiacenze
vector<K::Point_2> CoveragePathCreator::generateGridForOnePolygon(shared_ptr<CGAL::Polygon_2<K>> polygon, vector<bool> &borders) {
    
    cout << "generateGridForOnePolygon() " << endl;

    //riduzione sottopoligono in corrispondenza delle adiacenze 
    shared_ptr<CGAL::Polygon_2<K>> restrictedPolygon = polygon;
    // shared_ptr<CGAL::Polygon_2<K>> restrictedPolygon = reduceSubPolygon(polygon, borders);

    //stampo poligono ristretto
    m_Helper.plotPerimeter(restrictedPolygon, "Perimeter", false);

    //trovo la direzione di spazzata 

    //ATTENZIONE EDGE HA SOURCE E TARGET COINCIDENTI, POINT È L'ORIGINE 
    auto [edge, point] = findSweepDirection(restrictedPolygon);

    // edge è il lato del poligono parallelo alla direzione di spazzata
    // point è il punto più lontano a quel lato

    // creo griglia
    vector<CGAL::Line_2<K>> grid = createGrid(edge, point);

    vector<K::Point_2> intersections; // intersezioni tra le sweep lines e il poligono

    //intersezioni di grid[0] con il poligono sono il source e il target del lato parallelo
    if (grid.size() != 0) {
        intersections.push_back(edge.source()); 
        intersections.push_back(edge.target()); 
    }

    for (size_t i = 1; i < grid.size(); i++) {

        vector<K::Point_2> a = intersect_convex_polygon_line(restrictedPolygon, grid.at(i));
        for (int j = 0 ; j < a.size(); j++) {
            m_Helper.plotPoint(a[j], 'b', -1); 
        }

        // o non interseca oppure interseca solo in un punto
        if (a.size() == 0) {
            cout << "generateGridForOnePolygon(): non trovate intersezioni tra griglia e sottopoligono ristretto" << endl;
        }
        //due intersezioni non coincidenti
        else if (a.size() > 1){
            intersections.push_back(a[0]);
            intersections.push_back(a[1]);
        }
        else {
            cout << "generateGridForOnePolygon(): trovata una sola intersezione tra griglia e sottopoligono ristretto" << endl;
        }
    }

        // affinché le intersezioni siano tutte direzionate allo stesso modo (es.prima sx poi dx)
    for (size_t i = 0; i < intersections.size(); i = i + 2) {

        K::Point_2 p;

        if (intersections[i].x() == intersections[i + 1].x())
        {
            if (intersections[i].hy() < intersections[i + 1].hy())
            {

                p = intersections[i + 1];
                intersections[i + 1] = intersections[i];
                intersections[i] = p;
            }
        }

        else if (intersections[i].x() > intersections[i + 1].x()) {

            p = intersections[i + 1];
            intersections[i + 1] = intersections[i];
            intersections[i] = p;
        }
    }

    return intersections;
}

/*******************************************************/
vector<CGAL::Segment_2<K>> CoveragePathCreator::generatePathForOnePolygon(vector<K::Point_2> intersections, int start) {

    cout << "generatePathForOnePolygon()" << endl;
    // start deve essere uno tra 0,1,n-1,n

    vector<CGAL::Segment_2<K>> path; 

    if (start < 2) {
        bool reverted = (start == 0) ? false : true;

        if (!reverted)
        {
            CGAL::Segment_2<K> initial_segment(intersections[0], intersections[1]);
            path.push_back(initial_segment);
        }
        else
        {
            CGAL::Segment_2<K> initial_segment(intersections[1], intersections[0]);
            path.push_back(initial_segment);
        }
        size_t num_intersections = intersections.size();

        for (size_t i = 2; i < num_intersections; i = i + 2)
        {
            // ora i punti sono ordinati giusti
            if (!reverted) {
                CGAL::Segment_2<K> link(intersections[i + 1], intersections[i - 1]);
                path.push_back(link);
                CGAL::Segment_2<K> segment(intersections[i + 1], intersections[i]);
                path.push_back(segment);
            }
            else
            {
                CGAL::Segment_2<K> link(intersections[i - 2], intersections[i]);
                path.push_back(link);
                CGAL::Segment_2<K> segment(intersections[i], intersections[i + 1]);
                path.push_back(segment);
            }
            reverted = !reverted;
        }
    }
    else
    {
        bool reverted = (start == 3) ? false : true;

        if (!reverted)
        {
            CGAL::Segment_2<K> initial_segment(intersections[intersections.size() - 1], intersections[intersections.size() - 2]);
            path.push_back(initial_segment);
        }
        else
        {
            CGAL::Segment_2<K> initial_segment(intersections[intersections.size() - 2], intersections[intersections.size() - 1]);
            path.push_back(initial_segment);
        }

        size_t num_intersections = intersections.size();
        for (size_t i = 2; i < num_intersections; i = i + 2)
        {
            if (!reverted)
            {
                CGAL::Segment_2<K> link(intersections[num_intersections - i], intersections[num_intersections - i - 2]);
                path.push_back(link);
                CGAL::Segment_2<K> segment(intersections[num_intersections - 2 - i], intersections[num_intersections - i - 1]);
                path.push_back(segment);
            }
            else
            {
                CGAL::Segment_2<K> link(intersections[num_intersections - i + 1], intersections[num_intersections - i - 1]);
                path.push_back(link);
                CGAL::Segment_2<K> segment(intersections[num_intersections - i - 1], intersections[num_intersections - i - 2]);
                path.push_back(segment);
            }
            reverted = !reverted;
        }
    }

    //non so perché ci siano punti duplicati, andrebbe risolto in altro modo ==> e comunque questo forse lo metto più alla fine 
    if (path.size() > 2) {
        for (size_t i = 0; i < path.size(); i++) {
            if (areEqual(path.at(i).source(), path.at(i).target())) {
                path.erase(path.begin()+i);
            }
        }
    }

    return path; // l'ultimo target del path è il punto dove finisce
}

/*******************************************************/

void CoveragePathCreator::generatePathForSubpolygons(){
    
    cout << "generatePathForSubpolygons()" << endl;

    //scelgo da quale punto partire del primo poligono ==> il più vicino ad initialVertex
    size_t n = m_intersections[0].size(); 
    double distance1 = CGAL::squared_distance(m_firstVertex, m_intersections[0][0]); 
    double distance2 = CGAL::squared_distance(m_firstVertex, m_intersections[0][1]); 
    double distance3 = CGAL::squared_distance(m_firstVertex, m_intersections[0][n-2]); 
    double distance4 = CGAL::squared_distance(m_firstVertex, m_intersections[0][n-1]); 
    int index = initialIndex(distance1, distance2, distance3, distance4); 

    //inserisco il path del primo poligono in m_pathS 
    m_pathS.push_back(generatePathForOnePolygon(m_intersections[0], index)); 
    //plot
    m_Helper.plotPathForConvexPolygon(m_pathS.at(0));

    if (m_NumerOfDecomposedSubPolygon == 1) {
        return; 
    }
    
    //occhio a questo in base a come gestisco i sottopolgioni degeneri
    // if (m_intersections.size()!= m_NumerOfDecomposedSubPolygon) { //m_intersections.size() è il numero di poligoni 
    //     cout << "generatePathForSubpoligons: errore" << endl; 
    // }

    for (size_t i = 0; i < m_NumerOfDecomposedSubPolygon; i++) {
        //verifica , m anon credo sia possibile entrare qua 
        if (m_intersections[i].size() == 0 /*|| m_intersections[i][0] == pointError*/) {
            if (m_ordinatedSubpolygonsForPath[i]->vertices().size() != 0) {             
                m_intersections[i][0] = m_ordinatedSubpolygonsForPath[i]->vertex(0); 
                m_intersections[i][1] = m_ordinatedSubpolygonsForPath[i]->vertex(0); 
            }
        }
    
    }

    //ora ci sono almeno 2 intersezioni per ogni sottopoligono e almeno 2 sottopoligoni 
    for (size_t i = 1 ; i < m_NumerOfDecomposedSubPolygon ; i++) {

        n = m_intersections[i].size(); // è almeno 2

        //if (m_intersections[i][0] != pointError) { //altrimenti non inserisco il path
        K::Point_2 last_point = m_intersections[i-1][m_intersections[i-1].size() - 1];
        
        //calcolo il punto dei 4 a cui collegarsi 
        distance1 = CGAL::squared_distance(last_point, m_intersections[i][0]); 
        distance2 = CGAL::squared_distance(last_point, m_intersections[i][1]); 
        distance3 = CGAL::squared_distance(last_point, m_intersections[i][n-2]); 
        distance4 = CGAL::squared_distance(last_point, m_intersections[i][n-1]); 
        index = initialIndex(distance1, distance2, distance3, distance4); 

        m_pathS.push_back(generatePathForOnePolygon(m_intersections[i], index));
        m_Helper.plotPathForConvexPolygon(m_pathS[m_pathS.size()-1]);
        //}
    }

}

/*******************************************************/
void CoveragePathCreator::joinAndLinkPaths(){

    cout << "joinAndLinkPaths()" << endl;
    
    int cont = 0; //serve come contatore per il plot con i numeri per seguire il percorso (+1 per ogni inizio e ogni fine di singolo path)
    
    //collego startingPoint con firstVertex 
    m_finalPath.push_back(CGAL::Segment_2<K>(m_startingPoint, m_firstVertex)); 

    //plot del punto di partenza 
    m_Helper.plotPoint(m_startingPoint, 'b', cont); 
    cont++; 
    
    m_Helper.plotPoint(m_firstVertex, 'b',  cont);
    

    size_t num_perimeterEdges = m_simplyfiedPolygon->edges().size();
    //cerco il lato del perimetro il cui source è firstVertex
    size_t indexFirstVertex = 0; 
    for (size_t i = 0; i < num_perimeterEdges ; i++) {
        if (m_simplyfiedPolygon->edge(i).source() == m_firstVertex) {
            indexFirstVertex = i; 
        }
    }

    //Inserisco nel path il perimetro (partendo e tornando in firstVertex)
    size_t j = indexFirstVertex;  
    for (size_t i = 0; i < num_perimeterEdges; i++) {
        m_finalPath.push_back(m_simplyfiedPolygon->edge(j)); 
        m_Helper.plotPartialPath(m_finalPath, -1);
        j = ((j+1)%num_perimeterEdges);
    }
    cont++; 

    //RIPRENDI DA QUA 

    // poi collego il punto iniziale al punto da cui si parte del primo sottopoligono
    m_finalPath.push_back(CGAL::Segment_2<K>(m_firstVertex, m_pathS[0][0].source()));
    m_Helper.plotPartialPath(m_finalPath, cont);
    cont++; 

    // poi tutti i path dei sottopoligoni
    for (size_t i = 0; i < m_pathS.size(); i++)
    {
        for (size_t j = 0; j < m_pathS[i].size(); j++)
        {
            m_finalPath.push_back(m_pathS[i][j]);
        }
        m_Helper.plotPartialPath(m_finalPath, -1); //disegna le strisciate, 


        // collego l'ultimo punto del path precedente col primo del successivo
        if (i != m_pathS.size() - 1)
        {

            CGAL::Segment_2<K> segment_new;
                
            float distance1, distance2; 
            int index; //può essere 0 o 1 

            vector<int> route = findMinRouteBetween(m_polygonsSorted[i], m_polygonsSorted[i+1]);

            // cout << "percorso tra il poligono " << m_polygonsSorted[i] << " e " << m_polygonsSorted[i+1] << endl;   
           
            for (size_t j  = 0; j < route.size()-1; j++) {

                if ((m_adj[route[j]][route[j+1]][0] == -1 && m_adj[route[j]][route[j+1]][1] == -1)) { // non sono adiacenti
                    cout << "CoveragePathCreator::cover error" << endl;
                    return;
                }
                
                // scelgo il punto dell'adiacenza a distanza minima
                distance1 = CGAL::squared_distance(m_pathS[i][m_pathS[i].size() - 1].target(),
                                                        m_decomposedVertices[m_adj[route[j]][route[j+1]][0] ]  ) +
                                    CGAL::squared_distance(m_decomposedVertices[m_adj[route[j]][route[j+1]][0]], m_pathS[i+1][0].source());
                

                distance2 = CGAL::squared_distance(m_pathS[i][m_pathS[i].size() - 1].target(),
                                                        m_decomposedVertices[m_adj[route[j]][route[j+1]][1]] ) +
                                CGAL::squared_distance(m_decomposedVertices[m_adj[route[j]][route[j+1]][1]], m_pathS[i+1][0].source());


                if (distance1 <= distance2)
                { // passo per il punto 1
                    index = 0;
                }
                else
                { // passo per il punto 2
                    index = 1;
                }

                //se non sono già collegati
                if (! ( m_finalPath.at(m_finalPath.size()-1).target() == m_decomposedVertices[m_adj[route[j]][route[j+1]][index]])) {
                    
                    segment_new = CGAL::Segment_2<K>( m_finalPath.at(m_finalPath.size()-1).target() , m_decomposedVertices[m_adj[route[j]][route[j+1]][index]]);
                  
                    m_finalPath.push_back(segment_new); // inserisco il primo collegamento : dal target del path i-esimo al vertice di collegamento 
                    m_Helper.plotPartialPath(m_finalPath, cont);
                    cont++; 
                }
            }           
            
            //unisco con il source del path i+1 esimo 
            segment_new = CGAL::Segment_2<K>( m_finalPath[m_finalPath.size()-1].target(), m_pathS[i+1][0].source());
            m_finalPath.push_back(segment_new); // inserisco il secondo pezzo
            m_Helper.plotPartialPath(m_finalPath, cont);
            cont++; 

        }
    }
}
/*******************************************************/


void CoveragePathCreator::createPathToReturn() {
    
    cout << "createPathToReturn()" << endl; 

    for (size_t i = 0; i < m_finalPath.size(); i++)
    {
        vector<K::Point_2> v = divideSegment(m_finalPath.at(i), 0.2);
        for (size_t j = 0; j < v.size(); j++)
        {
            m_pathToReturn.push_back(v.at(j));
        }
    }
}

/*******************************************************/

void CoveragePathCreator::eliminateExcessPoints(shared_ptr<CGAL::Polygon_2<K>>  polygon, vector<bool> &isAdjacent) {
    
    cout << "eliminateExcessPoints()" << endl;

    for (int i = 0; i < polygon->vertices().size(); i++) {
        size_t N = polygon->vertices().size(); 
        
        if ((isLeft(polygon->vertex( (i-1+N)%N), polygon->vertex(i), polygon->vertex((i+1)%N)) == 0 )){
            // remove vertex i              
            auto it = polygon->vertices_begin() + i;
            polygon->erase(it);
            bool removedEdgeisAdjacent = isAdjacent[i];
            if (!removedEdgeisAdjacent){
               isAdjacent[(i-1+N)%N] = removedEdgeisAdjacent;
            }

            auto it2 = isAdjacent.begin()+i; 
            isAdjacent.erase(it2); 
            i--;
        }
    }

    return;
}

/*******************************************************/

void CoveragePathCreator::generateGridsForSubpolygons(){
    
    cout << "generateGridsForSubpolygons()" << endl;

    // creazione di un vettore di appoggio
    vector<Polygon> partitionPolys_tmp; //non ordinato
    for (const Polygon &poly : m_decomposedPolysOfIndices)
    {
        partitionPolys_tmp.push_back(poly);
    }

    int polygonIndex;

    //creazione di un vettore ordinato secondo tsp di sottopoligoni CGAL::Polygon_2<K>
    for (size_t k = 0; k < m_NumerOfDecomposedSubPolygon; k++) {

        //ricavo l'indice del poligono che è il k-esimo nell'ordinamento
        polygonIndex = m_polygonsSorted.at(k); 
        
        //estrazione dei poligoni ordinata rispetto all'ordine di percorrenza 
        Polygon current = partitionPolys_tmp.at(polygonIndex); 
        
        vector<K::Point_2> tmp;
        for (Point p : current.container())
        {
            tmp.push_back(K::Point_2(m_decomposedVertices.at(p).x(), m_decomposedVertices.at(p).hy()));
        }
      
        m_ordinatedSubpolygonsForPath.push_back(createPolygon(tmp));
    }

    //per ogni sottopoligono ordinatamente creo il vettore borders, riduco il sottopoligono,  e creo la griglia
    for (size_t k = 0; k < m_NumerOfDecomposedSubPolygon; k++) {

        //estraggo ordinatamente i sottopolugoni (ordine di spazzata)
        polygonIndex = m_polygonsSorted.at(k); 
        shared_ptr<CGAL::Polygon_2<K>> curr = m_ordinatedSubpolygonsForPath.at(k); 
       

        // borders indica quali lati sono in comune e quindi bisogna lasciare lo spazio , inizializzo tutto a false
        vector<bool> isEdgeAdjacent(curr->edges().size(), false);   

        for (size_t i = 0; i < m_adj[polygonIndex].size(); i++) { //per ogni altro poligono

            if (m_adj[polygonIndex][i][0] != -1 && m_adj[polygonIndex][i][1] != -1) { //se è adiacente al poligono polygonIndex
                
                CGAL::Segment_2<K> seg(m_decomposedVertices.at(m_adj[polygonIndex][i][0]), m_decomposedVertices.at(m_adj[polygonIndex][i][1]));

                //cerco quale lato del poligono k corrisponde all'adiacenza 
                for (size_t j = 0; j < curr->edges().size(); j++) {                    
                    CGAL::Segment_2<K> edge = curr->edge(j); 
                    
                    //pongo isEdgeAdjacent = true se edge è completamente contenuto sul segmento dell'adiacenza 
                    if (seg.has_on(edge.source()) && seg.has_on(edge.target())){
                        isEdgeAdjacent[j] = true;
                    }
                
                }

            }
        }

        //eliminazione dei punti in eccesso dai sottopoligoni 
        eliminateExcessPoints(curr,isEdgeAdjacent); 

        // m_Helper.plotPerimeterForTest(curr, "prova", 1); 
        m_intersections.push_back(generateGridForOnePolygon(curr, isEdgeAdjacent));       

    }
}





/*******************************************************/

void CoveragePathCreator::cover() {
    cout << "cover()"<< endl; 
    generateGridsForSubpolygons(); 
    generatePathForSubpolygons();
    joinAndLinkPaths();
    createPathToReturn();
    
}


/*******************************************************/
vector<pair<float, float>> CoveragePathCreator::getFinalPath() {

    cout << "getFinalPath()" << endl; 

    vector<pair<float, float>> result;
  
    for (size_t i = 0; i < m_pathToReturn.size(); i++)
    {
        pair<float, float> p(m_pathToReturn.at(i).x(), m_pathToReturn.at(i).hy());
        result.push_back(p);
    }

    
    //test ottimizzazione
    float timeTurning = 1;
    int numberOfTurns = (int)(m_finalPath.size()-1);
    cout << "Numero di curve : " <<  numberOfTurns << endl;
    float speedRobot = 1;
    float lunghezza = 0;
    for (size_t i = 0 ; i < m_pathToReturn.size()-1; i++) {
        lunghezza = lunghezza + sqrt(CGAL::squared_distance(m_pathToReturn.at(i), m_pathToReturn.at(i+1)));
    }
    cout << "Lunghezza percorso: " << lunghezza << endl;
    cout << "Tempo complessivo: " << lunghezza/speedRobot + numberOfTurns/timeTurning << endl;

    return result;
}

/*******************************************************/

// approssimazione poligono
void CoveragePathCreator::simplifyPerimeter(){

    cout <<"simplifyPerimeter()"<< endl; 

    PS::Squared_distance_cost cost;
    CGAL::Polygon_2<K> approx = PS::simplify(*m_initialPolygon, cost, Stop((m_sweepDistance / 2) * (m_sweepDistance / 2)));
    m_simplyfiedPolygon = make_shared<CGAL::Polygon_2<K>>(approx);
    K::Point_2 firstVertexOld = m_simplyfiedPolygon->vertex(0); 
    m_simplyfiedPolygon->erase(m_simplyfiedPolygon->vertices_begin()); //tolgo il vertice iniziale 
    //lo metto come ultimo 
    m_simplyfiedPolygon->push_back(firstVertexOld); 
    //approssimo nuovamente 
    approx = PS::simplify(*m_simplyfiedPolygon, cost, Stop((m_sweepDistance / 2) * (m_sweepDistance / 2)));
    m_simplyfiedPolygon = make_shared<CGAL::Polygon_2<K>>(approx);

    // salvo i nuovi vertici
    for (size_t i = 0; i < m_simplyfiedPolygon->vertices().size(); i++)
    {
        m_simplyfiedVertices.push_back(m_simplyfiedPolygon->vertex(i));
    }

}



/*******************************************************/
bool CoveragePathCreator::run() {

    // plot perimetro iniziale
    m_Helper.plotPerimeter(m_initialPolygon, "Perimeter", false);

    //punto di partenza del robot 
    m_startingPoint = m_initialPerimeterVertices[0]; 

    //approssimazione perimetro
    simplifyPerimeter();

    //aggiornamento del'immagine del perimetro in Coverage Plot Helper
    m_Helper.updatePerimeterImage(m_simplyfiedPolygon, false);

    //decomposizione in sottopoligoni convessi 

    if (!decompose())
    {
        return false;
    }

    //m_firstVertex deve essere il più vicino al punto di partenza del robot tra i vertici del poligono decomposto 
    m_firstVertex = nearestPoint(m_startingPoint, m_simplyfiedVertices); 
    
    // creazione matrice di adiacenza dei sottopoligoni
    createAdjMatrix();

    // ordinamento sottopoligoni con TSP
    if (!orderSubPolygons())
    {
        return false;
    }


    // stampo i sottopoligoni ordinati
    vector<Polygon> partitionPolys_new; // vector di appoggio in cui inserisco i sottopoligoni
    for (const Polygon &poly : m_decomposedPolysOfIndices) {
        partitionPolys_new.push_back(poly);
    }
    for (size_t pol_i = 0; pol_i < partitionPolys_new.size(); pol_i++) {
        Polygon poly = partitionPolys_new.at(m_polygonsSorted[pol_i]);
        m_Helper.plotSubPolygon(poly, m_decomposedVertices, m_decompositionName, true , to_string(m_polygonsSorted[pol_i]) , true );
    }

    // creazione dei path per ogni sottopoligoni e unione
    cover();
    
    m_Helper.plotFinalPath(m_finalPath, m_pathToReturn , m_firstVertex, m_sweepDistance); 
    cv::waitKey(0);

    return true;
}

/*******************************************************/
