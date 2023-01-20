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
        // case 1:
        //     CGAL::y_monotone_partition_2(polygon.vertices_begin(),
        //                                  polygon.vertices_end(),
        //                                  back_inserter(m_decomposedPolysOfIndices),
        //                                  traits);
        //     m_decompositionName = "Monotone partition";
        //     break;
        // case 2:
        //     CGAL::approx_convex_partition_2(polygon.vertices_begin(),
        //                                     polygon.vertices_end(),
        //                                     back_inserter(m_decomposedPolysOfIndices),
        //                                     traits);
        //     m_decompositionName = "Approx convex partition";
        //     break;
        // case 3:
        //     CGAL::greene_approx_convex_partition_2(polygon.vertices_begin(),
        //                                            polygon.vertices_end(),
        //                                            back_inserter(m_decomposedPolysOfIndices),
        //                                            traits);
                                                
        // //     break;

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
int CoveragePathCreator::indexOfMinimum(vector<float> &dist, bool *visited)
{
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

int CoveragePathCreator::initialIndex(float a, float b, float c, float d)
{ // si può fare in modo più furbo
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

int CoveragePathCreator::numAdiacency(int node)
{
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

    size_t N = m_adj.size();
    // creo matrice che abbia i pesi come elementi, al momento tutti 1 se i poligoni sono adiacenti
    vector<vector<int>> adj_new;

    for (size_t i = 0; i < N; i++)
    {
        vector<int> tmp;
        for (size_t j = 0; j < N; j++)
        {
            if (m_adj[i][j][0] != -1)
            { // il secondo può essere -1 e il primo no , ma non il contrario
                tmp.push_back(1);
            }
            else
            {
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
vector<int> CoveragePathCreator::findMinRoute(int start)
{

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
vector<int> CoveragePathCreator::findMinRoute(int start, int end) {

    // penso non sia possibile che non si possa andare da start ad end , sia perché parte di un poligono, che perché se siamo arrivati fino a chiamare questa funzione ,
    // il percorso minimo è già stato calcolato una volta

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

    // cerco qual è il poligono start, ovvero quello di cui fa parte il primo punto
    bool found = false;
    int start = -1;
    for (const Polygon &poly : m_decomposedPolysOfIndices)
    {
        if (found != false)
        { // già trovato uno
            break;
        }
        start++;
        for (size_t j = 0; j < poly.vertices().size(); j++)
        {
            if (m_decomposedVertices[poly.vertex(j)] == m_firstVertex)
            {
                found = true;
            }
        }
    }

    createAdjWeigthMatrix();
    vector<int> route = findMinRoute(start); // route è il vector dei precedenti -> all'indice i c'è il precedente del poligono i

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
    size_t n_edges = polygon->edges().size();
    K::Segment_2 edge;
    vector<float> altitudes;
    vector<K::Point_2> corrispondent_points;
    vector<K::Point_2> vertices = polygon->vertices();
    // float dist;
    // K::Point_2 point;
    // altitudine e lato hanno indice corrispondente nei due vector
    for (size_t i = 0; i < n_edges; i++)
    {
        edge = polygon->edge(i);
        // tuple<float, K::Point_2>
        auto [dist, point] = maxDistance(vertices, edge);
        altitudes.push_back(dist);
        corrispondent_points.push_back(point);
    }
    // cerco la minima altitudine
    float weigth = altitudes.at(0);
    int index = 0; // indice del lato corrispondente alla minima altitudine
    for (size_t i = 1; i < altitudes.size(); i++)
    {
        if (altitudes.at(i) < weigth)
        {
            weigth = altitudes.at(i);
            index = i;
        }
    }

    // la direzione di spazzata è la direzione di polygon.edge(index)
    return make_tuple(polygon->edge(index), corrispondent_points.at(index));
}

/*******************************************************/

// restituisce un vector di linee che creano una griglia sul poliigono
vector<CGAL::Line_2<K>> CoveragePathCreator::createGrid(CGAL::Segment_2<K> parallelEdge, K::Point_2 point)
{

    vector<CGAL::Line_2<K>> grid;

    CGAL::Line_2<K> ortogonal = parallelEdge.supporting_line().perpendicular(point); // linea perpendicolare alla direzione di spazzata e passante per il punto più lontano del poligono
    // m_Helper.plotPoint(point);
    // m_Helper.plotLineForTest(parallelEdge.supporting_line());
    // m_Helper.plotLineForTest(ortogonal); 
    
    auto projection = CGAL::intersection(ortogonal, parallelEdge.supporting_line()); // punto di proiezione del punto più lontano sul lato della direzione ==> è per forza un punto si ?

    const K::Point_2 *p = boost::get<K::Point_2>(&*projection);
    CGAL::Segment_2<K> h(*p, point);

    vector<K::Point_2> inters = divideSegment(h, m_sweepDistance);

    // genero una linea con direzione parallelEdge.direction e che passa per il punto i-esimo
    for (size_t i = 0; i < inters.size(); i++)
    {
        grid.push_back(CGAL::Line_2<K>(inters.at(i), parallelEdge.direction()));
    }

    return grid;
}




/*******************************************************/
shared_ptr<CGAL::Polygon_2<K>> CoveragePathCreator::reduceSubPolygon(shared_ptr<CGAL::Polygon_2<K>> polygon, vector<bool> &borders) {  
    
    cout << "reduceSubPolygon(), " << endl;

    size_t B = borders.size();
    
    vector<K::Point_2> vertices;

    shared_ptr<CGAL::Polygon_2<K>> reduced_polygon = polygon;
    // shared_ptr<CGAL::Polygon_2<K>> reduced_polygon = make_shared<CGAL::Polygon_2<K>>(*pol);

    // se ci sono adiacenze
    // creo un nuovo poligono con i lati spostati
    for (size_t i = 0; i < B; i++) {

        if (borders.at(i) != 0)
        { // lascio uno spazio diverso nelle adiacenze
            
            //se l'adiacenza è solo parte di un lato e non un lato intero non lascio spazio <=> è parallela al lato precedente o al successivo
            // if (false) {// 

            // se il lato da ridurre non è "parallelo a uno dei due lati a lui adiacenti"
            if (!MyDecomposition::isParallel(reduced_polygon, i)) {
                
                cout << "non parallelo" << endl;
                CGAL::Segment_2<K> corr = reduced_polygon->edge(i); // lato corrente

                CGAL::Line_2<K> perp(corr);
                perp = perp.perpendicular(corr.source()); // perpendicolare al lato
                CGAL::Vector_2<K> v = perp.to_vector();

                double angle1, angle2; //non possono venire 180 gradi perché lo ho escluso prima 

                if (i == 0)
                {
                    angle1 = calculateAngle(corr.to_vector(), reduced_polygon->edge(B - 1).to_vector());
                    angle2 = calculateAngle(corr.to_vector(), reduced_polygon->edge(i + 1).to_vector());
                }
                else if (i == B - 1)
                {
                    angle1 = calculateAngle(corr.to_vector(), reduced_polygon->edge(i - 1).to_vector());
                    angle2 = calculateAngle(corr.to_vector(), reduced_polygon->edge(0).to_vector());
                }
                else
                {
                    angle1 = calculateAngle(corr.to_vector(), reduced_polygon->edge(i - 1).to_vector());
                    angle2 = calculateAngle(corr.to_vector(), reduced_polygon->edge(i + 1).to_vector());
                }
                angle1 = acos(angle1);
                angle2 = acos(angle2);
                angle1 = sin(angle1);
                angle2 = sin(angle2);
                angle1 = min(angle1, angle2);

                double offset = (m_sweepDistance / 2) * angle1;


                K::Point_2 p = corr.source() + ((v / (sqrt(v.squared_length()))) * offset);
                CGAL::Line_2<K> new_line(p, corr.direction());
                K::Point_2 *a = intersect_polygon_line(reduced_polygon, new_line);


                for (K::Point_2 &p : reduced_polygon->container()) {
                    if (p == reduced_polygon->edge(i).source() || p == reduced_polygon->edge(i).target()) {
                       
                        if (CGAL::squared_distance(p, a[0]) < CGAL::squared_distance(p, a[1])) {
                            p = a[0];
                            // cout << "a[0]" << std::endl; 
                        }
                        else {
                            p = a[1];
                            // cout << "a[1]" << std::endl; 

                        }
                    }
                }
            }
        }
    }
    return reduced_polygon;
}


/*******************************************************/

// ritorna i punti di intersezione tra la griglia e il poligono ristretto nelle adiacenze
vector<K::Point_2> CoveragePathCreator::generateGridForOnePolygon(shared_ptr<CGAL::Polygon_2<K>> polygon, vector<bool> &borders)
{
    cout << "generateGridForOnePolygon() " << endl;
    //riduzione sottopoligono in corrispondenza delle adiacenze 
    shared_ptr<CGAL::Polygon_2<K>> restrictedPolygon = reduceSubPolygon(polygon, borders);

    m_Helper.plotPerimeter(restrictedPolygon);

    //trovo la direzione di spazzata 
    auto [edge, point] = findSweepDirection(restrictedPolygon);

    // edge è il lato parallelo alla direzione di spazzata
    // point è il punto più lontano a quel lato


    // creo griglia
    vector<CGAL::Line_2<K>> grid = createGrid(edge, point);

    vector<K::Point_2> intersections; // intersezioni tra le sweep lines e il poligono

    for (size_t i = 0; i < grid.size(); i++)
    {

        // cout << "cerco intersezione con linea " << i << endl;
        K::Point_2 *a = intersect_polygon_line(restrictedPolygon, grid.at(i));

        // o non interseca oppure interseca solo in un punto
        if (a[0].hx() == -1 || a[1].hx() == -1)
        {
            cout << a[0].hx() << ", " << a[0].hy() << "   " << a[1].hx() << ", " << a[1].hy() << endl;
        }
        else
        {
            if (a[0] != a[1])
            { // non inserisco quando ho due punti coincidenti
                intersections.push_back(a[0]);
                intersections.push_back(a[1]);
            }
        }
    }

    intersections.resize(intersections.size());
    // il primo il secondo , l'ultimo e il penultimo sono i 4 punti da cui si può partire

    // affinché le intersezioni siano tutte direzionate allo stesso modo (es.prima sx poi dx
    for (size_t i = 0; i < intersections.size(); i = i + 2)
    {

        K::Point_2 p;

        if (intersections[i].hx() == intersections[i + 1].hx())
        {
            if (intersections[i].hy() < intersections[i + 1].hy())
            {

                p = intersections[i + 1];
                intersections[i + 1] = intersections[i];
                intersections[i] = p;
            }
        }

        else if (intersections[i].hx() > intersections[i + 1].hx())
        {

            p = intersections[i + 1];
            intersections[i + 1] = intersections[i];
            intersections[i] = p;
        }
    }

    return intersections;
}

/*******************************************************/
vector<CGAL::Segment_2<K>> CoveragePathCreator::generatePathForOnePolygon(vector<K::Point_2> intersections, int start) {

    // start deve essere uno tra 0,1,n-1,n
    vector<CGAL::Segment_2<K>> path;

    if (start < 2)
    {
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
            if (!reverted)
            {
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

    return path; // l'ultimo target del path è il punto dove finisce
}

/*******************************************************/

void CoveragePathCreator::generatePathForSubpolygons(){
    
    cout << "generatePathForSubpolygons()" << endl;

    m_pathS.push_back(generatePathForOnePolygon(m_intersections.at(0), 0)); // parto dal punto 0 ==> volendo si può cambiare
    m_Helper.plotPathForConvexPolygon(m_pathS.at(0));

    for (size_t i = 1; i < m_intersections.size(); i++)
    { // per ogni poligono  (il numero poligoni potrei metterlo in una variabile)

        // scelgo il punto tra 0,1,N-1,N del poligono i+1 più vicino al punto N del poligono
        size_t n = m_intersections.at(i).size(); // numero punti intersezioni poligono corrente
        K::Point_2 last_point = m_intersections.at(i - 1).at(m_intersections.at(i - 1).size() - 1);
        int ind = initialIndex(CGAL::squared_distance(last_point, m_intersections.at(i).at(0)), CGAL::squared_distance(last_point, m_intersections.at(i).at(1)),
                               CGAL::squared_distance(last_point, m_intersections.at(i).at(n - 2)),
                               CGAL::squared_distance(last_point, m_intersections.at(i).at(n - 1)));

        m_pathS.push_back(generatePathForOnePolygon(m_intersections.at(i), ind));
        m_Helper.plotPathForConvexPolygon(m_pathS.at(i));
    }
}

/*******************************************************/
void CoveragePathCreator::joinAndLinkPaths(){
    cout << "joinAndLinkPaths()" << endl;
    // Parto inserendo nel path il perimetro (m_firstVertex dovrebbe essere il source del lato 0 , quindi dovrebbe partire e tornare lì)
    for (size_t i = 0; i < m_simplyfiedPolygon->edges().size(); i++)
    {
        m_finalPath.push_back(m_simplyfiedPolygon->edge(i));
    }

    m_Helper.plotPoint(m_firstVertex);
    m_Helper.plotPartialPath(m_finalPath); // blu il perimetro 
    // poi collego il punto iniziale al punto da cui si parte del primo sottopoligono
    m_finalPath.push_back(CGAL::Segment_2<K>(m_firstVertex, m_pathS.at(0).at(0).source()));
    m_Helper.plotPartialPath(m_finalPath);

    // poi tutti i path dei sottopoligoni
    for (size_t i = 0; i < m_pathS.size(); i++)
    {
        for (size_t j = 0; j < m_pathS.at(i).size(); j++)
        {
            m_finalPath.push_back(m_pathS.at(i).at(j));
        }
        m_Helper.plotPartialPath(m_finalPath); //rosse le strisciate 


        // collego l'ultimo punto del path precedente col primo del successivo
        if (i != m_pathS.size() - 1)
        {

            CGAL::Segment_2<K> segment_new;
            
            m_polygonsSorted.resize(m_polygonsSorted.size());
    
            float distance1, distance2; 
            int index; //può essere 0 o 1 

            vector<int> route = findMinRoute(m_polygonsSorted[i], m_polygonsSorted[i+1]);

            
            for ( size_t j = 0; j < route.size()-1; j++) {

                if ((m_adj[route[j]][route[j+1]][0] == -1 && m_adj[route[j]][route[j+1]][1] == -1)) { // non sono adiacenti
                    cout << "CoveragePathCreator::cover error" << endl;
                    return;
                }
                
                // scelgo il punto dell'adiacenza a distanza minima
                distance1 = CGAL::squared_distance(m_pathS.at(i).at(m_pathS.at(i).size() - 1).target(),
                                                        m_decomposedVertices[m_adj[route[j]][route[j+1]][0] ]  ) +
                                    CGAL::squared_distance(m_decomposedVertices[m_adj[route[j]][route[j+1]][0]], m_pathS.at(i+1).at(0).source());
                

                distance2 = CGAL::squared_distance(m_pathS.at(i).at(m_pathS.at(i).size() - 1).target(),
                                                        m_decomposedVertices[m_adj[route[j]][route[j+1]][1]] ) +
                                CGAL::squared_distance(m_decomposedVertices[m_adj[route[j]][route[j+1]][1]], m_pathS.at(i+1).at(0).source());


                if (distance1 <= distance2)
                { // passo per il punto 1
                    index = 0;
                }
                else
                { // passo per il punto 2
                    index = 1;
                }

                if (! ( m_pathS.at(i).at(m_pathS.at(i).size() - 1).target() == m_decomposedVertices[m_adj[route[j]][route[j+1]][index]] )) {
                    segment_new = CGAL::Segment_2<K>(m_pathS.at(i).at(m_pathS.at(i).size() - 1).target(), m_decomposedVertices[m_adj[route[j]][route[j+1]][index]]);
                    m_finalPath.push_back(segment_new); // inserisco il primo pezzo
                    m_Helper.plotPartialPath(m_finalPath);
                }

                if (m_decomposedVertices[m_adj[route[j]][route[j+1]][index]] == m_pathS.at(i+1).at(0).source()) {
                    continue; 
                }

                segment_new = CGAL::Segment_2<K>(m_decomposedVertices[m_adj[route[j]][route[j+1]][index]], m_pathS.at(i+1).at(0).source());
                m_finalPath.push_back(segment_new); // inserisco il secondo pezzo
                m_Helper.plotPartialPath(m_finalPath);

            }                
        }
    }
}
/*******************************************************/


void CoveragePathCreator::createPathToReturn() {
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
            tmp.push_back(K::Point_2(m_decomposedVertices.at(p).hx(), m_decomposedVertices.at(p).hy()));
        }
      
        m_ordinatedSubpolygonsForPath.push_back(createPolygon(tmp));
    }

    //per ogni sottopoligono ordinatamente creo il vettore borders, riduco il sottopoligono,  e creo la griglia
    for (size_t k = 0; k < m_NumerOfDecomposedSubPolygon; k++) {

        polygonIndex = m_polygonsSorted.at(k); 
        shared_ptr<CGAL::Polygon_2<K>> current = m_ordinatedSubpolygonsForPath.at(k); 

        // borders indica quali lati sono in comune e quindi bisogna lasciare lo spazio , inizializzo tutto a false
        vector<bool> borders(current->edges().size(), false);   


        // metto a true i lati che hanno adiacenza

        for (size_t i = 0; i < m_adj[polygonIndex].size(); i++) { //per ogni altro poligono

            if (m_adj[polygonIndex][i][0] != -1 || m_adj[polygonIndex][i][1] != -1) { //se è adiacente al poligono polygonIndex
                
                CGAL::Segment_2<K> seg(m_decomposedVertices.at(m_adj[polygonIndex][i][0]), m_decomposedVertices.at(m_adj[polygonIndex][i][1]));
                
                //cerco quale lato del poligono k corrisponde all'adiacenza 
                for (size_t j = 0; j < current->edges().size(); j++) {                    

                    if ((seg.source() == current->edge(j).source() && seg.target() == current->edge(j).target()) || 
                            (seg.source() == current->edge(j).target() && seg.target() == current->edge(j).source())) {

                        borders[j] = true;
                    }
                }

            }
        }

        m_intersections.push_back(generateGridForOnePolygon(current, borders));       

    }
}





/*******************************************************/

void CoveragePathCreator::cover() {
    generateGridsForSubpolygons(); 
    generatePathForSubpolygons();
    joinAndLinkPaths();
    createPathToReturn();
    
}


/*******************************************************/
vector<pair<float, float>> CoveragePathCreator::getFinalPath()
{
    vector<pair<float, float>> result;
    for (size_t i = 0; i < m_pathToReturn.size(); i++)
    {
        pair<float, float> p(m_pathToReturn.at(i).hx(), m_pathToReturn.at(i).hy());
        result.push_back(p);
    }
    return result;
}

/*******************************************************/

// approssimazione poligono
void CoveragePathCreator::simplifyPerimeter(){
    PS::Squared_distance_cost cost;
    CGAL::Polygon_2<K> approx = PS::simplify(*m_initialPolygon, cost, Stop((m_sweepDistance / 2) * (m_sweepDistance / 2)));
    m_simplyfiedPolygon = make_shared<CGAL::Polygon_2<K>>(approx);

    // salvo i nuovi vertici
    for (size_t i = 0; i < m_simplyfiedPolygon->vertices().size(); i++)
    {
        m_simplyfiedVertices.push_back(m_simplyfiedPolygon->vertex(i));
    }

}



/*******************************************************/
bool CoveragePathCreator::run()
{

    // plot perimetro iniziale
    m_Helper.plotPerimeter(m_initialPolygon);

    //approssimazione perimetro
    simplifyPerimeter();
    
    //aggiornamento del'immagine del perimetro in Coverage Plot Helper
    m_Helper.updatePerimeterImage(m_simplyfiedPolygon);

    //decomposizione in sottopoligoni convessi 

    if (!decompose())
    {
        return false;
    }

    //questa cosa si può cambiare 
    m_firstVertex = m_decomposedVertices.at(0);

    // creazione matrice di adiacenza dei sottopoligoni
    createAdjMatrix();

    // ordinamento sottopoligoni con TSP
    if (!orderSubPolygons())
    {
        return false;
    }

    // stampo i sottopoligoni ordinati
    vector<Polygon> partitionPolys_new; // vector di appoggio in cui inserisco i sottopoligoni
    for (const Polygon &poly : m_decomposedPolysOfIndices)
    {
        partitionPolys_new.push_back(poly);
    }
    for (size_t pol_i = 0; pol_i < partitionPolys_new.size(); pol_i++)
    {
        Polygon poly = partitionPolys_new.at(m_polygonsSorted.at(pol_i));
        m_Helper.plotSubPolygon(poly, m_decomposedVertices, pol_i, m_decompositionName);
    }

    // creazione dei path per ogni sottopoligoni e unione
    cover();
    
    m_Helper.plotFinalPath(m_finalPath, m_pathToReturn , m_firstVertex); 
    cv::waitKey(0);

    return true;
}

/*******************************************************/
