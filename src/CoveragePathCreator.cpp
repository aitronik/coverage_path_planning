#include "CoveragePathCreator.h"

CoveragePathCreator::CoveragePathCreator()
{
    // parametri di default
    m_doPlotting = true;
    m_addPerimeterToPath = true;
}

/*******************************************************/

CoveragePathCreator::~CoveragePathCreator()
{
}

/*******************************************************/

bool CoveragePathCreator::init(vector<pair<float, float>> points, float sweepDistance, int decompositionType)
{

    vector<K::Point_2> perimeter_vertices;
    for (size_t i = 0; i < points.size(); i++)
    {
        K::Point_2 p(points.at(i).first, points.at(i).second);
        perimeter_vertices.push_back(p);
    }

    // creazione del poligono iniziale
    m_perimeterVertices = perimeter_vertices;
    m_perimeter = createPolygon(m_perimeterVertices);
    m_decompositionType = decompositionType;
    m_sweepDistance = sweepDistance;
    m_firstVertex = perimeter_vertices.at(0); // il punto di partenza è il primo punto trai vertici dati

    // tipo di decomosizione
    if (m_decompositionType != 0 && m_decompositionType != 1 && m_decompositionType != 2 && m_decompositionType != 3)
    {
        cout << "CoveragePathCreator: decomposition type error" << endl;
        return false;
    }

    m_decompositionType = m_decompositionType;

    // inizializzazione dell'plot helper
    if (m_doPlotting == true)
    {
        if (!m_Helper.init(perimeter_vertices))
        {
            return false;
        }
    }

    return true;
}

/*******************************************************/

// decompone il poligono iniziale in sottopoligoni convessi
bool CoveragePathCreator::decompose()
{

    Partition_traits_2 traits(CGAL::make_property_map(m_perimeterVertices));
    Polygon polygon;
    for (size_t i = 0; i < m_perimeterVertices.size(); i++)
    {
        polygon.push_back(i);
    }

    // in base al tipo di decomposizione
    switch (m_decompositionType)
    {
    case 0:
        CGAL::optimal_convex_partition_2(polygon.vertices_begin(),
                                         polygon.vertices_end(),
                                         back_inserter(m_partitionPolys),
                                         traits);
        m_decompositionName = "Optimal convex partition";
        break;
    case 1:
        CGAL::y_monotone_partition_2(polygon.vertices_begin(),
                                     polygon.vertices_end(),
                                     back_inserter(m_partitionPolys),
                                     traits);
        m_decompositionName = "Monotone partition";
        break;
    case 2:
        CGAL::approx_convex_partition_2(polygon.vertices_begin(),
                                        polygon.vertices_end(),
                                        back_inserter(m_partitionPolys),
                                        traits);
        m_decompositionName = "Approx convex partition";
        break;
    case 3:
        CGAL::greene_approx_convex_partition_2(polygon.vertices_begin(),
                                               polygon.vertices_end(),
                                               back_inserter(m_partitionPolys),
                                               traits);
        break;
    default:
        cout << "CoveragePathCreator: decomposition error" << endl;
        return false;
    }

    // controllo decomposizione
    if (!CGAL::partition_is_valid_2(polygon.vertices_begin(), polygon.vertices_end(), m_partitionPolys.begin(), m_partitionPolys.end(), traits))
    {
        cout << "CoveragePathCreator: decomposition error" << endl;
        return false;
    }

    return true;
}

/*******************************************************/

void CoveragePathCreator::setAddPerimeterToPath(bool b) {
    m_addPerimeterToPath = b;
}

/*******************************************************/
// crea una matrice di adiacenza in cui gli elementi sono coppie di indici di punti che rappresentano un'adiacenza tra due sottopoligoni (suppongo siano convessi)
void CoveragePathCreator::createAdjMatrix() {
     
    size_t k = m_partitionPolys.size();
    m_adj.resize(k);
    for (size_t i = 0; i < k; i++)
    {
        m_adj.at(i).resize(k);
        for (size_t j = 0; j < k; j++)
        {
            m_adj.at(i).at(j).resize(2);
        }
    }

    int cont_i = 0;
    int cont_j = 0;

    for (const Polygon &poly1 : m_partitionPolys)
    {
        for (const Polygon &poly2 : m_partitionPolys)
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
void CoveragePathCreator::findMinRoute(int start, int end) {

    // cout << "da " << start << "a " << end << endl;
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
    // for (int i = 0; i < route.size(); i++) {
    //     cout << route.at(i) << endl;
    // } 
}


/*******************************************************/

bool CoveragePathCreator::orderSubPolygons()
{ // si può aggiungere l'indice di start che al momento è sempre 0 nella chiamata a findMinRoute

    // cerco qual è il poligono start, ovvero quello di cui fa parte il primo punto
    bool found = false;
    int start = -1;
    for (const Polygon &poly : m_partitionPolys)
    {
        if (found != false)
        { // già trovato uno
            break;
        }
        start++;
        for (size_t j = 0; j < poly.vertices().size(); j++)
        {
            if (m_perimeterVertices[poly.vertex(j)] == m_firstVertex)
            {
                found = true;
            }
        }
    }

    createAdjWeigthMatrix();
    vector<int> route = findMinRoute(start); // route è il vector dei precedenti -> all'indice i c'è il precedente del poligono i

    // cout << "route size: " << route.size() << endl;
    // for (int i = 0; i < route.size(); i++) {
    //     cout << route.at(i) << endl;
    // }

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
vector<CGAL::Line_2<K>> CoveragePathCreator::createGrid(CGAL::Segment_2<K> sweepDirection, K::Point_2 point)
{

    vector<CGAL::Line_2<K>> grid;

    CGAL::Line_2<K> ortogonal = sweepDirection.supporting_line().perpendicular(point); // linea perpendicolare alla direzione di spazzata e passante per il punto più lontano del poligono

    auto projection = CGAL::intersection(ortogonal, sweepDirection.supporting_line()); // punto di proiezione del punto più lontano sul lato della direzione ==> è per forza un punto si ?

    const K::Point_2 *p = boost::get<K::Point_2>(&*projection);
    CGAL::Segment_2<K> h(*p, point);

    vector<K::Point_2> inters = divideSegment(h, m_sweepDistance);

    // genero una linea con direzione sweepDirection.direction e che passa per il punto i-esimo
    for (size_t i = 0; i < inters.size(); i++)
    {
        grid.push_back(CGAL::Line_2<K>(inters.at(i), sweepDirection.direction()));
    }

    return grid;
}

/*******************************************************/

// ritorna i punti di intersezione tra la griglia e il poligono ristretto nelle adiacenze
vector<K::Point_2> CoveragePathCreator::generateGridForOnePolygon(int cont, vector<bool> &borders)
{

    borders.resize(borders.size());
    vector<K::Point_2> vertices;

    shared_ptr<CGAL::Polygon_2<K>> pol = m_polygonsForPath.at(cont);
    shared_ptr<CGAL::Polygon_2<K>> polygon_new = make_shared<CGAL::Polygon_2<K>>(*pol);

    // se ci sono adiacenze
    // creo un nuovo poligono con i lati spostati
    for (size_t i = 0; i < borders.size(); i++)
    {

        if (borders[i] != 0)
        { // lascio uno spazio diverso nelle adiacenze

            CGAL::Segment_2<K> corr = polygon_new->edge(i); // lato corrente

            CGAL::Line_2<K> perp(corr);
            perp = perp.perpendicular(corr.source()); // perpendicolare al lato
            CGAL::Vector_2<K> v = perp.to_vector();

            double angle1, angle2;
            if (i == 0)
            {
                angle1 = calculateAngle(corr.to_vector(), polygon_new->edge(borders.size() - 1).to_vector());
                angle2 = calculateAngle(corr.to_vector(), polygon_new->edge(i + 1).to_vector());
            }
            else if (i == borders.size() - 1)
            {
                angle1 = calculateAngle(corr.to_vector(), polygon_new->edge(i - 1).to_vector());
                angle2 = calculateAngle(corr.to_vector(), polygon_new->edge(0).to_vector());
            }
            else
            {
                angle1 = calculateAngle(corr.to_vector(), polygon_new->edge(i - 1).to_vector());
                angle2 = calculateAngle(corr.to_vector(), polygon_new->edge(i + 1).to_vector());
            }
            angle1 = acos(angle1);
            angle2 = acos(angle2);
            angle1 = sin(angle1);
            angle2 = sin(angle2);
            angle1 = min(angle1, angle2);

            double offset = (m_sweepDistance / 2) * angle1;

            K::Point_2 p = corr.source() + ((v / (sqrt(v.squared_length()))) * offset);
            CGAL::Line_2<K> new_line(p, corr.direction());
            K::Point_2 *a = intersect_polygon_line(polygon_new, new_line);

            for (K::Point_2 &p : polygon_new->container())
            {
                if (p == polygon_new->edge(i).source() || p == polygon_new->edge(i).target())
                {

                    if (CGAL::squared_distance(p, a[0]) < CGAL::squared_distance(p, a[1]))
                    {
                        p = a[0];
                    }
                    else
                    {
                        p = a[1];
                    }
                }
            }
        }
    }

    // plot poligono ridotto per test
    // m_Helper.plotCoveredPerimeter(polygon_new);

    auto [edge, point] = findSweepDirection(polygon_new); // questo dovrebbe funzionare bene
    // edge è il lato parallelo alla direzione di spazzata
    // point è il punto più lontano a quel lato

    // creo griglia
    vector<CGAL::Line_2<K>> grid = createGrid(edge, point);

    vector<K::Point_2> intersections; // intersezioni tra le sweep lines e il poligono

    for (size_t i = 0; i < grid.size(); i++)
    {

        // cout << "cerco intersezione con linea " << i << endl;
        K::Point_2 *a = intersect_polygon_line(polygon_new, grid.at(i));

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
vector<CGAL::Segment_2<K>> CoveragePathCreator::generatePathForOnePolygon(vector<K::Point_2> intersections, int start)
{ // start è il punto da cui partire

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

void CoveragePathCreator::cover()
{

    // vector di appoggio
    vector<Polygon> partitionPolys_new;
    for (const Polygon &poly : m_partitionPolys)
    {
        partitionPolys_new.push_back(poly);
    }

    int cont = 0;
    for (size_t pol_i = 0; pol_i < partitionPolys_new.size(); pol_i++)
    {

        Polygon poly = partitionPolys_new.at(m_polygonsSorted.at(pol_i));

        vector<K::Point_2> tmp;
        for (Point p : poly.container())
        {
            tmp.push_back(K::Point_2(m_perimeterVertices[p].hx(), m_perimeterVertices[p].hy()));
        }
        m_polygonsForPath.push_back(createPolygon(tmp));

        // borders indica quali lati sono in comune e quindi bisogna lasciare lo spazio , inizializzo tutto a 0
        vector<bool> borders;
        borders.resize(m_polygonsForPath.at(cont)->edges().size(), false);

        // metto a 1 i lati che hanno adiacenza
        for (size_t i = 0; i < m_adj[cont].size(); i++)
        {
            if (m_adj[cont][i][0] != -1 && m_adj[cont][i][1] != -1)
            {

                CGAL::Segment_2<K> seg(m_perimeterVertices[m_adj[cont][i][0]], m_perimeterVertices[m_adj[cont][i][1]]);

                for (size_t j = 0; j < m_polygonsForPath.at(cont)->edges().size(); j++)
                {

                    if ((seg.source() == m_polygonsForPath.at(cont)->edge(j).source() && seg.target() == m_polygonsForPath.at(cont)->edge(j).target()) || (seg.source() == m_polygonsForPath.at(cont)->edge(j).target() && seg.target() == m_polygonsForPath.at(cont)->edge(j).source()))
                    {
                        borders[j] = 1;
                    }

                }
            }
        }

        vector<K::Point_2> intersections = generateGridForOnePolygon(cont, borders);
        // scegli a quale punto unire

        m_intersections.push_back(generateGridForOnePolygon(cont, borders));
        cont++;
    }

    m_pathS.push_back(generatePathForOnePolygon(m_intersections.at(0), 0)); // parto dal punto 0 ==> volendo si può cambiare
    m_Helper.plotPathForConvexPolygon(m_pathS.at(0) /*, m_polygonsForPath.at(0) */);

    for (size_t i = 1; i < m_intersections.size(); i++)
    { // per ogni poligono  (il numero poligoni potrei metterlo in una variabile)

        // scelgo il punto tra 0,1,N-1,N del poligono i+1 più vicino al punto N del poligono
        size_t n = m_intersections.at(i).size(); // numero punti intersezioni poligono corrente
        K::Point_2 last_point = m_intersections.at(i - 1).at(m_intersections.at(i - 1).size() - 1);
        int ind = initialIndex(CGAL::squared_distance(last_point, m_intersections.at(i).at(0)), CGAL::squared_distance(last_point, m_intersections.at(i).at(1)),
                               CGAL::squared_distance(last_point, m_intersections.at(i).at(n - 2)),
                               CGAL::squared_distance(last_point, m_intersections.at(i).at(n - 1)));

        m_pathS.push_back(generatePathForOnePolygon(m_intersections.at(i), ind));
        m_Helper.plotPathForConvexPolygon(m_pathS.at(i) /*, m_polygonsForPath.at(i) */);
    }


    // unisco i path

    // Parto inserendo nel path il perimetro (m_firstVertex dovrebbe essere il source del lato 0 , quindi dovrebbe partire e tornare lì)
    for (size_t i = 0; i < m_simplyfiedPerimeter->edges().size(); i++)
    {
        m_finalPath.push_back(m_simplyfiedPerimeter->edge(i));
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
                                                        m_perimeterVertices[m_adj[route[j]][route[j+1]][0] ]  ) +
                                    CGAL::squared_distance(m_perimeterVertices[m_adj[route[j]][route[j+1]][0]], m_pathS.at(i+1).at(0).source());
                

                distance2 = CGAL::squared_distance(m_pathS.at(i).at(m_pathS.at(i).size() - 1).target(),
                                                        m_perimeterVertices[m_adj[route[j]][route[j+1]][1]] ) +
                                CGAL::squared_distance(m_perimeterVertices[m_adj[route[j]][route[j+1]][1]], m_pathS.at(i+1).at(0).source());


                if (distance1 <= distance2)
                { // passo per il punto 1
                    index = 0;
                }
                else
                { // passo per il punto 2
                    index = 1;
                }



                //stampo per debug
                int x, y;
                for (int k = 0; k < m_polygonsSorted.size(); k++) {
                    if (m_polygonsSorted[k] == route.at(j)  ) {
                        x = k;
                    }
                    else if (m_polygonsSorted[k] == route.at(j+1)  ) {
                        y = k;
                    }
                }
                cout << "unendo " << x << "con " << y << endl;



                if (! ( m_pathS.at(i).at(m_pathS.at(i).size() - 1).target() == m_perimeterVertices[m_adj[route[j]][route[j+1]][index]] )) {
                    segment_new = CGAL::Segment_2<K>(m_pathS.at(i).at(m_pathS.at(i).size() - 1).target(), m_perimeterVertices[m_adj[route[j]][route[j+1]][index]]);
                    m_finalPath.push_back(segment_new); // inserisco il primo pezzo
                    m_Helper.plotPartialPath(m_finalPath);
                }

                if (m_perimeterVertices[m_adj[route[j]][route[j+1]][index]] == m_pathS.at(i+1).at(0).source()) {
                    continue; 
                }

                segment_new = CGAL::Segment_2<K>(m_perimeterVertices[m_adj[route[j]][route[j+1]][index]], m_pathS.at(i+1).at(0).source());
                m_finalPath.push_back(segment_new); // inserisco il secondo pezzo
                m_Helper.plotPartialPath(m_finalPath);

            }                
        }
    }


    // m_finalPath è il path finale in segment

    // // inserendo prima il perimetro
    // for (size_t i = 0; i < m_simplyfiedPerimeter->edges().size(); i++)
    // {
    //     vector<K::Point_2> v = divideSegment(m_simplyfiedPerimeter->edge(i), 0.2);
    //     for (size_t j = 0; j < v.size() - 1; j++)
    //     { // il -1 è per non passare due volte sul punto finale del lato perché è lo stesso del primo punto del successivo
    //         m_pathToReturn.push_back(v.at(j));
    //     }
    // }

    // l'ultimo punto di questo sarà il source di approximatePolygon->edge(0) , da qui dovrebbe partire il path (oppure bisogna scegliere il primo lato su cui girare in base a questo )
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
bool CoveragePathCreator::run()
{

    // plot perimetro
    m_Helper.plotPerimeter(m_perimeter);

    // approssimazione poligono
    PS::Squared_distance_cost cost;
    CGAL::Polygon_2<K> approx = PS::simplify(*m_perimeter, cost, Stop((m_sweepDistance / 2) * (m_sweepDistance / 2)));
    m_simplyfiedPerimeter = make_shared<CGAL::Polygon_2<K>>(approx);

    // aggiorno vertici salvati
    m_perimeterVertices.clear();
    for (size_t i = 0; i < m_simplyfiedPerimeter->vertices().size(); i++)
    {
        m_perimeterVertices.push_back(m_simplyfiedPerimeter->vertex(i));
    }

    // plot nuovo perimetro
    m_Helper.updatePerimeterImage(m_simplyfiedPerimeter);

    // decomposizione
    if (!decompose())
    {
        return false;
    }

    // creazione matrice di adiacenza dei sottopoligoni
    createAdjMatrix();

    // ordinamento sottopoligoni con TSP
    if (!orderSubPolygons())
    {
        return false;
    }

    // stampo i sottopoligoni ordinati
    vector<Polygon> partitionPolys_new; // vector di appoggio in cui inserisco i sottopoligoni
    for (const Polygon &poly : m_partitionPolys)
    {
        partitionPolys_new.push_back(poly);
    }
    for (size_t pol_i = 0; pol_i < partitionPolys_new.size(); pol_i++)
    {
        Polygon poly = partitionPolys_new.at(m_polygonsSorted.at(pol_i));
        m_Helper.plotSubPolygon(poly, m_perimeterVertices, pol_i, m_decompositionName);
    }

    // creazione dei path per ogni sottopoligoni e unione
    cover();

    cv::waitKey(0);

    return true;
}

/*******************************************************/
