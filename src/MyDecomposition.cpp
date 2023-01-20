#include "MyDecomposition.h"

//se gli indici funzionano posso usare solo quelli? (di sicuro per il return)

MyDecomposition::MyDecomposition() {
}

/*******************************************************/

MyDecomposition::~MyDecomposition() {
}

/*******************************************************/


bool MyDecomposition::init(shared_ptr<CGAL::Polygon_2<K>> poly) { 
    //poligono da decomporre
    m_initialPolygon = poly; 
    //creo poligono iniziale di indici
    Polygon initial;
    for (size_t i = 0; i < m_initialPolygon->vertices().size(); i++) {
        m_vertices.push_back(m_initialPolygon->vertex(i));
        initial.push_back(i); 
    }
    //inserisco il poligono iniziale nella lista di poligoni di indici 
    m_indexedPolygons.push_back(initial);

    // //stampa indici per test 
    // for (const Polygon &pol : m_indexedPolygons) {
    //     cout << "poligono iniziale"<< endl;
    //     for (size_t i = 0; i < pol.size(); i++) {
    //         cout << pol.vertex(i) << endl;
    //     }
    // }

    return true;
}

/*******************************************************/

pair<Polygon_list, vector<K::Point_2> > MyDecomposition::getDecomposition() {
    return make_pair(m_indexedPolygons, m_vertices); 
}

/*******************************************************/

void MyDecomposition::run(){

      vector<shared_ptr<CGAL::Polygon_2<K>>> tmp_decomposition; //sono i due sottopoligoni in cui viene diviso un sottopoligono 
      Polygon_list tmp_indexedPolygons;
      Polygon indexedToCut; 

      shared_ptr<CGAL::Polygon_2<K>> toCut;
      pair<CGAL::Segment_2<K>, int> cutter; 

      m_decomposition.push_back(m_initialPolygon);
      //nella lista di Polygon di indici già c'è il primo 

      bool allConvex = false;

      while (!allConvex) {
            
            pair<int, int> p = allSubPolygonsConvex();
            
            if (p.first == -1) {// sono tutti convessi 
            allConvex = true;
            }
            else {

            toCut = m_decomposition.at(p.first);
            // cout << "taglio il poligono " << p.first << " in " << p.second << endl;

            // //trovo elemento corrispondente nella Polygonlist
            // indexedToCut = getIthIndexedPolygon(p.first);

            // std::cout << m_decomposition.size() << std::endl;
            // std::cout << m_indexedPolygons.size() << std::endl;
             
             
            //rimuovo dal vector il poligono da tagliare
            m_decomposition.erase(m_decomposition.begin() + p.first);
            //rimuovo dalla lista il corrispondente poligono di indici
            Polygon_list::iterator it = m_indexedPolygons.begin();
            advance(it,p.first);
            m_indexedPolygons.erase(it);

            //calcolo il segmento di taglio 
            cutter = calculateCutter(toCut , p.second);

            //taglio il poligono 
            /*,tmp_decompositionIdexes*/cutPolygon(toCut, cutter.first , p.second, cutter.second); 

            // //inserisco le due parti nel vector 
            // for (size_t j = 0; j < tmp_decomposition.size(); j++ ) { 
            //     m_decomposition.push_back(tmp_decomposition.at(j)); 
            //       //decompositionIndexes.pushBack(tmp_decompositionIdexes)
            // }                    
            // }
      }
      //sono tutti convessi 
    }
}

/*******************************************************/

//trovo elemento corrispondente nella Polygonlist
Polygon MyDecomposition::getIthIndexedPolygon(size_t i) {
    //si può ottenere più velocemente l'i-esimo elemento di una lista? 
    size_t cont = 0;
    Polygon toReturn;
    for (const Polygon &pol : m_indexedPolygons) {
        if (cont ==i) {
            toReturn = pol;
            break;
        }
        else {
            cont++;
        }
    }
    return toReturn; //se non lo trova può essere null
}
/*******************************************************/

bool MyDecomposition::isParallel(shared_ptr<CGAL::Polygon_2<K>> polygon, int edgeIndex) {
    
    bool to_return;
    size_t N = polygon->edges().size(); 

    if ( isLeft( polygon->edge((edgeIndex-1)%(N)).source(), polygon->edge(edgeIndex).source(), polygon->edge(edgeIndex).target()) == 0 ) {
        to_return = true; 
    }
    else if (isLeft(polygon->edge(edgeIndex).source(), polygon->edge(edgeIndex).target(), polygon->edge((edgeIndex+1)%N).target() ) == 0 ){
        to_return = true; 

    }
    else {
        to_return = false;
    }
    return to_return;
}

/*******************************************************/
int MyDecomposition::isLeft(K::Point_2 a, K::Point_2 b, K::Point_2 c){ //se l'angolo è di 180 viene circa 0 (perché non 0 ? ==> capire cosa fa questa funzione)
    
    float k = (b.hx() - a.hx())*(c.hy() - a.hy()) - (b.hy() - a.hy())*(c.hx() - a.hx());

    if ( k > 0.00000001) {
        return 1;
    }
    else if( k < -0.00000001){
        return -1;
    } 
    else{
        return 0;
    }

}


/*******************************************************/



//dato l'indice di un vertice CONCAVO calcola il segmento con cui tagliare il poligono 
pair<CGAL::Segment_2<K>, int> MyDecomposition::calculateCutter(shared_ptr<CGAL::Polygon_2<K>> poly, int &startingVertex) {
    

    size_t N = poly->vertices().size(); 

    //posso creare il nuovo lato come prolungamento di uno o dell'altro lato che comprendono quel vertice 
    //scelgo quello che renderebbe il nuovo segmento più corto  
    pair<vector<K::Point_2>, int> resultIntersection1 = intersect_polygon_line_2(poly, startingVertex);
    pair<vector<K::Point_2>, int> resultIntersection2 = intersect_polygon_line_2(poly, (startingVertex-1+N)%N);

    float d1,d2;

    //devo anche capire se il più vicino è il source o il target 
    d1 = CGAL::squared_distance(poly->edge(startingVertex).source(), resultIntersection1.first.at(0)); 
    d2 = CGAL::squared_distance(poly->edge((startingVertex-1+N)%N).target(), resultIntersection2.first.at(0)); 

    
    CGAL::Segment_2<K> cutter;
    int indexIntersectionEdge; 

    if (d1 <= d2) {
        cutter = CGAL::Segment_2<K> (poly->edge(startingVertex).source(), resultIntersection1.first.at(0));
        indexIntersectionEdge = resultIntersection1.second;
    }
    else {
        cutter = CGAL::Segment_2<K> (poly->edge((startingVertex-1+N)%N).target(), resultIntersection2.first.at(0)); 
        indexIntersectionEdge = resultIntersection2.second;
    }
 
    return make_pair(cutter, indexIntersectionEdge); 

}


/*******************************************************/

//restituisce il primo indice (del vertice) dove si trova una concavità . -1 se il poligono è convesso
size_t MyDecomposition::isConcave(shared_ptr<CGAL::Polygon_2<K>> perimeter) {
    vector<int> orientations;
    size_t N = perimeter->vertices().size();
    int index; //indice del primo punto di concavità
    for (size_t i = 0; i < N ; i++) {
        int k = isLeft(perimeter->vertex((i-1+N)%N), perimeter->vertex(i) , perimeter->vertex( (i+1)%N) );
        orientations.push_back(k);
    }

    // cout <<"ORIENTATION" << endl; 
    // for (size_t i = 0; i < orientations.size();i++) {
    //     cout << orientations.at(i) << endl;
    // }

    // controlla quale è la maggioranza (più positivi o più negativi ) e quali sono girati al contrario 
    int numPositives = 0; 
    int numNegatives = 0; 

    N = orientations.size();
    for (size_t i = 0; i < N; i++) {
        if (orientations.at(i) > 0) {
            numPositives++; 
        }
        else if (orientations.at(i) < 0){
            numNegatives++; 
        }
    }

    if (numPositives >= numNegatives) { //cerco il primo negativo 
        for (size_t i = 0; i < N; i++) {
            if (orientations.at(i) < 0) {
                index = i;
                break;
            }
        }
    }  
    else { //cerco il primo positivo 
        for (size_t i = 0; i < N; i++) {
            if (orientations.at(i) > 0) {
                index = i; 
                break;
            }
        }
    }
    if (numPositives == 0 || numNegatives == 0) {  //ovvero è convesso 
        index = -1;
    }
    return index; 
}

/*******************************************************/


//restituisce la posizione nel vector del primo poligono concavo e l'indice del vertice della convacità trovata , -1 il primo se sono tutti convessi 
pair<int, int> MyDecomposition::allSubPolygonsConvex () {
    
    int concavity;
    for (size_t i = 0; i < m_decomposition.size(); i++ ) {
        concavity = isConcave(m_decomposition.at(i));
        if ( concavity != -1) {
            return make_pair((int)i,concavity);
        }
    }
    //se sono tutti convessi 
    return make_pair(-1,-1);

} 

/*******************************************************/
//trova l'indice di p in m_vertices , -1 se non lo trova
int MyDecomposition::findIndex(K::Point_2 p) {
    for (size_t i = 0; i < m_vertices.size(); i++) {
        if (p == m_vertices.at(i)) {
            return (int)i;
        }
    }
    return -1;
}

/*******************************************************/

//divide in due un poligono tramite un nuovo segmento che parte da un vertice del poligono 
//sia poly che il corrispondente polugono di indici sono già stati rimossi rispettivamente dal vector e dalla lista
void MyDecomposition::cutPolygon(shared_ptr<CGAL::Polygon_2<K>> poly, /*Polygon indexededPoly,*/  CGAL::Segment_2<K> newEdge, int startVertex , int endEdge) {
    

    size_t N = poly->edges().size();
    
//     vector<shared_ptr<CGAL::Polygon_2<K>>> decomposition; 
    vector<K::Point_2> points1; 
    vector<K::Point_2> points2; 
    int cont; 

    //endEdge è l'indice del lato su cui sta il target del lato nuovo 
    //startEdge dove sta il source, il source è già un vertice di poly 
    //divido i vertici+il vertice nuovo in points1 e points2, vertici di un sottopoligono e vertici dell'altro 

    //newEdge->target è l'unico punto che non fa già parte dei vertici di poly
    Polygon indexedPoly1; 
    Polygon indexedPoly2; 
    int index;

    // cout << "start: " << startVertex << " end: " << endEdge << endl;    

    if (startVertex == endEdge || startVertex == (endEdge+1)%N || startVertex == (endEdge-1+N)%N )  { //non fa nulla 
        m_decomposition.push_back(poly); 
        for (size_t i = 0; i < poly->vertices().size(); i++ ){
            index = findIndex(poly->vertex(i)); 
            if (index == -1) {
                cout << "MyDecomposition: cutPolygon error" << endl;
                //qua ci metto un return? 
            }
            else {
                indexedPoly1.push_back(index);
            }
        }  
        m_indexedPolygons.push_back(indexedPoly1); 
        return; 
    }


    else if (startVertex < endEdge) {
        //points1
        points1.push_back(newEdge.source()); 
        for (int i = startVertex ; i < endEdge ; i++) {
            points1.push_back(poly->edge(i).target());
        }
        points1.push_back(newEdge.target());

        //points2
        points2.push_back(newEdge.target()); 
        cont = endEdge;
        while (cont != startVertex) {
            points2.push_back(poly->edge(cont).target());
            cont = (cont+1)%(N);            
        }
                
    }   

    else {
        //points1
        cont = startVertex%N; 
        while (cont != endEdge+1) {
            points1.push_back(poly->edge(cont).source());
            cont = (cont+1)%N;
        }
        points1.push_back(newEdge.target());

        //points2
        points2.push_back(newEdge.target()); 
        for (int i = endEdge; i < startVertex; i++) {
            points2.push_back(poly->edge(i).target()); 
        }
    }

    //newEdge.target() è il vertice nuovo quindi lo devo inserire in m_vertices 
    m_vertices.push_back(newEdge.target()); 


    for (size_t i = 0; i < points1.size(); i++ ){
        index = findIndex(points1.at(i)); 
        if (index == -1) {
            cout << "MyDecomposition: cutPolygon error" << endl;
            //qua ci metto un return? 
        }
        else {
            indexedPoly1.push_back(index);
        }
    }

    for (size_t i = 0; i < points2.size(); i++ ){
        index = findIndex(points2.at(i)); 
        if (index == -1) {
            cout << "MyDecomposition: cutPolygon error" << endl;
            //qua ci metto un return? 
        }
        else {
            indexedPoly2.push_back(index);
        }
    }
    
    //inserisco poligoni decomposti 
    m_decomposition.push_back(createPolygon(points1)); 
    m_decomposition.push_back(createPolygon(points2));

    //inserisco gli indici dei polugoni decomposti 
    m_indexedPolygons.push_back(indexedPoly1); 
    m_indexedPolygons.push_back(indexedPoly2);


    // //stampa indici per test 
    // int k =0; 
    // for (const Polygon &pol : m_indexedPolygons) {
    //     cout << "poligono " << k << endl;
    //     for (size_t i = 0; i < pol.size(); i++) {
    //         cout << pol.vertex(i) << endl;
    //     }
    //     k++;
    // }
    
    // cout << "POINTS1" << endl;
    // for (int k = 0; k < points1.size(); k++) {
    //             // m_Helper.plotPoint(points1.at(k)); 

    //     cout << points1.at(k) << endl;
    // }

    // cout << "POINTS2" << endl;
    // for (int k = 0; k < points2.size(); k++) {
    //     cout << points2.at(k) << endl;
    // }
}
