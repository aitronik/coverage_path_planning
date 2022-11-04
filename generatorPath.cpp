// #include "generatorPath.hpp"
// using namespace std; 

// float angularCoeff(Segment segment) {  //CALCOLA BENE
//     Point target = segment.target();
//     Point source = segment.source();
//     return (target.hy() - source.hy()) / ( target.hx() - source.hx() );
// }

// float qCoeff(Segment segment, float m) { //return coefficiente q di y = mx + q
//     Point target, source;
//     target = segment.target();
//     source = segment.source();
//     float q = m;
//     q = q * source.hx();
//     return source.hy()-q;
// }

// // Point* findPointsAtDistance(float m, float q, Point p, float dx) { //l'eq è di secondo grado quindi ne trova 2
// //     float b = ( m*q - 2*m*(p.hy()) - 2*p.hx() );
// //     float a = (m*m -1);
// //     float c = q*q + p.hy()*p.hy() -2*q*p.hy() - p.hx()*p.hx() - dx;
// //     float deltaSq = sqrt(b*b - 4*a*c);
// //     float x1 = (-b + deltaSq) /(2*a);
// //     float x2 = (-b -deltaSq) /(2*a);
// //     Point p1(x1, m*(x1)+q);
// //     Point p2(x2, m*(x2)+q);
// //     static Point points[2];
// //     points[0] = p1;
// //     points[1] = p2;
// //     return points;
// // }    

// // //tra i due risultati dell'eq-ne sceglie il più vicino al target 
// // Point choosePoint (Point points[2], Point target) {
// //     if ( sqrt(squared_distance(points[0], target)) >= sqrt(squared_distance(points[1], target))) return points[1];
// //     else return points[0];
// // }

// // vector<Point> divideSegment(Segment segment, float sweep_distance){
// //     float length = sqrt(segment.squared_length());
// //     cout << "length segment : " << length << endl;
// //     cout << "sweep_distance: " << sweep_distance << endl;
// //     int num_points = length/sweep_distance +1 ; //intero inferiore ==> l'ultimo pezzo sarà più lungo AND 5 pezzi ==> 6 punti 
// //     cout << "num_points: " << num_points << endl;
// //     vector <Point> points;
// //     Point target = segment.target();
// //     Point source = segment.source();
// //     points.resize(num_points);
// //     points[0] = source;
// //     //devo trovare le coordinate dei punti ==> ho bisogno del coefficiente angolare del segmento
// //     float m = angularCoeff(segment); //se ci fosse modo si potrebbe fare direttamente sulla retta
// //     float q = qCoeff(segment, m);
// //     for (int i = 1; i < num_points -1; i++) {
        
// //         Point* tmp = findPointsAtDistance(m, q, points[i-1], sweep_distance);
// //         //ne trova 2, qual è? 
// //         //quello più vicino a target tra i due 
// //         points[i] = choosePoint(tmp, target);

// //     }

// //     points[num_points-1] = target;
// //     return points;
// // }

// vector<Point> divideSegment(Segment segment, float sweep_distance) {
    
//     //creo il vettore 
//     Point source = segment.source();
//     Point target = segment.target();
//     Vector v(source, target);
   

   
//     float length = sqrt(v.squared_length());
//     // //in quanti punti lo divido
//     // int num_points = (length/sweep_distance) +1 ;

//     //creazione del path 
//     vector<Point> path;

//     Point next = source;
//     int i = 0;

//     while (segment.collinear_has_on(next)) {
//         // printPoint(next);
//         path.push_back(next);
//         i++;
//         next = source + ( (v/length)  * sweep_distance * i);  //  v/length dovrebbe essere il vettore direzione 
//     }
//     path.push_back(target);

//     return path;
// }



// void generatePath(shared_ptr<Polygon> poly, vector<Point>& final) {

//     vector<Point> vertices = extractVertices(poly); //vettore dei vertici
//     // a due a due creo il segmento 
//     vector<Segment> segments;
//     //lunghezze dei segmenti
//     // vector<float> lengths;
    
//     Point current;
//     Point old = vertices.at(0);


//     for (int i = 1; i <vertices.size(); i++) {
//         current = vertices.at(i);
//         Segment seg(current,old);
//         segments.push_back(seg);
//         // lengths.push_back(sqrt(seg.squared_length()));
//         old = current;
//     }
//     Segment seg(current, vertices.at(0)); //non so come fare senza ridefinirlo ogni volta
//     segments.push_back(seg);
//     // lengths.push_back(sqrt(seg.squared_length()));


//     float sweep_distance = 20; //questo sweep_distance poi va dato dall'esterno 
//   //  final = divideSegment(segments.at(0), sweep_distance);
//     for (int i = 0 ; i < segments.size() ; i++) {
//         vector<Point> tmp = divideSegment(segments.at(i), sweep_distance);
//         final.insert(final.end(), tmp.begin(), tmp.end());
//     }
// }