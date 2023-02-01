#include <stdlib.h>
#include <iostream>
#include <vector>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/partition_2.h>
#include <CGAL/Partition_traits_2.h>
#include "utils.hpp"
#include "CoveragePathCreator.h"

//deve funzionare con sweepdistance 0? 
//sistemare gli shared ptr
//controllare punto iniziale e collegamenti iniziali 
//quando il un "trapezio" il lato opposto a quello parallelo allo sweepLine non è parallelo a quel lato non viene integrato nel path 
//e non viene pulito
//quando non ci sono intersezioni prendi il centro invece che il primo  vertice
//in questo momento il collegamento tra due path di sottopoligoni si collega prima a un vertice di collegamento e poi all'altro, 
//ma non funziona bene questa cosa ==> modificare i collegamenti tra i path
//manca una strisciata a volte ==> riguardare come trova il punto a max distanza, forse non è divideSegment che si perde un punto, ma proprio dall'inizio 


using namespace std;

int main(int argc, char* argv[]) {
    
    //argomenti linea di comando
    if (argc < 4) { 
        cout << "Argument Error" << endl;
        // printInfo();
        return 1;
    }
    string filename = argv[1];
    
    //leggo input 
    vector<pair<float,float>> points = readFromFile(filename);


    //inizializzazione Coverage Path Creator 
    CoveragePathCreator coverage_path_creator;
    if (!coverage_path_creator.init(points, stof(argv[3]) , stoi(argv[2]))) {
        cout << "CoveragePathCreator: Errore inizializzazione" << endl;
        return 1;
    } 

    coverage_path_creator.run();

    points = coverage_path_creator.getFinalPath(); 

    return 0;
}
