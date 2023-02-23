#include <stdlib.h>
#include <iostream>
#include <vector>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/partition_2.h>
#include <CGAL/Partition_traits_2.h>
#include "utils.hpp"
#include "CoveragePathCreator.h"
#include <exception>

// ./coverage_path ../test/nometest.txt <decompositionType> <sweepDistance>  
//sistemare gli shared ptr
//quando non ci sono intersezioni prendi il centro invece che il primo  vertice
//non so se funziona con coordinate negative 

//TO DO: 
//2.
//Se due lati non adiacenti di un poligono ridotto si intersecano ("fiocco") inserire il punto di intersezione tra i vertici 
//e togliere gli altri due 


using namespace std;

int main(int argc, char* argv[]) {
    
    //argomenti linea di comando
    if (argc < 4) { 
        cout << "Argument Error" << endl;
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

    //qui eventualmente aggiungere setAddPerimeterToPath e setRangeToReturn per modificarne i valori 
    try {
        coverage_path_creator.run();
        points = coverage_path_creator.getFinalPath(); 

    }
    catch (...){
        cout << "Error " << endl;
    }


    return 0;
}
