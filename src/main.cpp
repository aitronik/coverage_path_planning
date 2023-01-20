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
//algoritmo per minimizzare i costi degli spostamenti tra i poligoni 
//rivedere distanza che lascio nelle adiacenze  
//SEGMENTATION FAULT IN REDUCE SUB POLYGON -> NON TROVA PARALLEL 
//rivedere il firstVertex, deve potersi scegliere? 

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
