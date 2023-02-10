#include <stdlib.h>
#include <iostream>
#include <vector>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/partition_2.h>
#include <CGAL/Partition_traits_2.h>
#include "utils.hpp"
#include "CoveragePathCreator.h"


// ./coverage_path ../test/nometest.txt <decompositionType> <sweepDistance> <rangeToReturn> 
//deve funzionare con sweepdistance 0? 
//sistemare gli shared ptr
//quando non ci sono intersezioni prendi il centro invece che il primo  vertice
//non capisco perché a volte il perimetro esterno sembra più spesso 


//BUG: 
//./coverage_path ../test/area.txt 1 20
//./coverage_path ../test/complex.txt 1 1
//./coverage_path ../test/path2.txt 1 1 ==> manca una riga
//../test/pol1.txt 1 0.4 ==> crasha per openCV 

//ELIMINATI 
//joinAndLinkPaths()
//calculateAngle() 

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
    
    coverage_path_creator.run();

    points = coverage_path_creator.getFinalPath(); 

    return 0;
}
