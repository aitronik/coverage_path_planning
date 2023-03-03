#pragma once 

#include <stdlib.h>
#include <vector>
#include "utils.hpp"

class pathPoints {
    public:

        pathPoints();
        ~pathPoints();

        bool loadingOK;                      // booleano per verificare se i punti sono stati caricati o meno
        vector<pair<double,double>> points;    // punti del perimetro (caricati da un file .txt)

        bool init();

        void loadPoints(std::string file_to_load);    // funzione che carica i punti nell'attributo "points della classe"

};