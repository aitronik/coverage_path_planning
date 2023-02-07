#include "pathPoints.h"

pathPoints::pathPoints()
{
    loadingOK = false;
}

pathPoints::~pathPoints()
{
}

bool pathPoints::init()
{

    return true;

}

void pathPoints::loadPoints(std::string file_to_load)
{

    points = readFromFile(file_to_load);

    // controllo che i punti siano stati caricati correttamente facendo un check sulla lunghezza del vettore
    if(points.size() == 0){
        loadingOK = false;
    }
    else {
        loadingOK = true;
    }
    

}