#define BUILDING_DEF_STRATEGY_LIB 1
#include "../simulador/Asedio.h"
#include "../simulador/Defense.h"
#ifdef PRINT_DEFENSE_STRATEGY
#include "ppm.h"
#endif
#ifdef CUSTOM_RAND_GENERATOR
RAND_TYPE SimpleRandomGenerator::a;
#endif

using namespace Asedio;

Vector3 cellCenterToPosition(int i, int j, float cellWidth, float cellHeight)
{ return Vector3((j * cellWidth) + cellWidth * 0.5f, (i * cellHeight) + cellHeight * 0.5f, 0); }

void positionToCell(const Vector3 pos, int &i_out, int &j_out, float cellWidth, float cellHeight)
{ i_out = (int)(pos.y * 1.0f / cellHeight); j_out = (int)(pos.x * 1.0f / cellWidth); }

struct Celda
{
    int x,y;
    float value;
    Celda() {}
    Celda(int x, int y, float value): x(x), y(y), value(value) {}
};

inline bool operator<(const Celda& c1, const Celda& c2)
{ return c1.value < c2.value; }

bool DefenseSort(Defense* d1, Defense* d2) 
{
    return (d1->range != d2->range? d1->range < d2->range:
    d1->dispersion*d1->damage < d2->dispersion*d2->damage);
}

float cellValue1(int row, int col, int nCellsWidth, int nCellsHeight, List<Object*> obstacles)
{
    float area= 0.3;
    float value =  //A mayor distancia del centro, menos valor
        (1/(1+_distance( Vector3(row, col,0), Vector3(nCellsWidth/2, nCellsHeight/2, 0))));
    
    List <Object*>::const_iterator itObsColocadas = obstacles.begin();
    int cont = 0;
    while(itObsColocadas != obstacles.end())
    { 
        if(_distance(Vector3(row, col,0), (*itObsColocadas)->position) >= area*nCellsWidth)
            cont++; //Una celda tiene más valor si (en un area del x% del tamaño del mapa) tiene algun obstáculo
        ++itObsColocadas;
    }
	return (value*cont); // valor segun lo centrico que sea, por el n de obstaculos en cierto rango
}

float cellValue2(int row, int col, Defense* def, float range, float cellWidth, float cellHeight, int nCellsWidth, int nCellsHeight,List<Object*> obstacles) 
{ 
    int i=0,j=0; float aux;
    range/=(2.5*(sqrt(cellWidth*cellHeight)));
    positionToCell(def->position,i,j,cellWidth,cellHeight);
    return ((aux=_distance( Vector3(row, col,0), Vector3(i, j,0)))>range)?aux:0;
}

bool factible(Defense* dActual, const List<Defense *>& colocadas, const List<Object*>& obstacles, float mapWidth, float mapHeight)
{
    bool esFactible = dActual->position.x + dActual->radio <= mapWidth && dActual->position.x-dActual->radio >=0 &&
        dActual->position.y + dActual->radio <= mapHeight && dActual->position.y-dActual->radio>=0;

    List <Defense*>::const_iterator itDefColocadas = colocadas.begin();
    while(esFactible && itDefColocadas != colocadas.end())
        esFactible = _distance(dActual->position, (*itDefColocadas)->position) >= dActual->radio + (*itDefColocadas)->radio,
        ++itDefColocadas;

    List <Object*>::const_iterator itObsColocadas = obstacles.begin();
    while(esFactible && itObsColocadas != obstacles.end())
        esFactible = _distance(dActual->position, (*itObsColocadas)->position) >= dActual->radio + (*itObsColocadas)->radio,
        ++itObsColocadas;

    return esFactible;
}

void DEF_LIB_EXPORTED placeDefenses(bool** freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight, List<Object*> obstacles, List<Defense*> defenses)
{
    float cellWidth = mapWidth / nCellsWidth;
    float cellHeight = mapHeight / nCellsHeight; 

    List<Defense*> colocadas;
    float aux, vMax=-1.0;
    Celda cPrometedora;
    size_t x = 0, y = 0;
    for( size_t x = 0; x< nCellsWidth; ++x)
        for(size_t y = 0; y< nCellsHeight; ++y)
            if((aux=cellValue1(x,y,nCellsWidth,nCellsHeight,obstacles)) > vMax)
            {
                float auxX = (*defenses.begin())->position.x, auxY=(*defenses.begin())->position.y;
                (*defenses.begin())->position = cellCenterToPosition(x, y,cellWidth,cellHeight);
                if(factible(*defenses.begin(), colocadas, obstacles, mapWidth, mapHeight))
                    vMax = aux,  cPrometedora = Celda(x,y,vMax);
                else
                    (*defenses.begin())->position.x=auxX, (*defenses.begin())->position.y= auxY;
            }   

    colocadas.push_back(*defenses.begin());
    Defense* centroExtraccion =  (*defenses.begin());
    defenses.pop_front();

    defenses.sort(DefenseSort);

    float rangemin = (*defenses.begin())->range;

    std::vector<Celda> vCeldas;
    for( size_t x = 0; x< nCellsWidth; ++x)
        for(size_t y = 0; y< nCellsWidth; ++y)
            vCeldas.push_back(Celda(x,y, cellValue2(x,y, centroExtraccion, rangemin, cellWidth, cellHeight, nCellsWidth,nCellsHeight,obstacles)));

    std::sort(vCeldas.begin(),vCeldas.end(),std::less<Celda>());

    size_t indice=0;

    while(defenses.size()>0)
    {
        Celda c = vCeldas[indice];
        (*defenses.begin())->position = cellCenterToPosition(c.x,c.y,cellWidth,cellHeight);
        if(factible(*defenses.begin(), colocadas, obstacles, mapWidth, mapHeight)) 
            colocadas.push_back(*defenses.begin()), defenses.pop_front();
        indice++;
    }
}