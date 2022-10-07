#define BUILDING_DEF_STRATEGY_LIB 1

#include "../simulador/Asedio.h"
#include "../simulador/Defense.h"
#include "cronometro.h"


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

inline bool operator>(const Celda& c1, const Celda& c2)
{ return c2<c1; }

inline bool operator==(const Celda& c1, const Celda& c2)
{ return c1.value == c2.value; }

inline bool operator>=(const Celda& c1, const Celda& c2)
{ return !(c1<c2); }

float defaultCellValue(int row, int col, bool** freeCells, int nCellsWidth, int nCellsHeight
    , float mapWidth, float mapHeight, List<Object*> obstacles, List<Defense*> defenses)
{    	
    float cellWidth = mapWidth / nCellsWidth;
    float cellHeight = mapHeight / nCellsHeight;

    Vector3 cellPosition((col * cellWidth) + cellWidth * 0.5f, (row * cellHeight) + cellHeight * 0.5f, 0);
    	
    float val = 0;
    for (List<Object*>::iterator it=obstacles.begin(); it != obstacles.end(); ++it)
	    val += _distance(cellPosition, (*it)->position);

    return val;
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

Celda findAndRemoveWithoutOrder (std::vector<Celda>& celdas)
{
    float max = -999.0f;
    Celda celdaToReturn;
    std::vector<Celda>::iterator itBestValorated;
    for(std::vector<Celda>::iterator it = celdas.begin(); it!=celdas.end(); it++)
        if( it->value > max )
            max = it->value, itBestValorated = it, celdaToReturn = *it;
    celdas.erase(itBestValorated);
    
    return celdaToReturn;
}

void fusionAux(std::vector<Celda>& v, size_t i, size_t k, size_t j){
    std::vector<Celda> w(j-i+1);
    size_t n = j-i+1, p = i, q = k+1;
    for(int l = 0; l < n; ++l)
        w[l] = ((p <= k) and ((q > j) or (v[p] >= v[q])))? w[l] = v[p++]: v[q++];

    for (int l = 0; l < n; v[i+l] = w[l++]);
}
void sortByFusion(std::vector<Celda>& v, size_t i, size_t j){
    size_t n=j-i+1;
    if(n <= 2){
        if(v[i]<v[j])
            std::swap(v[i],v[j]);
    }else{
        size_t k = i + n/2;
        sortByFusion(v, i, k);
        sortByFusion(v, k+1, j);
        fusionAux(v, i, k, j);
    }
}

size_t pivot(std::vector<Celda>& v, size_t i, size_t j){
    size_t p = i;
    Celda x = v[i];
    for (size_t k = i+1; k <= j; ++k)
        if (v[k] >= x)
            ++p, std::swap(v[k], v[p]);
        
    v[i] = v[p];
    v[p] = x;
    return p;
}
void quickSort(std::vector<Celda>& v, size_t i, size_t j){
    size_t n = j-i+1;
    if (n<=2){
        if(v[i]<v[j])
            std::swap(v[i], v[j]);
    }else{
        size_t p = pivot(v, i, j);
        if(p>i)quickSort(v, i, p-1);
        if(p<j)quickSort(v, p+1, j);
    }
}

void cajaNegra()
{
    std::vector<Celda> start, copy;

    for(int i = 10; i<=50; i = i+10)
    {
        for(int j = i-1; j >= 0; --j)
            start.push_back(Celda(0,0,j)), copy.push_back(Celda(0,0,j));
        
        do{ sortByFusion(copy,0,i-1);
        } while (start==copy && std::next_permutation(copy.begin(),copy.end()));
    }
    std::cout << "fusión " << (start==copy ? "si":"no")<<" funciona." << std::endl;

    start.clear(),copy.clear();

    for(int i = 10; i<=50; i = i+10)
    {
        for(int j = i-1; j >= 0; --j)
            start.push_back(Celda(0,0,j)), copy.push_back(Celda(0,0,j));
        
        do{ quickSort(copy,0,i-1);
        } while (start==copy && std::next_permutation(copy.begin(),copy.end()));
    }
    std::cout << "ord rapida " << (start==copy ? "si":"no")<<" funciona." << std::endl;
}

void calculaTiempos(double &t1_sin, double &t2_fusion, double &t3_rapida, double &t4_monticulo,int id, bool** freeCells, int nCellsWidth,
                        int nCellsHeight, float mapWidth, float mapHeight, List<Object*> obstacles, List<Defense*> defenses )
{
    float cellWidth = mapWidth / nCellsWidth, cellHeight = mapHeight / nCellsHeight; 
    const double e_abs= 0.01, /*Maximo error absoluto cometido.*/ e_rel = 0.1; /*Maximo error relativo aceptado*/
    cronometro c;
    long int r = 0;
    c.activar(); 

    do {
        List<Defense*> colocadas, copyDefs(defenses);

        std::vector<Celda> Celdas;
            for( size_t x = 0; x< nCellsWidth; ++x)
                for(size_t y = 0; y< nCellsHeight; ++y)
                    if(freeCells[x][y])
                        Celdas.push_back(Celda(x,y, defaultCellValue(x, y, freeCells, nCellsWidth, nCellsHeight, mapWidth, mapHeight, obstacles, defenses)));
           

        size_t it=0, maxIt= defenses.size() * Celdas.size();
        while(defenses.size()!=0 and !Celdas.empty() and it<maxIt)
        {
//            if(id==1)
               Celda celda = findAndRemoveWithoutOrder(Celdas);
//            else
//            {
//                if(id==4)
//                    std::make_heap(Celdas.begin(),Celdas.end());
//                    
//                else if (id==2)
//                    sortByFusion(Celdas,0,Celdas.size());
//                else
//                   quickSort(Celdas,0,Celdas.size());
//
//                celda = Celdas.front(); Celdas.erase(Celdas.begin());
//            }  
            (*defenses.begin())->position = cellCenterToPosition(celda.x, celda.y, cellWidth,cellHeight);
            if(factible(*defenses.begin(), colocadas,obstacles, mapWidth, mapHeight)) 
                colocadas.push_back(*defenses.begin()), defenses.pop_front();
            it++;
        } 
        
        defenses = copyDefs;
        r++;
    } while(c.tiempo() < (e_abs / e_rel + e_abs));
    c.parar();

    switch (id)
    {
        case 1:
            t1_sin = c.tiempo()/r; break;
        case 2:
            t2_fusion = c.tiempo()/r; break;
        case 3:
            t3_rapida = c.tiempo()/r; break; 
        default:
            t4_monticulo = c.tiempo()/r;  break;
    }
    //if(id!=4)
     //       calculaTiempos(t1_sin, t2_fusion, t3_rapida, t4_monticulo, id+1, freeCells, nCellsWidth, nCellsHeight, mapWidth, mapHeight, obstacles, defenses);
}

void DEF_LIB_EXPORTED placeDefenses3(bool** freeCells, int nCellsWidth, int nCellsHeight, float mapWidth,
                     float mapHeight, List<Object*> obstacles, List<Defense*> defenses) 
{
    double t1_sin=10, t2_fusion=10, t3_rapida=10, t4_monticulo=10, tam = nCellsWidth * nCellsHeight;
    calculaTiempos(t1_sin, t2_fusion, t3_rapida, t4_monticulo, 1, freeCells, nCellsWidth, nCellsHeight, mapWidth, mapHeight, obstacles, defenses);

    std::cout<< tam << '\t' << t1_sin << '\t' << t2_fusion << '\t' << t3_rapida << '\t' << t4_monticulo <<std::endl;
}
