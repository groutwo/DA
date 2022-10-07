#define BUILDING_DEF_STRATEGY_LIB 1
#include "../simulador/Asedio.h"
#include "../simulador/Defense.h"
#include <algorithm>
#include <vector>
using namespace Asedio;

struct Data
{   int cost; float value; Defense* defense;
    bool operator<(const Data& d2) {return value<d2.value;}
    Data(int c=0, float v=0, Defense* d=nullptr): cost(c), value(v), defense(d) {}
};

float defAbility(Defense* defensa, const float &po)//po = proportionObstacles
{
    float damagePerSecond = defensa->damage * defensa->attacksPerSecond;//En torno a 5
    return ( damagePerSecond + defensa->range*(po<0.3?0.17:0.34)) / (0.01*defensa->cost) ;//Aprox 10/1, or 20/1
}                                                   //lots obtacles -> range more valued

void buyFirstDefense(std::list<Defense*> &defenses, unsigned int &ases, std::list<int> &selectedIDs)
{
    std::list<Defense*>::iterator defense0 = defenses.begin();
    selectedIDs.push_back((*defense0)->id);
    ases -= (*defense0)->cost;
    defenses.erase(defense0);
}

void evalueDefs(std::list<Defense*> &defenses, std::vector<Data> &vDatas, float proportionObstacles)
{
    int index = 0;
    for(auto const &it : defenses)
        vDatas[index++] = Data((it)->cost, defAbility(it, proportionObstacles), it)
        ///,std::cout<<"La defensa con id: "<<(it->id)<<" es "<<vDatas[index-1].value<<"\n"
        ;
}

void generateTSR(std::vector<std::vector<float> > &matValues, std::vector<Data> &vDatas, const unsigned int &ases)
{
    for(int j = 0; j <= ases; ++j)
        matValues[0][j] = (j < vDatas[0].cost ? 0: vDatas[0].value);

    for(int i = 1; i < vDatas.size(); ++i)
        for(int j = 0; j <= ases; ++j)
            matValues[i][j] = ( j < vDatas[i].cost ? matValues[i-1][j]:
            std::max(matValues[i-1][j], matValues[i-1][j-vDatas[i].cost] + vDatas[i].value) );
}

void bestCombinationDefs(std::vector<std::vector<float> > &matValues, std::vector<Data> &vDatas, std::list<int> &selectedIDs,  unsigned int &ases)
{
    for(int i = vDatas.size()-1; i > 0; i--)
        if(matValues[i][ases] != matValues[i-1][ases])
            ///std::cout<<"Metemos la defensa con id "<<vDatas[i].defense->id<<"\n",
            selectedIDs.push_back(vDatas[i].defense->id),
            ases -= vDatas[i].cost;
        
    if(matValues[0][ases] != 0)
        selectedIDs.push_back(vDatas[0].defense->id);
}

void DEF_LIB_EXPORTED selectDefenses(std::list<Defense*> defenses, unsigned int ases, std::list<int> &selectedIDs
            , float mapWidth, float mapHeight, std::list<Object*> obstacles)
{       
    buyFirstDefense(defenses, ases, selectedIDs);

    std::vector<Data> vDatas(defenses.size());
    evalueDefs(defenses, vDatas, (float)obstacles.size()/mapHeight*mapWidth);

    std::vector<std::vector<float> > matValues(vDatas.size(), std::vector<float>(ases+1));
    generateTSR(matValues, vDatas, ases);
    bestCombinationDefs(matValues, vDatas, selectedIDs, ases);
}