#define PRINT_PATHS 1 // ###### Config options ################

#define BUILDING_DEF_STRATEGY_LIB 1
#include "../simulador/Asedio.h"
#include "../simulador/Defense.h"

#ifdef PRINT_PATHS
    #include "ppm.h"
#endif

using namespace Asedio;

Vector3 cellCenterToPosition(int i, int j, float cellWidth, float cellHeight)
{ return Vector3((j * cellWidth) + cellWidth * 0.5f, (i * cellHeight) + cellHeight * 0.5f, 0); }

void DEF_LIB_EXPORTED calculateAdditionalCost(float** additionalCost, int cellsWidth, int cellsHeight,
                float mapWidth, float mapHeight , List<Object*> obstacles, List<Defense*> defenses) 
{
    float cellWidth = mapWidth / cellsWidth, cellHeight = mapHeight / cellsHeight, aux;
    for(int i = 0 ; i < cellsHeight ; ++i)
        for(int j = 0 ; j < cellsWidth ; ++j)
        {
            for(auto& o: obstacles)
                if(aux = _distance(cellCenterToPosition(i,j,cellWidth,cellHeight),o->position) >= o->radio)
                    additionalCost[i][j]*=(aux*o->radio);
            for(auto d: defenses)
                if(aux = _distance(cellCenterToPosition(i,j,cellWidth,cellHeight),d->position) >= d->radio)
                    additionalCost[i][j]*=(aux*d->radio*d->range*d->range*0.3333f);
        }
}

bool belongs(std::list<AStarNode*> list2find, AStarNode* node) 
{ return (std::find(list2find.begin(),list2find.end(), node) == list2find.end()); }

float estimatedDistance(AStarNode* c, AStarNode* t, float** ac, float ch, float cw)
{ return _sdistance(c->position, t->position) + ac [(int)(c->position.y / ch)] [(int)(c->position.x / cw)]; }

void DEF_LIB_EXPORTED calculatePath(AStarNode* originNode, AStarNode* targetNode, int cellsWidth, int cellsHeight,
                                float mapWidth, float mapHeight, float** additionalCost, std::list<Vector3> &path)
{
    float cellWidth = mapWidth / cellsWidth, cellHeight = mapHeight / cellsHeight;

    std::list<AStarNode*> open, closed; AStarNode* cur = originNode;
    cur->G=0; cur->H = estimatedDistance(cur, targetNode, additionalCost, cellHeight, cellWidth); path.clear();
    cur->F=cur->G+cur->H;
    open.push_back(cur);
    while(cur != targetNode && open.size()>0)
    {   cur = *open.begin(); open.pop_front();
        closed.push_back(cur);
        for (auto j : cur->adjacents )
            if(j != NULL && !belongs(closed, j))
            {   if(!belongs(open, j))
                {
                    j->parent = cur; j->G = cur->G +_sdistance(cur->position, j->position);
                    j->H = estimatedDistance(j, targetNode, additionalCost, cellHeight, cellWidth);
                    j->F = j->G + j->H;
                    open.push_back(j);
                }
                else
                {   float d = _sdistance(cur->position, j->position);
                    if(j->G > cur->G + d)
                    {   open.erase(std::find(open.begin(),open.end(), j));
                        j->parent = cur; j->G = cur->G + d; j->F = j->G + j->H;
                        open.push_back(j);
                    }
                }
            }   
    }
    cur = targetNode;
    while ( cur != NULL && cur->parent != originNode)
        path.push_front(cur->position),
        cur = cur->parent;
}

/*
    class AStarNode {
    public:
        List<AStarNode*> adjacents; L[i] 
        Vector3 position;           
        AStarNode* parent;          P[i]
        float F, G, H;
    //F=G+H; G=dis_min(start, current), H=dis_estimated(current, end)
        AStarNode() : parent(NULL), F(0), G(0), H(0) {};
    };  

    L -> ady list node i
    P -> path
*/