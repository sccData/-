#ifndef _MAP_H
#define _MAP_H

#include "mylib.h"
#include "cross.h"
using namespace std;

struct Choice
{
    int MyChoice[5] = {-1,-1,-1,-1,-1};
};



struct GuideMap
{
    vector< vector<Choice> > MapMatrix;
    int **min_cost;
    int ***shortest_path;
    int map_size;
    GuideMap(Cross_Info &CrossInfo) :min_cost(nullptr),shortest_path(nullptr),map_size(CrossInfo.NumCross){ MapMatrix.resize(map_size+1,vector<Choice>(map_size+1)); }
    void calculate_map(Cross_Info &CrossInfo,Road_Info &RoadInfo);
    void dijkstra(Cross_Info &CrossInfo);
    ~GuideMap();
};

#endif