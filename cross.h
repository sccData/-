#ifndef _CROSS_H
#define _CROSS_H

#include "mylib.h"
#include "road.h"
using namespace std;


enum direction {unknown = -1,up = 0, right = 1,down = 2,left = 3};

class Cross
{
    friend class Cross_Info;
    friend struct GuideMap;
    friend class Road_Info;
private:
    typedef pair<int,int> coordinate;
    int ID;
    coordinate position;
    bool Iscalculated = false;
    int NeiborRoad[4] = {-1,-1,-1,-1};
    int DirectNeiborRoad[4] = {-1,-1,-1,-1};
    direction first_direction = unknown;

public:
    Cross(){};
    ~Cross(){};
};

class Cross_Info
{
    friend struct GuideMap;
    friend class Road_Info;
private:
    int NumCross;
    vector<Cross> Crosses;

    int **trav_cost;
    void get_orentation(Road_Info &RoadInfo);
    void get_coordinate(Road_Info &RoadInfo);
    void get_directed_neighbor();
    void set_trave_cost_Matrix(Road_Info &RoadInfo);    //设置遍历成本矩阵
public:
    unordered_map<int,int> CrossMap;
    Cross_Info(){trav_cost = nullptr;};
    ~Cross_Info();
    void initialize(string filename,Road_Info &RoadInfo);
    int total_cross_num() {return NumCross;}
};


#endif