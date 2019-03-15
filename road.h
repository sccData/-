#ifndef _ROAD_H
#define _ROAD_H

#include "mylib.h"


using namespace std;

class Road
{
    friend class Road_Info;
    friend class Cross_Info;
    friend struct GuideMap;
private:
    typedef pair<int,int> Endpoint;
    int ID;
    int length;
    bool DoubleOrientation;
    int MaxSpeed;
    int NumberOfLane;
    Endpoint endpoints;

    struct NeighborRoad
    {
        int left = -1;
        int right = -1;
        int front = -1;
    };
    NeighborRoad first_point,second_point;

    unordered_map<int,int>  corresponding_cross;

public:
    Road() {};
    ~Road() {};


};


class Road_Info
{
    friend class Cross_Info;
    friend struct GuideMap;
private:
    int NumRoad;
    vector<Road> Roads;
    unordered_map<int,int> RoadMap;
public:
    Road_Info() {};
    ~Road_Info() {};

    void initialize(string filename);
    void get_road_neighbor(class Cross_Info &CrossInfo);
    int  get_corresponding_cross(int RoadID,int CrossID);
};


#endif