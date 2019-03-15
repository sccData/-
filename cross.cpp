#include "cross.h"

void Cross_Info::initialize(string filename,Road_Info &RoadInfo)
{
    ifstream fin;
    string useless;
    char HeadCharacter;
    int count = 0;

    fin.open(filename.c_str());

    if(!fin.is_open())
    {
        cerr<<"Error! Cannot open Cross file!\n";
        exit(-1);
    }

    while(!fin.eof())
    {
        HeadCharacter = fin.get();
        if(HeadCharacter == '#')
            getline(fin,useless);
        else if(HeadCharacter == '(') {
            Cross temp;
            fin >> temp.ID;
            fin >> useless;
            fin >> temp.NeiborRoad[0];
            fin >> useless;
            fin >> temp.NeiborRoad[1];
            fin >> useless;
            fin >> temp.NeiborRoad[2];
            fin >> useless;
            fin >> temp.NeiborRoad[3];
            fin >> useless;
            getline(fin, useless);
            Crosses.push_back(temp);
            CrossMap.insert(make_pair(temp.ID, count));
            count++;
        }
    }
    NumCross = count;

    fin.close();

    trav_cost = new int*[NumCross];
    trav_cost[0] = new int[NumCross*NumCross];
    for(int i = 1;i<NumCross;++i)
        trav_cost[i] = trav_cost[i-1] + NumCross;


    set_trave_cost_Matrix(RoadInfo);
    get_orentation(RoadInfo);
    get_coordinate(RoadInfo);
    get_directed_neighbor();
    return;
}


Cross_Info::~Cross_Info() {
    if(trav_cost != nullptr)
    {
        delete[] trav_cost[0];
        delete[] trav_cost;
    }
}

void Cross_Info::get_orentation(Road_Info &RoadInfo)
{
    queue<int> TraversalSet;
    Crosses[0].first_direction = up;
    TraversalSet.push(Crosses[0].ID);
    while(!TraversalSet.empty())
    {
        int current_cross_ID;
        int current_road_ID;
        int next_cross_ID;
        current_cross_ID = TraversalSet.front();
        TraversalSet.pop();
        for(int i = 0;i<4;i++)
        {
            current_road_ID = Crosses[CrossMap[current_cross_ID]].NeiborRoad[i];
            if(current_road_ID == -1)
                continue;

            next_cross_ID = RoadInfo.Roads[RoadInfo.RoadMap[current_road_ID]].corresponding_cross[current_cross_ID];
            if(Crosses[CrossMap[next_cross_ID]].first_direction != -1)
                continue;
            else if(Crosses[CrossMap[next_cross_ID]].NeiborRoad[0] == current_road_ID)
            {
                int actual_orientation;
                actual_orientation = (i+Crosses[CrossMap[current_cross_ID]].first_direction) % 4;

                Crosses[CrossMap[next_cross_ID]].first_direction = direction((actual_orientation + 2) % 4);
                TraversalSet.push(next_cross_ID);
            }
            else if(Crosses[CrossMap[next_cross_ID]].NeiborRoad[1] == current_road_ID)
            {
                int actual_orientation;
                actual_orientation = (i+Crosses[CrossMap[current_cross_ID]].first_direction) % 4;

                Crosses[CrossMap[next_cross_ID]].first_direction = direction((actual_orientation + 2 + 3) % 4);
                TraversalSet.push(next_cross_ID);
            }
            else if(Crosses[CrossMap[next_cross_ID]].NeiborRoad[2] == current_road_ID)
            {
                int actual_orientation;
                actual_orientation = (i+Crosses[CrossMap[current_cross_ID]].first_direction) % 4;

                Crosses[CrossMap[next_cross_ID]].first_direction = direction((actual_orientation + 2 + 2) % 4);
                TraversalSet.push(next_cross_ID);
            }
            else if(Crosses[CrossMap[next_cross_ID]].NeiborRoad[3] == current_road_ID)
            {
                int actual_orientation;
                actual_orientation = (i+Crosses[CrossMap[current_cross_ID]].first_direction) % 4;

                Crosses[CrossMap[next_cross_ID]].first_direction = direction((actual_orientation + 2 + 1) % 4);
                TraversalSet.push(next_cross_ID);
            }
            else
            {
                cerr<<"Error! This Cross is not the endpoint of the current road!\n";
                exit(-1);
            }
        }
    }
    return;
}


void Cross_Info::get_coordinate(Road_Info &RoadInfo)
{
    int dx[4] = {0,1,0,-1};
    int dy[4] = {1,0,-1,0};
    queue<int> TraversalSet;

    Crosses[0].Iscalculated = true;
    Crosses[0].position = {0,0};
    TraversalSet.push(Crosses[0].ID);
    while(!TraversalSet.empty())
    {
        int current_cross_ID;
        int current_road_ID;
        int next_cross_ID;
        current_cross_ID = TraversalSet.front();
        TraversalSet.pop();
        for(int i = 0;i<4;i++)
        {
            current_road_ID = Crosses[CrossMap[current_cross_ID]].NeiborRoad[i];
            if(current_road_ID == -1)
                continue;

            next_cross_ID = RoadInfo.Roads[RoadInfo.RoadMap[current_road_ID]].corresponding_cross[current_cross_ID];
            if(!Crosses[CrossMap[next_cross_ID]].Iscalculated)
            {
                int actual_orientation;
                actual_orientation = (i+Crosses[CrossMap[current_cross_ID]].first_direction) % 4;
                Crosses[CrossMap[next_cross_ID]].position.first = Crosses[CrossMap[current_cross_ID]].position.first
                                                                                      + dx[actual_orientation]*RoadInfo.Roads[RoadInfo.RoadMap[current_road_ID]].length;
                Crosses[CrossMap[next_cross_ID]].position.second = Crosses[CrossMap[current_cross_ID]].position.second
                                                                                       + dy[actual_orientation]*RoadInfo.Roads[RoadInfo.RoadMap[current_road_ID]].length;
                Crosses[CrossMap[next_cross_ID]].Iscalculated = true;
                TraversalSet.push(next_cross_ID);
            }
        }
    }
    return;
}

void Cross_Info::get_directed_neighbor()
{
    for(int i = 1;i<=NumCross;i++)
    {
        if(Crosses[CrossMap[i]].first_direction == direction::up)
        {
            Crosses[CrossMap[i]].DirectNeiborRoad[0] = Crosses[CrossMap[i]].NeiborRoad[0];
            Crosses[CrossMap[i]].DirectNeiborRoad[1] = Crosses[CrossMap[i]].NeiborRoad[1];
            Crosses[CrossMap[i]].DirectNeiborRoad[2] = Crosses[CrossMap[i]].NeiborRoad[2];
            Crosses[CrossMap[i]].DirectNeiborRoad[3] = Crosses[CrossMap[i]].NeiborRoad[3];
        }
        else if(Crosses[CrossMap[i]].first_direction == direction::right)
        {
            Crosses[CrossMap[i]].DirectNeiborRoad[0] = Crosses[CrossMap[i]].NeiborRoad[3];
            Crosses[CrossMap[i]].DirectNeiborRoad[1] = Crosses[CrossMap[i]].NeiborRoad[0];
            Crosses[CrossMap[i]].DirectNeiborRoad[2] = Crosses[CrossMap[i]].NeiborRoad[1];
            Crosses[CrossMap[i]].DirectNeiborRoad[3] = Crosses[CrossMap[i]].NeiborRoad[2];
        }
        else if(Crosses[CrossMap[i]].first_direction == direction::down)
        {
            Crosses[CrossMap[i]].DirectNeiborRoad[0] = Crosses[CrossMap[i]].NeiborRoad[2];
            Crosses[CrossMap[i]].DirectNeiborRoad[1] = Crosses[CrossMap[i]].NeiborRoad[3];
            Crosses[CrossMap[i]].DirectNeiborRoad[2] = Crosses[CrossMap[i]].NeiborRoad[0];
            Crosses[CrossMap[i]].DirectNeiborRoad[3] = Crosses[CrossMap[i]].NeiborRoad[1];
        }
        else if(Crosses[CrossMap[i]].first_direction == direction::left)
        {
            Crosses[CrossMap[i]].DirectNeiborRoad[0] = Crosses[CrossMap[i]].NeiborRoad[1];
            Crosses[CrossMap[i]].DirectNeiborRoad[1] = Crosses[CrossMap[i]].NeiborRoad[2];
            Crosses[CrossMap[i]].DirectNeiborRoad[2] = Crosses[CrossMap[i]].NeiborRoad[3];
            Crosses[CrossMap[i]].DirectNeiborRoad[3] = Crosses[CrossMap[i]].NeiborRoad[0];
        }
    }
    return;
}

void Cross_Info::set_trave_cost_Matrix(Road_Info &RoadInfo) {
    int i,j;
    bool IsDuplex;
    for(int i = 0;i<NumCross;i++)
        for(int j = 0;j<NumCross;j++)
        {
            if(i!=j)
                trav_cost[i][j] = INF;
            else
                trav_cost[i][j] = 0;
        }

    for(auto ite = RoadInfo.Roads.begin();ite != RoadInfo.Roads.end();ite++)
    {
        i = ite->endpoints.first;
        j = ite->endpoints.second;
        IsDuplex = ite->DoubleOrientation;
        if(!IsDuplex)
            trav_cost[CrossMap[i]][CrossMap[j]] = ite->length;
        else
        {
            trav_cost[CrossMap[i]][CrossMap[j]] = ite->length;
            trav_cost[CrossMap[j]][CrossMap[i]] = ite->length;
        }
    }

}

