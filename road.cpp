#include "road.h"
#include "cross.h"

void Road_Info::initialize(string filename)
{
    ifstream fin;
    fin.open(filename.c_str());

    if(!fin.is_open())
    {
        cerr<<"Error! Cannot open Road file!\n";
        exit(-1);
    }

    string useless;
    char HeadCharacter;

    int count = 0;
    while(!fin.eof())
    {
        HeadCharacter = fin.get();
        if(HeadCharacter == '#')
            getline(fin,useless);
        else if(HeadCharacter == '(')
        {
            Road temp;
            fin>>temp.ID;
            fin>>useless;
            fin>>temp.length;
            fin>>useless;
            fin>>temp.MaxSpeed;
            fin>>useless;
            fin>>temp.NumberOfLane;
            fin>>useless;
            fin>>temp.endpoints.first;
            fin>>useless;
            fin>>temp.endpoints.second;
            fin>>useless;
            fin>>temp.DoubleOrientation;
            temp.corresponding_cross.insert(make_pair(temp.endpoints.first,temp.endpoints.second));
            temp.corresponding_cross.insert(make_pair(temp.endpoints.second,temp.endpoints.first));
            getline(fin,useless);
            Roads.push_back(temp);
            RoadMap.insert(make_pair(temp.ID,count));
            count++;
        }
    }
    NumRoad = count;

    fin.close();
    return;
}

void Road_Info::get_road_neighbor(class Cross_Info &CrossInfo)
{
    for(int i = 0;i<NumRoad;i++)
    {
        int road_position = i;
        if(Roads[road_position].DoubleOrientation)
        {
            int first_cross,second_cross;
            first_cross = Roads[road_position].endpoints.first;
            second_cross = Roads[road_position].endpoints.second;

            int dx,dy;

            dx = CrossInfo.Crosses[CrossInfo.CrossMap[second_cross]].position.first - CrossInfo.Crosses[CrossInfo.CrossMap[first_cross]].position.first;
            dy = CrossInfo.Crosses[CrossInfo.CrossMap[second_cross]].position.second - CrossInfo.Crosses[CrossInfo.CrossMap[first_cross]].position.second;
            if(dx != 0 && dy != 0 && Roads[road_position].corresponding_cross[first_cross] != second_cross)
            {
                cerr<<"Error! Two Crosses are not adjacent!\n";
                exit(-1);
            }
            int next_front_road,next_left_road,next_right_road;
            if(dx > 0)
            {
                next_front_road = CrossInfo.Crosses[CrossInfo.CrossMap[second_cross]].DirectNeiborRoad[direction::right];
                if(next_front_road != -1 && (Roads[RoadMap[next_front_road]].DoubleOrientation || (Roads[RoadMap[next_front_road]].endpoints.first == second_cross)))
                    Roads[road_position].second_point.front = next_front_road;
                else
                    Roads[road_position].second_point.front = -1;

                next_left_road = CrossInfo.Crosses[CrossInfo.CrossMap[second_cross]].DirectNeiborRoad[direction::up];
                if(next_left_road != -1 && (Roads[RoadMap[next_left_road]].DoubleOrientation || (Roads[RoadMap[next_left_road]].endpoints.first == second_cross)))
                    Roads[road_position].second_point.left = next_left_road;
                else
                    Roads[road_position].second_point.left = -1;

                next_right_road = CrossInfo.Crosses[CrossInfo.CrossMap[second_cross]].DirectNeiborRoad[direction::down];
                if(next_right_road != -1 && (Roads[RoadMap[next_right_road]].DoubleOrientation || (Roads[RoadMap[next_right_road]].endpoints.first == second_cross)))
                    Roads[road_position].second_point.right = next_right_road;
                else
                    Roads[road_position].second_point.right = -1;

                /*--------------------------------------------------------------------------------------------------------------*/

                next_front_road = CrossInfo.Crosses[CrossInfo.CrossMap[first_cross]].DirectNeiborRoad[direction::left];
                if(next_front_road != -1 && (Roads[RoadMap[next_front_road]].DoubleOrientation || (Roads[RoadMap[next_front_road]].endpoints.first == first_cross)))
                    Roads[road_position].first_point.front = next_front_road;
                else
                    Roads[road_position].first_point.front = -1;

                next_left_road = CrossInfo.Crosses[CrossInfo.CrossMap[first_cross]].DirectNeiborRoad[direction::down];
                if(next_left_road != -1 && (Roads[RoadMap[next_left_road]].DoubleOrientation || (Roads[RoadMap[next_left_road]].endpoints.first == second_cross)))
                    Roads[road_position].first_point.left = next_left_road;
                else
                    Roads[road_position].first_point.left = -1;

                next_right_road = CrossInfo.Crosses[CrossInfo.CrossMap[first_cross]].DirectNeiborRoad[direction::up];
                if(next_right_road != -1 && (Roads[RoadMap[next_right_road]].DoubleOrientation || (Roads[RoadMap[next_right_road]].endpoints.first == second_cross)))
                    Roads[road_position].first_point.right = next_right_road;
                else
                    Roads[road_position].first_point.right = -1;

            }
            else if(dx < 0)
            {
                next_front_road = CrossInfo.Crosses[CrossInfo.CrossMap[second_cross]].DirectNeiborRoad[direction::left];
                if(next_front_road != -1 && (Roads[RoadMap[next_front_road]].DoubleOrientation || (Roads[RoadMap[next_front_road]].endpoints.first == second_cross)))
                   Roads[road_position].second_point.front = next_front_road;
                else
                    Roads[road_position].second_point.front = -1;

                next_left_road = CrossInfo.Crosses[CrossInfo.CrossMap[second_cross]].DirectNeiborRoad[direction::down];
                if(next_left_road != -1 && (Roads[RoadMap[next_left_road]].DoubleOrientation || (Roads[RoadMap[next_left_road]].endpoints.first == second_cross)))
                    Roads[road_position].second_point.left = next_left_road;
                else
                    Roads[road_position].second_point.left = -1;

                next_right_road = CrossInfo.Crosses[CrossInfo.CrossMap[second_cross]].DirectNeiborRoad[direction::up];
                if(next_right_road != -1 && (Roads[RoadMap[next_right_road]].DoubleOrientation || (Roads[RoadMap[next_right_road]].endpoints.first == second_cross)))
                    Roads[road_position].second_point.right = next_right_road;
                else
                    Roads[road_position].second_point.right = -1;

                /*--------------------------------------------------------------------------------------------------------------*/

                next_front_road = CrossInfo.Crosses[CrossInfo.CrossMap[first_cross]].DirectNeiborRoad[direction::right];
                if(next_front_road != -1 && (Roads[RoadMap[next_front_road]].DoubleOrientation || (Roads[RoadMap[next_front_road]].endpoints.first == first_cross)))
                    Roads[road_position].first_point.front = next_front_road;
                else
                    Roads[road_position].first_point.front = -1;

                next_left_road = CrossInfo.Crosses[CrossInfo.CrossMap[first_cross]].DirectNeiborRoad[direction::up];
                if(next_left_road != -1 && (Roads[RoadMap[next_left_road]].DoubleOrientation || (Roads[RoadMap[next_left_road]].endpoints.first == second_cross)))
                    Roads[road_position].first_point.left = next_left_road;
                else
                    Roads[road_position].first_point.left = -1;

                next_right_road = CrossInfo.Crosses[CrossInfo.CrossMap[first_cross]].DirectNeiborRoad[direction::down];
                if(next_right_road != -1 && (Roads[RoadMap[next_right_road]].DoubleOrientation || (Roads[RoadMap[next_right_road]].endpoints.first == second_cross)))
                    Roads[road_position].first_point.right = next_right_road;
                else
                    Roads[road_position].first_point.right = -1;
            }
            else if(dy > 0)
            {
                next_front_road = CrossInfo.Crosses[CrossInfo.CrossMap[second_cross]].DirectNeiborRoad[direction::up];
                if(next_front_road != -1 && (Roads[RoadMap[next_front_road]].DoubleOrientation || (Roads[RoadMap[next_front_road]].endpoints.first == second_cross)))
                    Roads[road_position].second_point.front = next_front_road;
                else
                    Roads[road_position].second_point.front = -1;

                next_left_road = CrossInfo.Crosses[CrossInfo.CrossMap[second_cross]].DirectNeiborRoad[direction::left];
                if(next_left_road != -1 && (Roads[RoadMap[next_left_road]].DoubleOrientation || (Roads[RoadMap[next_left_road]].endpoints.first == second_cross)))
                    Roads[road_position].second_point.left = next_left_road;
                else
                    Roads[road_position].second_point.left = -1;

                next_right_road = CrossInfo.Crosses[CrossInfo.CrossMap[second_cross]].DirectNeiborRoad[direction::right];
                if(next_right_road != -1 && (Roads[RoadMap[next_right_road]].DoubleOrientation || (Roads[RoadMap[next_right_road]].endpoints.first == second_cross)))
                    Roads[road_position].second_point.right = next_right_road;
                else
                    Roads[road_position].second_point.right = -1;

                /*--------------------------------------------------------------------------------------------------------------*/

                next_front_road = CrossInfo.Crosses[CrossInfo.CrossMap[first_cross]].DirectNeiborRoad[direction::down];
                if(next_front_road != -1 && (Roads[RoadMap[next_front_road]].DoubleOrientation || (Roads[RoadMap[next_front_road]].endpoints.first == first_cross)))
                   Roads[road_position].first_point.front = next_front_road;
                else
                    Roads[road_position].first_point.front = -1;

                next_left_road = CrossInfo.Crosses[CrossInfo.CrossMap[first_cross]].DirectNeiborRoad[direction::right];
                if(next_left_road != -1 && (Roads[RoadMap[next_left_road]].DoubleOrientation || (Roads[RoadMap[next_left_road]].endpoints.first == second_cross)))
                    Roads[road_position].first_point.left = next_left_road;
                else
                    Roads[road_position].first_point.left = -1;

                next_right_road = CrossInfo.Crosses[CrossInfo.CrossMap[first_cross]].DirectNeiborRoad[direction::left];
                if(next_right_road != -1 && (Roads[RoadMap[next_right_road]].DoubleOrientation || (Roads[RoadMap[next_right_road]].endpoints.first == second_cross)))
                    Roads[road_position].first_point.right = next_right_road;
                else
                    Roads[road_position].first_point.right = -1;
            }
            else if(dy < 0)
            {
                next_front_road = CrossInfo.Crosses[CrossInfo.CrossMap[second_cross]].DirectNeiborRoad[direction::down];
                if(next_front_road != -1 && (Roads[RoadMap[next_front_road]].DoubleOrientation || (Roads[RoadMap[next_front_road]].endpoints.first == second_cross)))
                    Roads[road_position].second_point.front = next_front_road;
                else
                    Roads[road_position].second_point.front = -1;

                next_left_road = CrossInfo.Crosses[CrossInfo.CrossMap[second_cross]].DirectNeiborRoad[direction::right];
                if(next_left_road != -1 && (Roads[RoadMap[next_left_road]].DoubleOrientation || (Roads[RoadMap[next_left_road]].endpoints.first == second_cross)))
                    Roads[road_position].second_point.left = next_left_road;
                else
                    Roads[road_position].second_point.left = -1;

                next_right_road = CrossInfo.Crosses[CrossInfo.CrossMap[second_cross]].DirectNeiborRoad[direction::left];
                if(next_right_road != -1 && (Roads[RoadMap[next_right_road]].DoubleOrientation || (Roads[RoadMap[next_right_road]].endpoints.first == second_cross)))
                    Roads[road_position].second_point.right = next_right_road;
                else
                    Roads[road_position].second_point.right = -1;

                /*--------------------------------------------------------------------------------------------------------------*/

                next_front_road = CrossInfo.Crosses[CrossInfo.CrossMap[first_cross]].DirectNeiborRoad[direction::up];
                if(next_front_road != -1 && (Roads[RoadMap[next_front_road]].DoubleOrientation || (Roads[RoadMap[next_front_road]].endpoints.first == first_cross)))
                    Roads[road_position].first_point.front = next_front_road;
                else
                    Roads[road_position].first_point.front = -1;

                next_left_road = CrossInfo.Crosses[CrossInfo.CrossMap[first_cross]].DirectNeiborRoad[direction::left];
                if(next_left_road != -1 && (Roads[RoadMap[next_left_road]].DoubleOrientation || (Roads[RoadMap[next_left_road]].endpoints.first == second_cross)))
                    Roads[road_position].first_point.left = next_left_road;
                else
                    Roads[road_position].first_point.left = -1;

                next_right_road = CrossInfo.Crosses[CrossInfo.CrossMap[first_cross]].DirectNeiborRoad[direction::right];
                if(next_right_road != -1 && (Roads[RoadMap[next_right_road]].DoubleOrientation || (Roads[RoadMap[next_right_road]].endpoints.first == second_cross)))
                    Roads[road_position].first_point.right = next_right_road;
                else
                    Roads[road_position].first_point.right = -1;
            }
            else{}
        }
        else if(!Roads[road_position].DoubleOrientation)
        {
            int first_cross,second_cross;
            first_cross = Roads[road_position].endpoints.first;
            second_cross = Roads[road_position].endpoints.second;

            int dx,dy;

            dx = CrossInfo.Crosses[CrossInfo.CrossMap[second_cross]].position.first - CrossInfo.Crosses[CrossInfo.CrossMap[first_cross]].position.first;
            dy = CrossInfo.Crosses[CrossInfo.CrossMap[second_cross]].position.second - CrossInfo.Crosses[CrossInfo.CrossMap[first_cross]].position.second;
            if(dx != 0 && dy != 0 && Roads[road_position].corresponding_cross[first_cross] != second_cross)
            {
                cerr<<"Error! Two Crosses are not adjacent!\n";
                exit(-1);
            }
            int next_front_road,next_left_road,next_right_road;
            if(dx > 0)
            {
                next_front_road = CrossInfo.Crosses[CrossInfo.CrossMap[second_cross]].DirectNeiborRoad[direction::right];
                if(next_front_road != -1 && (Roads[RoadMap[next_front_road]].DoubleOrientation || (Roads[RoadMap[next_front_road]].endpoints.first == second_cross)))
                    Roads[road_position].second_point.front = next_front_road;
                else
                    Roads[road_position].second_point.front = -1;

                next_left_road = CrossInfo.Crosses[CrossInfo.CrossMap[second_cross]].DirectNeiborRoad[direction::up];
                if(next_left_road != -1 && (Roads[RoadMap[next_left_road]].DoubleOrientation || (Roads[RoadMap[next_left_road]].endpoints.first == second_cross)))
                    Roads[road_position].second_point.left = next_left_road;
                else
                    Roads[road_position].second_point.left = -1;

                next_right_road = CrossInfo.Crosses[CrossInfo.CrossMap[second_cross]].DirectNeiborRoad[direction::down];
                if(next_right_road != -1 && (Roads[RoadMap[next_right_road]].DoubleOrientation || (Roads[RoadMap[next_right_road]].endpoints.first == second_cross)))
                    Roads[road_position].second_point.right = next_right_road;
                else
                    Roads[road_position].second_point.right = -1;
            }
            else if(dx < 0)
            {
                next_front_road = CrossInfo.Crosses[CrossInfo.CrossMap[second_cross]].DirectNeiborRoad[direction::left];
                if(next_front_road != -1 && (Roads[RoadMap[next_front_road]].DoubleOrientation || (Roads[RoadMap[next_front_road]].endpoints.first == second_cross)))
                    Roads[road_position].second_point.front = next_front_road;
                else
                    Roads[road_position].second_point.front = -1;

                next_left_road = CrossInfo.Crosses[CrossInfo.CrossMap[second_cross]].DirectNeiborRoad[direction::down];
                if(next_left_road != -1 && (Roads[RoadMap[next_left_road]].DoubleOrientation || (Roads[RoadMap[next_left_road]].endpoints.first == second_cross)))
                    Roads[road_position].second_point.left = next_left_road;
                else
                    Roads[road_position].second_point.left = -1;

                next_right_road = CrossInfo.Crosses[CrossInfo.CrossMap[second_cross]].DirectNeiborRoad[direction::up];
                if(next_right_road != -1 && (Roads[RoadMap[next_right_road]].DoubleOrientation || (Roads[RoadMap[next_right_road]].endpoints.first == second_cross)))
                    Roads[road_position].second_point.right = next_right_road;
                else
                    Roads[road_position].second_point.right = -1;

            }
            else if(dy > 0)
            {
                next_front_road = CrossInfo.Crosses[CrossInfo.CrossMap[second_cross]].DirectNeiborRoad[direction::up];
                if(next_front_road != -1 && (Roads[RoadMap[next_front_road]].DoubleOrientation || (Roads[RoadMap[next_front_road]].endpoints.first == second_cross)))
                    Roads[road_position].second_point.front = next_front_road;
                else
                    Roads[road_position].second_point.front = -1;

                next_left_road = CrossInfo.Crosses[CrossInfo.CrossMap[second_cross]].DirectNeiborRoad[direction::left];
                if(next_left_road != -1 && (Roads[RoadMap[next_left_road]].DoubleOrientation || (Roads[RoadMap[next_left_road]].endpoints.first == second_cross)))
                    Roads[road_position].second_point.left = next_left_road;
                else
                    Roads[road_position].second_point.left = -1;

                next_right_road = CrossInfo.Crosses[CrossInfo.CrossMap[second_cross]].DirectNeiborRoad[direction::right];
                if(next_right_road != -1 && (Roads[RoadMap[next_right_road]].DoubleOrientation || (Roads[RoadMap[next_right_road]].endpoints.first == second_cross)))
                    Roads[road_position].second_point.right = next_right_road;
                else
                    Roads[road_position].second_point.right = -1;

            }
            else if(dy < 0)
            {
                next_front_road = CrossInfo.Crosses[CrossInfo.CrossMap[second_cross]].DirectNeiborRoad[direction::down];
                if(next_front_road != -1 && (Roads[RoadMap[next_front_road]].DoubleOrientation || (Roads[RoadMap[next_front_road]].endpoints.first == second_cross)))
                    Roads[road_position].second_point.front = next_front_road;
                else
                    Roads[road_position].second_point.front = -1;

                next_left_road = CrossInfo.Crosses[CrossInfo.CrossMap[second_cross]].DirectNeiborRoad[direction::right];
                if(next_left_road != -1 && (Roads[RoadMap[next_left_road]].DoubleOrientation || (Roads[RoadMap[next_left_road]].endpoints.first == second_cross)))
                    Roads[road_position].second_point.left = next_left_road;
                else
                    Roads[road_position].second_point.left = -1;

                next_right_road = CrossInfo.Crosses[CrossInfo.CrossMap[second_cross]].DirectNeiborRoad[direction::left];
                if(next_right_road != -1 && (Roads[RoadMap[next_right_road]].DoubleOrientation || (Roads[RoadMap[next_right_road]].endpoints.first == second_cross)))
                    Roads[road_position].second_point.right = next_right_road;
                else
                    Roads[road_position].second_point.right = -1;
            }
            else{}
        }
    }
    return;
}

int Road_Info::get_corresponding_cross(int RoadID,int CrossID)
{
    if(Roads[RoadMap[RoadID]].endpoints.first != CrossID && Roads[RoadMap[RoadID]].endpoints.second != CrossID)
    {
        cerr<<"Error! This Cross doesn't belong to this road!\n";
        cin.get();
        exit(-1);
    }
    else
        return Roads[RoadMap[RoadID]].corresponding_cross[CrossID];

}