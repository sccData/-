#include "map.h"

//This is a simpler policy
//void GuideMap::calculate_map(Cross_Info &CrossInfo,Road_Info &RoadInfo)
//{
//    for(int i = 1;i<CrossInfo.NumCross+1;i++)
//    {
//        for(int j = 1;j<CrossInfo.NumCross+1;j++)
//        {
//            MapMatrix[i][j].MyChoice[0] = 0;
//            if(i == j) continue;
//            else
//            {
//                int horizon,vertical;
//                horizon = CrossInfo.Crosses[CrossInfo.CrossMap[j]].position.first - CrossInfo.Crosses[CrossInfo.CrossMap[i]].position.first;
//                if(horizon > 0)
//                {
//                    MapMatrix[i][j].MyChoice[0]++;
//                    int temp = -1;
//                    temp = CrossInfo.Crosses[CrossInfo.CrossMap[i]].DirectNeiborRoad[int(direction::right)];
//                    if (temp == -1 || (!RoadInfo.Roads[RoadInfo.RoadMap[temp]].DoubleOrientation && RoadInfo.Roads[RoadInfo.RoadMap[temp]].endpoints.first != i))
//                        MapMatrix[i][j].MyChoice[0]--;
//                    else
//                        MapMatrix[i][j].MyChoice[MapMatrix[i][j].MyChoice[0]] = temp;
//                }
//                else if(horizon < 0)
//                {
//                    MapMatrix[i][j].MyChoice[0]++;
//                    int temp = -1;
//                    temp = CrossInfo.Crosses[CrossInfo.CrossMap[i]].DirectNeiborRoad[int(direction::left)];
//                    if (temp == -1 || (!RoadInfo.Roads[RoadInfo.RoadMap[temp]].DoubleOrientation && RoadInfo.Roads[RoadInfo.RoadMap[temp]].endpoints.first != i))
//                        MapMatrix[i][j].MyChoice[0]--;
//                    else
//                        MapMatrix[i][j].MyChoice[MapMatrix[i][j].MyChoice[0]] = temp;
//                }
//                else {}
//
//                vertical = CrossInfo.Crosses[CrossInfo.CrossMap[j]].position.second - CrossInfo.Crosses[CrossInfo.CrossMap[i]].position.second;
//                if(vertical>0)
//                {
//                    MapMatrix[i][j].MyChoice[0]++;
//                    int temp = -1;
//                    temp = CrossInfo.Crosses[CrossInfo.CrossMap[i]].DirectNeiborRoad[int(direction::up)];
//                    if (temp == -1 || (!RoadInfo.Roads[RoadInfo.RoadMap[temp]].DoubleOrientation && RoadInfo.Roads[RoadInfo.RoadMap[temp]].endpoints.first != i))
//                        MapMatrix[i][j].MyChoice[0]--;
//                    else
//                        MapMatrix[i][j].MyChoice[MapMatrix[i][j].MyChoice[0]] = temp;
//                }
//                else if(vertical<0)
//                {
//                    MapMatrix[i][j].MyChoice[0]++;
//                    int temp = -1;
//                    temp = CrossInfo.Crosses[CrossInfo.CrossMap[i]].DirectNeiborRoad[int(direction::down)];
//                    if (temp == -1 || (!RoadInfo.Roads[RoadInfo.RoadMap[temp]].DoubleOrientation && RoadInfo.Roads[RoadInfo.RoadMap[temp]].endpoints.first != i))
//                        MapMatrix[i][j].MyChoice[0]--;
//                    else
//                        MapMatrix[i][j].MyChoice[MapMatrix[i][j].MyChoice[0]] = temp;
//                }
//                else {}
//            }
//        }
//    }
//
//    return;
//}


//This is the policy based on the shortest path
void GuideMap::calculate_map(Cross_Info &CrossInfo,Road_Info &RoadInfo)
{
    mod_dijkstra(CrossInfo);

    for(int i = 1;i<CrossInfo.NumCross+1;i++)
    {
        for(int j = 1;j<CrossInfo.NumCross+1;j++)
        {
            MapMatrix[i][j].MyChoice[0] = 0;
            if(i == j) continue;
            else
            {
                int source = CrossInfo.CrossMap[i];
                int target = CrossInfo.CrossMap[j];
                int minimum_distance = min_cost[source][target];
                if(shortest_path[source][target][0] <= 0)
                {
                    cerr<<"Wrong! cannot reach the destination!\n";
                    exit(-1);
                }
                int next_cross;
                next_cross = CrossInfo.Crosses[shortest_path[source][target][2]].ID;
                int next_road = -1;
                int benchmark = -1;
                for(int k = 0;k<4;k++){
                    if(CrossInfo.Crosses[source].DirectNeiborRoad[k] == -1)
                        continue;
                    if(RoadInfo.Roads[RoadInfo.RoadMap[CrossInfo.Crosses[source].DirectNeiborRoad[k]]].corresponding_cross[i] == next_cross)
                    {
                        next_road = RoadInfo.Roads[RoadInfo.RoadMap[CrossInfo.Crosses[source].DirectNeiborRoad[k]]].ID;
                        benchmark = k;
                        break;
                    }

                }
                if(next_road == -1 || benchmark == -1)
                {
                    cerr<<"I can't find the related road!\n";
                    exit(-1);
                }
                MapMatrix[i][j].MyChoice[0]++;
                MapMatrix[i][j].MyChoice[MapMatrix[i][j].MyChoice[0]] = next_road;
                for(int k = 0;k<4;k++)
                {
                    if(k == benchmark || CrossInfo.Crosses[source].DirectNeiborRoad[k] == -1) continue;
                    int next_source = CrossInfo.CrossMap[RoadInfo.Roads[RoadInfo.RoadMap[CrossInfo.Crosses[source].DirectNeiborRoad[k]]].corresponding_cross[i]];
                    int my_length = RoadInfo.Roads[RoadInfo.RoadMap[CrossInfo.Crosses[source].DirectNeiborRoad[k]]].length + min_cost[next_source][target];
                    if(minimum_distance == my_length)
                    {
                        next_road = RoadInfo.Roads[RoadInfo.RoadMap[CrossInfo.Crosses[source].DirectNeiborRoad[k]]].ID;
                        if(!RoadInfo.Roads[RoadInfo.RoadMap[next_road]].DoubleOrientation && RoadInfo.Roads[RoadInfo.RoadMap[next_road]].endpoints.first != i)
                            continue;
                        MapMatrix[i][j].MyChoice[0]++;
                        MapMatrix[i][j].MyChoice[MapMatrix[i][j].MyChoice[0]] = next_road;
                    }
                }
            }
        }
    }

    return;
}

void GuideMap::mod_dijkstra(Cross_Info &CrossInfo)
{
    int i, j, k, m, minimum;
    int node_num = CrossInfo.NumCross;


    if(min_cost == nullptr)
    {
        min_cost = new int*[CrossInfo.NumCross];
        min_cost[0] = new int[CrossInfo.NumCross*CrossInfo.NumCross];
        for(int i = 1;i<CrossInfo.NumCross;i++)
            min_cost[i] = min_cost[i-1] + CrossInfo.NumCross;
    }
    else
    {
        cerr<<"Wrong! min_cost is not a nullptr!\n";
        exit(-1);
    }

    if(shortest_path == nullptr)
    {
        shortest_path = new int**[node_num];
        for (int i = 0; i<node_num; ++i)
        {
            shortest_path[i] = new int*[node_num];
            for (int j = 0; j<node_num; ++j)
            {
                shortest_path[i][j] = new int[node_num];
            }
        }
    }
    else
    {
        cerr<<"Wrong! shortest_path is not a nullptr!\n";
        exit(-1);
    }



    for (i = 0; i < node_num; i++)
    {
        for (j = 0; j < node_num; j++)
        {
            if (j == i)
            {
                min_cost[i][j] = 0;
                continue;
            }

            shortest_path[i][j][0] = 1;
            shortest_path[i][j][1] = i;
            min_cost[i][j] = INF;
        }
    }

    //cout << "Initial min cost matrix:\n\n\t";
    //for (int i = 1; i <= node_num; i++) cout << i << '\t';
    //cout << endl << endl;
    //for (int i = 1; i <= node_num; i++)
    //{
    //	cout << i << '\t';
    //	for (int j = 1; j <= node_num; j++)
    //	{
    //		if (min_cost[i][j] == INF) cout << "INF\t";	//����ֱ���֮��ı����ɱ�Ϊ����
    //		else cout << min_cost[i][j] << '\t';
    //	}
    //	cout << endl << endl;
    //}


    //cout << "Initial shortest path information:\n\n\t";
    //for (int i = 1; i <= node_num; i++) cout << i << '\t';
    //cout << endl << endl;
    //for (int i = 1; i <= node_num; i++)
    //{
    //	for (int j = 1; j <= node_num; j++)
    //	{
    //		if (min_cost[i][j] == INF)
    //		{
    //			cout << i << "->" << j << ": no path\n";
    //			continue;
    //		}
    //		else if (min_cost[i][j] == 0)
    //		{
    //			cout << i << "->" << j << ": node \n";
    //			continue;
    //		}

    //		cout << i << "->" << j << "(through "<<shortest_path[i][j][0]<<" nodes): ";
    //		for (int k = 1; k <= shortest_path[i][j][0]; k++)
    //		{
    //			cout << shortest_path[i][j][k]<<"->";
    //		}
    //		cout << "\b\b  "<< endl;
    //	}
    //	cout << endl << endl;
    //}

#define MAX_NODE_TAG_LENGTH 300
    int mark[MAX_NODE_TAG_LENGTH], dist[MAX_NODE_TAG_LENGTH], dist1[MAX_NODE_TAG_LENGTH], nearest_neighbor[MAX_NODE_TAG_LENGTH];

    for (i = 0; i < node_num; i++) // For each vertex
    {
        mark[i] = 1;

        for (j = 0; j < node_num; j++)
        {
            if (j == i)
                continue;

            mark[j] = 0;
            dist[j] = CrossInfo.trav_cost[i][j];
            dist1[j] = dist[j];
        }

        for (k = 0; k < node_num; k++) // there are vertex_num steps
        {
            minimum = INF;
            nearest_neighbor[0] = 0;

            for (j = 0; j < node_num; j++)
            {
                if (mark[j])
                    continue;

                if (dist1[j] == INF)
                    continue;

                if (dist1[j] < minimum)
                    minimum = dist1[j];
            }

            if (minimum == INF)
                continue;

            for (j = 0; j < node_num; j++)
            {
                if (mark[j])
                    continue;

                if (dist1[j] == minimum)
                {
                    nearest_neighbor[0] ++;
                    nearest_neighbor[nearest_neighbor[0]] = j;
                }
            }

            int v = nearest_neighbor[1];
            dist1[v] = INF;
            mark[v] = 1;

            if (shortest_path[i][v][0] == 0 || (shortest_path[i][v][0] > 0 && shortest_path[i][v][shortest_path[i][v][0]] != v))
            {
                shortest_path[i][v][0] ++;
                shortest_path[i][v][shortest_path[i][v][0]] = v;
            }

            for (j = 0; j < node_num; j++)
            {
                if (mark[j])
                    continue;

                if (minimum + CrossInfo.trav_cost[v][j] < dist[j])
                {
                    dist[j] = minimum + CrossInfo.trav_cost[v][j];
                    dist1[j] = minimum + CrossInfo.trav_cost[v][j];
                    for (m = 0; m <= shortest_path[i][v][0]; m++)
                    {
                        shortest_path[i][j][m] = shortest_path[i][v][m];
                    }
                }
            }

            for (j = 0; j < node_num; j++)
            {
                if (j == i)
                    continue;

                min_cost[i][j] = dist[j];
            }
        }
    }

    for (i = 0; i < node_num; i++)
    {
        for (j = 0; j < node_num; j++)
        {
            if (shortest_path[i][j][0] == 1)
                shortest_path[i][j][0] = 0;
        }
    }

    for (i = 0; i < node_num; i++)
    {
        shortest_path[i][i][0] = 1;
        shortest_path[i][i][1] = i;
        min_cost[i][i] = 0;
    }

//    cout << "Final min cost matrix:\n\n\t";
//    for (int i = 0; i < node_num; i++) cout << i << '\t';
//    cout << endl << endl;
//    for (int i = 0; i < node_num; i++)
//    {
//    	cout << i << '\t';
//    	for (int j = 0; j < node_num; j++)
//    	{
//    		if (min_cost[i][j] == INF) cout << "INF\t";	//����ֱ���֮��ı����ɱ�Ϊ����
//    		else cout << min_cost[i][j] << '\t';
//    	}
//    	cout << endl << endl;
//    }
//
//    cout << "Final shortest path information:\n\n";
//    for (int i = 0; i < node_num; i++)
//    {
//    	for (int j = 0; j < node_num; j++)
//    	{
//    		if (min_cost[i][j] == INF)
//    		{
//    			cout << i << "->" << j << ": no path\n";
//    			continue;
//    		}
//
//    		cout << i << "->" << j << "(through "<<shortest_path[i][j][0]<<" nodes): ";
//    		for (int k = 1; k <= shortest_path[i][j][0]; k++)
//    		{
//    			cout << shortest_path[i][j][k]<<"->";
//    		}
//    		cout << "\b\b  "<< endl;
//    	}
//    	cout << endl << endl;
//    }
}

GuideMap::~GuideMap()
{
    if(shortest_path != nullptr)
    {
        for (int i = 0; i<map_size; ++i)
        {
            for (int j = 0; j<map_size; ++j)
                delete[] shortest_path[i][j];
            delete[] shortest_path[i];
        }
        delete[] shortest_path;
    }
    if(min_cost != nullptr)
    {
        delete[] min_cost[0];
        delete[] min_cost;
    }

}