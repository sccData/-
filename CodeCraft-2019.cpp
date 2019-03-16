
#include "mylib.h"
#include "road.h"
#include "cross.h"
#include "map.h"

using namespace std;


struct Car
{
    int ID;
    int origin,destination;
    int maxSpeed;
    int StartUpTime;
    struct {
        vector<int> routine;
        int start_time;
    } answer;

};

struct Car_Info
{
    int NumCars;
    vector<Car> Cars;
    unordered_map<int,int> CarMap;

    static bool comp(const Car &i1, const Car &i2)
    {
        if(i1.StartUpTime == i2.StartUpTime)
            return (i1.ID < i2.ID);
        else
            return (i1.StartUpTime < i2.StartUpTime);
    }
};

int main(int argc, char *argv[])
{
    std::cout << "Begin" << std::endl;
	
	if(argc < 5){
		std::cout << "please input args: carPath, roadPath, crossPath, answerPath" << std::endl;
		exit(1);
	}
	
	std::string carPath(argv[1]);
	std::string roadPath(argv[2]);
	std::string crossPath(argv[3]);
	std::string answerPath(argv[4]);
	
	std::cout << "carPath is " << carPath << std::endl;
	std::cout << "roadPath is " << roadPath << std::endl;
	std::cout << "crossPath is " << crossPath << std::endl;
	std::cout << "answerPath is " << answerPath << std::endl;

	/*-------------------------Read File--------------------------------------------*/


    Road_Info RoadInfo;
    RoadInfo.initialize(roadPath);

    Cross_Info CrossInfo;
    CrossInfo.initialize(crossPath,RoadInfo);


    GuideMap guide_map(CrossInfo);
    guide_map.calculate_map(CrossInfo,RoadInfo);

	RoadInfo.get_road_neighbor(CrossInfo);

    Car_Info CaRInfo;


    ifstream fin;
    fin.open(carPath.c_str());

    if(!fin.is_open())
    {
        cerr<<"Error! Cannot open Road file!\n";
        exit(-1);
    }

    string useless;
    char HeadCharacter;

    while(!fin.eof())
    {
        HeadCharacter = fin.get();
        if(HeadCharacter == '#')
            getline(fin,useless);
        else if(HeadCharacter == '(')
        {
            Car temp;
            fin>>temp.ID;
            fin>>useless;
            fin>>temp.origin;
            fin>>useless;
            fin>>temp.destination;
            fin>>useless;
            fin>>temp.maxSpeed;
            fin>>useless;
            fin>>temp.StartUpTime;
            getline(fin,useless);
            CaRInfo.Cars.push_back(temp);
        }
    }
    CaRInfo.NumCars = CaRInfo.Cars.size();

    fin.close();
    sort(CaRInfo.Cars.begin(), CaRInfo.Cars.end(), CaRInfo.comp);
    for(int i = 0;i < CaRInfo.Cars.size();i++)
        CaRInfo.CarMap.insert(make_pair(CaRInfo.Cars[i].ID,i));

    srand(time(0));
    int start_time = CaRInfo.Cars[0].StartUpTime;
    for(int i = 0;i<CaRInfo.NumCars;i++)
    {
        int count = 0;
        Car &current_car = CaRInfo.Cars[i];
        int current_cross = current_car.origin;
        int current_road;
        current_car.answer.start_time = start_time;
        start_time += ((guide_map.min_cost[CrossInfo.CrossMap[current_car.origin]][CrossInfo.CrossMap[current_car.destination]] / current_car.maxSpeed) + 1);
        start_time %= 997;
        start_time = max(start_time,current_car.StartUpTime);
        while(current_cross != current_car.destination )
        {
            if(count > 4*CrossInfo.total_cross_num())
            {
                cerr<<"Wrong! Unstoppable loop!\n";
                exit(-1);
            }

            if(guide_map.MapMatrix[current_cross][current_car.destination].MyChoice[0] == 1)
                current_road = guide_map.MapMatrix[current_cross][current_car.destination].MyChoice[1];
            else if(guide_map.MapMatrix[current_cross][current_car.destination].MyChoice[0] == 2)
                current_road = guide_map.MapMatrix[current_cross][current_car.destination].MyChoice[rand() % 2 + 1];
            else if(guide_map.MapMatrix[current_cross][current_car.destination].MyChoice[0] == 3)
                current_road = guide_map.MapMatrix[current_cross][current_car.destination].MyChoice[rand() % 3 + 1];
            else if(guide_map.MapMatrix[current_cross][current_car.destination].MyChoice[0] == 4)
                current_road = guide_map.MapMatrix[current_cross][current_car.destination].MyChoice[rand() % 4 + 1];

            if(current_road == -1)
            {
                cerr<<"Wrong! This car cannot reach the destination!\n";
                exit(-1);
            }

            current_car.answer.routine.push_back(current_road);
            current_cross = RoadInfo.get_corresponding_cross(current_road,current_cross);
            count++;
        }
    }

    ofstream fout;
    fout.open(answerPath.data());

    fout<<"#(carID,StartTime,RoadId...)\n";
    for(auto ite = CaRInfo.Cars.begin();ite != CaRInfo.Cars.end();ite++)
    {
        fout<<"("<<ite->ID<<","
            <<ite->answer.start_time<<",";
        for(int i = 0;i<ite->answer.routine.size()-1;i++)
            fout<<ite->answer.routine[i]<<",";
        fout<<ite->answer.routine[ite->answer.routine.size()-1]<<")\n";
    }

    fout.close();

	// TODO:read input filebuf
	// TODO:process
	// TODO:write output file

	return 0;
}