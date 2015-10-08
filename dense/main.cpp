#include "road.h"
//#include "trajectory.h"

int main()
{
	RoadNetwork rn;
	rn.buildGraph();
//	rn.testDijA();
//	rn.testTimeDij();
	
	vector<int> vRoadList;
	vector<int>::iterator ivRL;
/*
	vector<double> vRTime;
	vector<double>::iterator ivRT;

	vector<double> vRTakeTime;
	vector<double>::iterator ivRTT;

	int time = 23;
	double id1 = 10918051;
	double id2 = 5413660;

	cout << setprecision(15) << "Query:" << id1 << "\t" << id2 << endl;
	float t = rn.shortestTimeDij(id1, id2, 23, vRoadList, vRTime, vRTakeTime);
	for(ivRL = vRoadList.begin(), ivRT = vRTime.begin(), ivRTT = vRTakeTime.begin(); ivRL != vRoadList.end(); ivRL++, ivRT++, ivRTT++)
		cout << "roadID:" <<*ivRL << "\tstart time:" << *ivRT << "\ttake time:" << *ivRTT << endl;;
*/
//	float d = rn.distanceDijkstra(10918051, 5413660, vRoadList);
//	float d = rn.distanceDijkstra(5413664, 5413660, vRoadList);
//	float d = rn.distanceDijkstra(5413660, 10918051, vRoadList);
/*	float d = rn.distanceDijkstra(5261638,	47643793, vRoadList);
	cout << setprecision(15) << "distance:" << d << endl;
	for(ivRL = vRoadList.begin(); ivRL != vRoadList.end(); ivRL++)
		cout << *ivRL << "\t";
	cout << endl;*/
//	rn.testGraph();

//	cout << rn.distanceDijkstra(41,2) << endl;

//	Trajectory tr;
//rn.trajectory.readRawTrajectory();
//	rn.attachTrajectory();
//	rn.writeRoadSpeed();
//	rn.readRoadSpeed();
//	rn.testRoadSpeed();

	return 0;
}
