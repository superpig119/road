#include "road.h"
//#include "trajectory.h"

int main()
{
	RoadNetwork rn;
	rn.buildGraph();
	
	vector<double> vRoadList;
	vector<double>::iterator ivRL;

//	float d = rn.distanceDijkstra(10918051, 5413660, vRoadList);
//	float d = rn.distanceDijkstra(5413664, 5413660, vRoadList);
//	float d = rn.distanceDijkstra(5413660, 10918051, vRoadList);
/*	cout << setprecision(15) << "distance:" << d << endl;
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
