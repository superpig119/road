#include "road.h"
#include "trajectory.h"

int main()
{
	RoadNetwork rn;
	rn.buildGraph();
	rn.buildQuadTree();
	rn.testQuadTree();
//	rn.testGraph();

//	cout << rn.distanceDijkstra(41,2) << endl;

//	Trajectory tr;
	rn.trajectory.readRawTrajectory();
	rn.matchTrajectory();

	return 0;
}
