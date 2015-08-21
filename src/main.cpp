#include "road.h"

int main()
{
	RoadNetwork rn;
	rn.buildGraph();
//	rn.testGraph();

	cout << rn.distanceDijkstra(41,2) << endl;

	return 0;
}
