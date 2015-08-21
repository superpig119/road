#include "road.h"

int main()
{
	RoadNetwork rn;
	rn.buildGraph();
//	rn.testGraph();

	rn.distanceDijkstra(41,2);

	return 0;
}
