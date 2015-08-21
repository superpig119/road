#include <iostream>
#include <vector>
#include <map>
#include <sstream>
#include <iomanip>
#include "conf.h"

using namespace std;

typedef struct ROAD
{
	int roadID;
	int ID1;	//one end ID
	int ID2;	//the other end ID
	float length;
	vector<pair<double, double> > vpRoadDetail;
}roadInfo;

typedef struct NODE
{
	int ID;
	double x;
	double y;
	map<int, float> mNeighborLength;	//ID, length
	map<int, int> mNeighborRoad;	//ID, RoadID
}node;

typedef struct GRAPH
{
	vector<node> vNode;
	vector<roadInfo> vRoad;
}graph;

class RoadNetwork
{
public:
	graph g;

	int buildGraph();
	void testGraph();
//	int loadGraph();
//	int writeGraph();
};
