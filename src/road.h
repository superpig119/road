#include <vector>
#include <map>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <queue>
#include "conf.h"
#include "quadtree.h"

using namespace std;

typedef struct ROAD
{
	int		roadID;
	int		ID1;	//one end ID
	int		ID2;	//the other end ID
	float	length;
	vector<pair<double, double> > vpRoadDetail;
}roadInfo;

typedef struct NODE
{
	int		ID;
	double	x;
	double	y;
	map<int, float>	mNeighborLength;//ID, length
	map<int, int>	mNeighborRoad;	//ID, RoadID
}node;

typedef struct GRAPH
{
	vector<node>		vNode;
	vector<roadInfo>	vRoad;
}Graph;

class RoadNetwork
{
public:
	Graph		g;
	double		minX, minY, maxX, maxY;
	Quadtree*	qt;

	int		buildGraph();
	void	testGraph();
	void	buildQuadTree();
	void	testQuadTree();
	void	updateMMXY(int x, int y);//update the min/max XY

	float	distanceDijkstra(int ID1, int ID2);
//	int loadGraph();
//	int writeGraph();
};

