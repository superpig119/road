#include <vector>
#include <map>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <queue>
#include "conf.h"
#include "quadtree.h"
#include "trajectory.h"

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
    Trajectory  trajectory;

	int		buildGraph();
	void	testGraph();
    int    	buildQuadTree();
	void	testQuadTree();
	void	updateMMXY(double x, double y);//update the min/max XY
    void    matchTrajectory(taxiTrajectory tt);

	float	distanceDijkstra(int ID1, int ID2);
//	int loadGraph();
//	int writeGraph();
};

