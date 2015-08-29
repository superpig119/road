#include <vector>
#include <map>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <queue>
#include <math.h>
#include "conf.h"
#include "quadtree.h"
#include "trajectory.h"

using namespace std;

typedef struct ROADTRAJECTORYUNIT
{
	double	x;
	double	y;
	struct	tm t;
}roadTrajectoryUnit;	//read from the trajectory

typedef struct ROADTRAJECTORY
{
	bool	direction;	//0:ID1->ID2 1:ID2->ID1
	vector<roadTrajectoryUnit> vRoadTrajectoryUnit;
}roadTrajectory;

typedef struct ROAD
{
	int		roadID;
	int		ID1;	//one end ID
	int		ID2;	//the other end ID
	float	length;
	vector<pair<double, double> > vpRoadDetail;	//Road line detail
	vector<roadTrajectory> vRoadTrajectory;		//The trajectory attached to this road
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

	int		buildGraph();	//Build the roadnetwork
	void	testGraph();
    int    	buildQuadTree();//Build the index of the nodes
	void	testQuadTree();
	void	updateMMXY(double x, double y);//update the min/max XY

    void    attachTrajectory();	//Attach the trajectory to the road
	void	matchTrajectory(vector<trajectoryUnit> vTU);
	void	posMatchRoad(double px, double py, vector<int>& vRoadList, double &x, double &y);
	double	nodeDist(double x1, double y1, double x2, double y2);

	float	distanceDijkstra(int ID1, int ID2);
	
	
//	int loadGraph();
//	int writeGraph();
};

