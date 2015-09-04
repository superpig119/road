#include <vector>
#include <map>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <queue>
#include <set>
#include <math.h>
#include "conf.h"
#include "quadtree.h"
#include "trajectory.h"
#include <pthread.h>

using namespace std;

typedef struct ROADTRAJECTORYUNIT
{
	double	x;
	double	y;
	time_t	t;
}roadTrajectoryUnit;	//read from the trajectory

typedef struct ROADTRAJECTORY
{
	bool	direction;	//0:ID1->ID2 1:ID2->ID1
	vector<roadTrajectoryUnit> vRoadTrajectoryUnit;
}roadTrajectory;

typedef struct TIME
{
	int hour;
	int minute;
	int second;
}road_time;

typedef struct ROAD
{
	int		roadID;
	int		ID1;	//one end ID
	int		ID2;	//the other end ID
	float	length;
	vector<pair<double, double> > vpRoadDetail;	//Road line detail
//	vector<roadTrajectory> vRoadTrajectory;		//The trajectory attached to this road
//	map<road_time, double>	mInboundV;	//From ID1 to ID2, time and speed
//	map<road_time, double>	mOutboundV;	//From ID2 to ID1
//	map<road_time, double>	mTV;
	map<int, double>	mTV;	//time, speed
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
	void	updateMMXY(double x, double y);	//update the min/max XY

    void    attachTrajectory();				//Attach the trajectory to the road
	void	trajectoryMatchRoads(taxiTrajectory &tt, int i);
	void	posMatchRoad(double px, double py, int &roadID, double &x, double &y);
	void	nearestTwoNodeOnRoad(int roadID, double px, double py, double &x1, double &y1, double &x2, double &y2);
	double	nodeDist(double x1, double y1, double x2, double y2);
	void	findNStepNeighbor(int nodeID, int N, map<int, float> &mNeighbor, map<int, int> &mPrevious);
	void	testAttachTrajectory(set<int> sRoad);
	void	testRoadSpeed();
	int		writeRoadSpeed();
	int		readRoadSpeed();

	float	distanceDijkstra(int ID1, int ID2, vector<int>& vRoadList);
	double	distanceDijkBetween2Pair(int n11, int n12, double rLength, int n21, int n22, int &type, vector<int> &vRoadList);
	double	distanceAnyNodePair(double x1, double y1, double x2, double y2, vector<int>& vRoadList, vector<double> &vLength);	//return distance between any two points and the road segments involved, and the length
	void	distanceToEnds(double x, double y, int roadID, double &d1, double &d2);	//d1 distance to ID1, d2 distance to ID2
	double	pointToRoadDist(double px, double py, double rx1, double ry1, double rx2, double ry2);	//dist from px,py to road, return large number if px,py is not on road segment
	double	sameRoadDist(double px1, double py1, double px2, double py2, int roadID);	//1 and 2 on the same road segment, return the distance

	road_time	addTime(road_time t, int sec);
	
	
//	int loadGraph();
//	int writeGraph();
};

