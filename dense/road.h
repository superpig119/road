#include <vector>
#include <map>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <queue>
#include <set>
#include <math.h>
#include "conf.h"
#include <pthread.h>
#include <time.h> 
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
	
#define EARTH_RADIUS  6371.004
#define PI 3.1415926

using namespace std;

typedef struct ROAD
{
	double	roadID;		//A2
	double	ID1;		//one end ID
	double	ID2;		//the other end ID
	int		direction;	//A6
	float	length;		//A13
	bool	isolated;
	map<double, double>	mV;
	set<int>			sNeighborRoad;	//connected road,include main 
	map<int, double>		mAvgV;		//time slot number, average speed
	map<int, vector<double> >	mGraV;	//time slot number, speed
	map<int, double>	mCost;		//time slot number, travel time(sec)
	vector<pair<double, double> > vpRoadDetail;	//Road line detail
}roadInfo;

typedef struct NODE
{
	double	ID;		//A2
	int		type;	//A5,0123
	double	x;
	double	y;
	double	MainID;	//A8,type=1,2,3
	bool	isolated;
	vector<double>		vSubID;				//A9,A10,type=3
	map<double, float>	mNeighborLength;	//Length of all neighbor
//	map<double, int>	mMainNeighborRoad;	//A7,type=2,3
	map<double, int>	mSubNeighborRoad;	//A13
}node;


typedef struct GRAPH
{
	map<double, node>		mNode;	//ID, node
	map<double, roadInfo>	mRoad;	//ID, roadInfo
}Graph;

class RoadNetwork
{
public:
	Graph	g;
	Conf	conf;

	int		buildGraph();	//Build the roadnetwork
	int		readRoad();
	int		readNode();
	int		readTrajectory();
	void	testGraph();
	
	int		readSpeed();
	void	outputSpeed();
	void	organizeSpeed();
	int		readAvgSpeed(map<int, vector<int> > &mTNumRoad);
	void	fillVoidSpeedST(map<int, vector<int> > &mTNumRoad);
	int		readCost();

	double	nodeDist(double x1, double y1, double x2, double y2);
	float	distanceDijkstra(double ID1, double ID2, vector<double>& vRoadList);
	vector<string> split(const string &s, const string &seperator);

	map<double, double> mIDTrans;	//Original,Order
	map<double, double> mRIDTrans;	//Order,Original

	int		T;	//Speed interval size
	int		TN;	//Speed interval number
};

