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
#include <numeric>
	
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
	map<int, double>	mMinV;
	map<int, double>	mSpanV;
	map<double, vector<double> >	mV;
	set<int>					sNeighborRoad;	//connected road,include main 
	map<int, vector<double> >	mGraV;		//time slot number, speed
	map<int, vector<double> >	mGraC;		//time slot number. cost
	map<int, int>				mVC;		//time slot number, roadCID
	map<int, double>			mCost;		//time slot number, travel time(sec)
	map<int, double>			mAvgV;		//time slot number, average speed
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
	double	neighborNode;	//special case, A12
	vector<double>		vSubID;				//A9,A10,type=3
	map<double, float>	mNeighborLength;	//Length of all neighbor
//	map<double, int>	mMainNeighborRoad;	//A7,type=2,3
	map<double, int>	mSubNeighborRoad;	//A13
	vector<int>			vRoad0;	//A13 when A5=0
}node;


typedef struct GRAPH
{
	map<double, node>		mNode;	//ID, node
	map<double, roadInfo>	mRoad;	//ID, roadInfo
}Graph;

typedef struct RCU
{
	vector<pair<int, int> > vpRT;	//roadID, t
	double avg;
	double dev;
}rcu;

typedef struct trajectorybrief
{
	int		id;
	double	sTimeStamp;	
	double	eTimeStamp;	
	int		sTime;		//second in day
	int		eTime;		
	double	sx,sy,ex,ey;
	vector<int>	vRoad;
}TB;

typedef struct DRIVER
{
	double	driverID;
	map<int, vector<double> >	mRVtmp;	//road type, speed history
	map<int, double>			mRV;	//road type, estimate speed
	vector<int>					vRoad;
	vector<double>				vTime;
	vector<int>					vSpeed;
	vector<TB>					vTB;
}driver;

typedef struct TRAVELTIMEUNIT
{
	int		t1;	//interval start
	int		t2;	//interval end
	double	a;	//(1/v1-v2)	
	double	b;	//(d/v2)
	double	cost;//bc=false,costant cost
	double	min;//min cost
	bool	bc;	//true:constant false:function
}travelTimeUnit;

typedef struct TRAVELTIME
{
	int		t1;
	double	min;
	int		minIndex;
	map<int, travelTimeUnit> vTTU; //start time,travel info
	map<int, vector<double> > mvRoad;//each travelTime Unit's trajectory
}travelTimeInfo;

class RoadNetwork
{
public:
	Graph	g;
	Conf	conf;
	vector<driver>	vDriver;

	int		buildGraph();	//Build the roadnetwork
	int		readNodeMap();
	int		readRoad();
	int		readNode();
	int		readTrajectory();
	void	testGraph();
	
	int		readSpeed();
	void	outputSpeed();	//output the speed in slots,for distribution analysis
	void	organizeSpeed();
	int		readAvgSpeed(map<int, vector<int> > &mTNumRoad);
	void	fillVoidSpeedST(map<int, vector<int> > &mTNumRoad);
	int		readTotalAvgSpeed();
	int		readCost();

	double	nodeDist(double x1, double y1, double x2, double y2);
	float	distanceDijkstraA(double ID1, double ID2, vector<int>& vRoadList);
	vector<string> split(const string &s, const string &seperator);

	double	shortestTimeDij(double ID1, double ID2, int t1, vector<int>& vRoadList, vector<double>& vRTime, vector<double>& vRTakeTime, double &d);
	double	FastestSingleTimePoint(double ID1, double ID2, int startTime, vector<int> &vRoadList, vector<double> &vRTime, vector<double> & vRTakeTime, double &distance);
	void	IntervalSingleFastestPaths(double ID1, double ID2, vector<int>& vRoadList, int t1, int t2, vector<int>& vt);	//ICDE2006
	void	testISFP();
	int		getTimeInterval(int t1, int t2, double roadID, vector<pair<int, int> > &vpTI);

	void	testDijA();
	void	testTimeDij();
	vector<pair<double, double> >	generateNodePair(int N);
	double	rad(double d);
	double coorDistance(double ID1, double ID2);

	void	driverAttach();
	void	speedRawClassify();	//
	void	calMinSpan();	//road's min&span
	void	readRC();		//read road's category
	int		readDriverSpeed();	//get driver's speed from trajectory
	int		driverAvg();	//average the speed in driverRCDetail file

	void	testDriverFastest();
	void	readDriver();	//build up driver before shortest path
	int		readMinSpan();	//read road's minV spanV file
	int		readDriverProfile();//read driverRC file
	double	shortestTimeDijDriver(double ID1, double ID2, int t1, vector<int>& vRoadList, vector<double>& vRTime, vector<double>& vRTakeTime, double &d, double driverID);
	double	getDriverCost(int driverID, int roadID, int t, map<int, map<int, double > > &mmCost, map<int, map<int, double> > &mmV);
	double  getDriverV(int driverID, int roadID, int t, map<int, map<int, double> > &mmV);

	int		extractBriefTrajectory();
	int		readBriefTrajectory();
	double	testDriver();
	double	testAvg();
	double	recreatePathTimeDriver(int driverID, int trajectoryID, bool &diff, int testNO);
	double	recreatePathTimeAvg(int driverID, int trajectoryID);
	int		extractTrajectoryTime();

	map<double, double> mIDTrans;	//Original,Order
	map<double, double> mRIDTrans;	//Order,Original
	map<double, double> mNodeMap;
	map<int, rcu>		mRCU;		//CID,rcu
 
	int		T;	//Speed interval size
	int		TN;	//Speed interval number
};

