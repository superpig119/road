#include "road.h"

#define INF 99999999

typedef struct heap
{
	pair<int, float> pif;//ID, distance
	bool operator < (const struct heap &a) const  
	{  
		return pif.second > a.pif.second; 
	} 
}h;

bool operator <(road_time rt1, road_time rt2)
{
	if(rt1.hour < rt2.hour)
		return true;
	else if(rt1.hour > rt2.hour)
		return false;
	else
	{
		if(rt1.minute < rt2.minute)
			return true;
		else if(rt1.minute > rt2.minute)
			return false;
		else
		{
			if(rt1.second <= rt2.second)
				return true;
			else if(rt1.second > rt2.second)
				return false;
		}
	}
}

int RoadNetwork::buildGraph()
{
	Conf conf;
	if(conf.readConf())
	{
		return -1;
	}
	
    int count = 0;
	maxX = 0;
	maxY = 0;
	minX = 90;
	minY = 180;

	ifstream ifile(conf.roadFilePath.c_str());
	cout << conf.roadFilePath << endl;
	if(!ifile)
	{
		cout << "Cannot open data file" << endl;
		return -1;
	}

	int nodeNum, roadNum, i, j;
	string stmp;

	ifile >> nodeNum;
	cout << "nodeNum:" << nodeNum << endl;
	setprecision(15);
	for(i = 0; i < nodeNum; i++)
	{
		node n;
		ifile >> n.ID;
		ifile >> n.x;
		ifile >> n.y;
		updateMMXY(n.x, n.y);
		g.vNode.push_back(n);
        count++;
	}

	ifile >> roadNum;
	cout << "roadNum:" << roadNum << endl;
	bool f;
	int pos, ID1, ID2;
	double x, y, xtmp;
	stringstream ss;

	int roadID = 0;
	for(i = 0; i < roadNum; i++)
	{
		roadInfo ri;
		ri.roadID = roadID++;
		ifile >> ri.ID1;
		ifile >> ri.ID2;
		ifile >> ri.length;
		for(j = 0; j < 8; j++)
			ifile >> stmp;
		
		g.vNode[ri.ID1].mNeighborLength[ri.ID2] = ri.length;
		g.vNode[ri.ID2].mNeighborLength[ri.ID1] = ri.length;

		g.vNode[ri.ID1].mNeighborRoad[ri.ID2] = ri.roadID;
		g.vNode[ri.ID2].mNeighborRoad[ri.ID1] = ri.roadID;

		f = true;
		ifile >> x;
		while(f)
		{
			ifile >> stmp;
			pos = stmp.find(',');
			if(pos != string::npos)
			{
				ss.clear();
				ss.str("");
				ss <<  stmp.substr(0, pos);
				ss >> y;
				ri.vpRoadDetail.push_back(make_pair(x,y));
				
				ss.clear();
				ss.str("");
				ss << stmp.substr(pos + 1, stmp.length() - pos - 1);
				ss >> x;
			}
			else
			{
				f = false;
				ss.clear();
				ss.str("");
				ss << stmp;
				ss >> y;
				ri.vpRoadDetail.push_back(make_pair(x,y));
				updateMMXY(x, y);
                count++;
			}
		}
		ri.vpRoadDetail[0].first = g.vNode[ri.ID1].x;
		ri.vpRoadDetail[0].second = g.vNode[ri.ID1].y;
		ri.vpRoadDetail[ri.vpRoadDetail.size() - 1].first = g.vNode[ri.ID2].x;
		ri.vpRoadDetail[ri.vpRoadDetail.size() - 1].second = g.vNode[ri.ID2].y;
		g.vRoad.push_back(ri);
	}
    cout << setprecision(15) << "minX:" << minX << "\tmaxX:" << maxX << "\tminY:" << minY << "\tmaxY:" << maxY << endl;
    cout << "Total node number:" << count << endl;
	return 0;
}

void RoadNetwork::testGraph()
{
	vector<node>::iterator ivNode;
	vector<roadInfo>::iterator ivRoad;
	map<int, float>::iterator imNL;	//ID, length
	vector<pair<double, double> >::iterator ivpRD;
	cout << "Road size " << g.vRoad.size() << endl;
	for(ivRoad = g.vRoad.begin(); ivRoad != g.vRoad.end(); ivRoad++)
		cout << (*ivRoad).roadID << "\t" << (*ivRoad).ID1 << "\t" << (*ivRoad).ID2 << "\t" << (*ivRoad).length << endl;
/*
	for(ivNode = g.vNode.begin(); ivNode != g.vNode.end(); ivNode++)
	{
		cout << "Intersection ID:" << (*ivNode).ID << endl;
		cout << "Coordinates:" << setprecision(15) << (*ivNode).x << " " << (*ivNode).y << endl;
		cout << "Neighbors:" << endl;
		for(imNL = (*ivNode).mNeighborLength.begin(); imNL != (*ivNode).mNeighborLength.end(); imNL++)
		{
			cout << (*imNL).first << "\tdistance:" << (*imNL).second << "\tCoordinate:" << g.vNode[(*imNL).first].x << " " << g.vNode[(*imNL).first].y << "\tRoadID:" << (*ivNode).mNeighborRoad[(*imNL).first] << endl;
			cout << "RoadDetail:";
			cout << g.vRoad[(*ivNode).mNeighborRoad[(*imNL).first]].roadID << endl;
			for(ivpRD = g.vRoad[(*ivNode).mNeighborRoad[(*imNL).first]].vpRoadDetail.begin(); ivpRD != g.vRoad[(*ivNode).mNeighborRoad[(*imNL).first]].vpRoadDetail.end(); ivpRD++)
			{
				cout << setprecision(15) << (*ivpRD).first << "\t" << (*ivpRD).second << endl;
			}
		}
		cout << endl;
	}*/
}

int RoadNetwork::buildQuadTree()
{
	Conf conf;
	if(conf.readConf())
	{
		return -1;
	}

	cout << "Constructing QuadTree" << endl;
	qt = new Quadtree(minX, minY, maxX - minX, maxY - minY, 1, conf.QuadTreeLevel);
	vector<node>::iterator		ivNode;
	vector<roadInfo>::iterator	ivRoad;
	map<int, int>::iterator		imNR;
	vector<pair<double, double> >::iterator ivpRD;

	cout << "Building QuadTree with Nodes" << endl;
	for(ivNode = g.vNode.begin(); ivNode != g.vNode.end(); ivNode++)
	{
		simpleNode sn;
		sn.x = (*ivNode).x;
		sn.y = (*ivNode).y;
		for(imNR = (*ivNode).mNeighborRoad.begin(); imNR != (*ivNode).mNeighborRoad.end(); imNR++)
		{
			sn.vRoadList.push_back((*imNR).second);
		}
		if((*ivNode).x == 39.9154218077778 && (*ivNode).y == 116.359934796944)
		{
			for(imNR = (*ivNode).mNeighborRoad.begin(); imNR != (*ivNode).mNeighborRoad.end(); imNR++)
				cout << "ROADID:" << (*imNR).second << endl;
		}
		qt->AddNode(sn);
	}
		
	cout << "Building QuadTree with Roads" << endl;
	for(ivRoad = g.vRoad.begin(); ivRoad != g.vRoad.end(); ivRoad++)
	{
		for(ivpRD = (*ivRoad).vpRoadDetail.begin() + 1;ivpRD != (*ivRoad).vpRoadDetail.end() - 1; ivpRD++)
		{
			simpleNode sn;
			sn.x = (*ivpRD).first;
			sn.y = (*ivpRD).second;
			sn.vRoadList.push_back((*ivRoad).roadID);
			qt->AddNode(sn);
		}
	}
	cout << "QuadTree finish!" << endl;

    return 0;
}
	
void RoadNetwork::testQuadTree()
{
	cout << "Testing QuadTree" << endl;
	vector<simpleNode> vs;
	vector<simpleNode>::iterator ivs;
    vector<int>::iterator ivR;
    Quadtree *qtt;
    qtt = qt->NW->SE->NW->SE->SW;
    for(ivs = qtt->vSimpleNode.begin(); ivs != qtt->vSimpleNode.end(); ivs++)
    {
/*		cout << setprecision(15) << (*ivs).x << "\t" << (*ivs).y << "\troadID:";
        for(ivR = (*ivs).vRoadList.begin(); ivR != (*ivs).vRoadList.end(); ivR++)
            cout << *ivR << "\t";
        cout << endl;
 */   }
    cout << "region size:" << qtt->vSimpleNode.size() << endl;

	qtt = qt->getRegion(39.9632504625,116.109541320556);
    cout << "Return region size:" << qtt->vSimpleNode.size() << endl;
/*	for(ivs = vs.begin(); ivs != vs.end(); ivs++)
	{
		cout << setprecision(15) << (*ivs).x << "\t" << (*ivs).y << endl;
	}*/
}
    
void RoadNetwork::updateMMXY(double x, double y)
{
	if(x > maxX)
		maxX = x;
	if(x < minX)
		minX = x;
	if(y > maxY)
		maxY = y;
	if(y < minY)
		minY = y;
}

void RoadNetwork::attachTrajectory()
{
 	vector<taxiTrajectory>::iterator ivT;
	vector<trajectoryUnit>::iterator ivTU;
	int i;
	while(trajectory.readNextTrajectory())
	{
		for(ivT = trajectory.vTrajectory.begin(); ivT != trajectory.vTrajectory.end(); ivT++, i++)
		{
			cout << endl << "Analyzing Trajectory No." << i << endl << endl;;
/*		for(ivTU = (*ivT).vTU.begin(); ivTU != (*ivT).vTU.end(); ivTU++)
		{
			cout << setprecision(15) << (*ivTU).x << "," << (*ivTU).y << "," << ctime(&(*ivTU).t) << endl;
		}*/
			trajectoryMatchRoads(*ivT, i);
//			break;	//Test for the first trajectory
		}
		trajectory.vTrajectory.clear();
	}
}

void RoadNetwork::trajectoryMatchRoads(taxiTrajectory &tt, int i)
{
	vector<int> vRoadList;
	vector<int>::iterator ivR;
	vector<double> vLength;
	vector<double>::iterator ivL;
	double x, y, xtmp, ytmp;
	double distance;
	double v;
	time_t t, ttmp;
	int ts;
	vector<trajectoryUnit>::iterator ivTU;
    vector<roadTrajectory>::iterator ivRT;
	int j = 0;
    for(ivTU = tt.vTU.begin(); ivTU != tt.vTU.end() - 1; ivTU++, j++)
	{
		if(j % 30 == 0)
			cout << endl << "Trajectory " << i << endl << endl;;
		distance = distanceAnyNodePair((*ivTU).x, (*ivTU).y, (*(ivTU + 1)).x, (*(ivTU + 1)).y, vRoadList, vLength);
		v = distance / 30;
		cout << "distance:" << distance << endl;
		cout << "v:" << v << endl;
/*		cout << setprecision(15) << "From " << (*ivTU).x << "," << (*ivTU).y << " to " << (*(ivTU+1)).x << "," << (*(ivTU+1)).y << endl;
		for(ivR = vRoadList.begin(); ivR != vRoadList.end(); ivR++)
		{
			cout << "roadID:" << *ivR << "\t"  << g.vNode[g.vRoad[*ivR].ID1].ID << ": " << g.vNode[g.vRoad[*ivR].ID1].x << "," << g.vNode[g.vRoad[*ivR].ID1].y << "\t" << g.vNode[g.vRoad[*ivR].ID2].ID << ": " << g.vNode[g.vRoad[*ivR].ID2].x << "," << g.vNode[g.vRoad[*ivR].ID2].y << endl;
		}
		cout << endl;*/
		road_time tt;
		struct tm * ptm;
		ptm = gmtime(&((*ivTU).t));
		tt.hour = ptm->tm_hour;
		tt.minute = ptm->tm_min;
		tt.second = ptm->tm_sec;
		for(ivR = vRoadList.begin(), ivL = vLength.begin(); ivR != vRoadList.end(); ivR++, ivL++)
		{
			g.vRoad[*ivR].mTV[tt] = v;
			ts = 30 * (*ivL / distance);
			tt = addTime(tt, ts);
		}
		vRoadList.clear();
		vLength.clear();
	}
}

void RoadNetwork::posMatchRoad(double px, double py, int& roadID, double &x, double &y)
{
    Quadtree *qtt;
	qtt = qt->getRegion(px, py);
	double d = 9999999;
	double dtmp;
    vector<simpleNode>::iterator ivSN;
	vector<int>::iterator ivR;
	double x1,x2,y1,y2;
	for(ivSN = qtt->vSimpleNode.begin(); ivSN != qtt->vSimpleNode.end(); ivSN++)
	{
		dtmp = nodeDist(px, py, (*ivSN).x, (*ivSN).y);
		if(dtmp > 300)
			continue;

		for(ivR = (*ivSN).vRoadList.begin(); ivR != (*ivSN).vRoadList.end(); ivR++)
		{
			nearestTwoNodeOnRoad(*ivR, px, py, x1, y1, x2, y2);
			dtmp = pointToRoadDist(px, py, x1, y1, x2, y2);
			if(dtmp < d)
			{
				d = dtmp;
				x = (*ivSN).x;
				y = (*ivSN).y;
				roadID = *ivR;
			}
		}
	}
}

void RoadNetwork::nearestTwoNodeOnRoad(int roadID, double px, double py, double &x1, double &y1, double &x2, double &y2)
{
	vector<pair<double, double> >::iterator ivpRD;
	double d1, d2, dtmp;
	d1 = nodeDist(px, py, g.vRoad[roadID].vpRoadDetail[0].first, g.vRoad[roadID].vpRoadDetail[0].second);
	d2 = nodeDist(px, py, g.vRoad[roadID].vpRoadDetail[1].first, g.vRoad[roadID].vpRoadDetail[1].second);

	x1 = g.vRoad[roadID].vpRoadDetail[0].first;
	y1 = g.vRoad[roadID].vpRoadDetail[0].second;
	x2 = g.vRoad[roadID].vpRoadDetail[1].first;
	y2 = g.vRoad[roadID].vpRoadDetail[1].second;

	for(ivpRD = g.vRoad[roadID].vpRoadDetail.begin() + 2; ivpRD != g.vRoad[roadID].vpRoadDetail.end(); ivpRD++)
	{
		dtmp = nodeDist(px, py, (*ivpRD).first, (*ivpRD).second);
		if(dtmp < d1 && d1 < d2)
		{
			d2 = dtmp;
			x2 = (*ivpRD).first;
			y2 = (*ivpRD).second;
		}
		else if(dtmp < d1 && d1 > d2)
		{
			d1 = dtmp;
			x1 = (*ivpRD).first;
			y1 = (*ivpRD).second;
		}
	}
}

double RoadNetwork::nodeDist(double x1, double y1, double x2, double y2)
{
	return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

void RoadNetwork::testRoadSpeed()
{
	vector<roadInfo>::iterator ivR;
	map<road_time, double>::iterator imTV;
	for(ivR = g.vRoad.begin(); ivR != g.vRoad.end(); ivR++)
	{
		if(!(*ivR).mTV.empty())
		{
			cout << "Road ID:" << (*ivR).roadID << endl;;
			for(imTV = (*ivR).mTV.begin(); imTV != (*ivR).mTV.end(); imTV++)
			{
				cout <<	(*imTV).first.hour << ":" << (*imTV).first.minute << ":" << (*imTV).first.second << "\t" << (*imTV).second << endl;
			}
		}
	}
}
	
void RoadNetwork::findNStepNeighbor(int nodeID, int N, map<int, float> &mDistance, map<int, int> &mPrevious)
{
	if(N > 0)
	{
		map<int, float>::iterator imNL;
		N--;
		for(imNL = g.vNode[nodeID].mNeighborLength.begin(); imNL != g.vNode[nodeID].mNeighborLength.end(); imNL++)
		{
			mDistance[(*imNL).first] = INF;
			mPrevious[(*imNL).first] = -1;
			findNStepNeighbor((*imNL).first, N, mDistance, mPrevious);
		}
	}
}

int RoadNetwork::writeRoadSpeed()
{
	vector<roadInfo>::iterator ivR;
	map<road_time, double>::iterator imTV;
	ofstream outFile("speed");
	for(ivR = g.vRoad.begin(); ivR != g.vRoad.end(); ivR++)
	{
		if(!(*ivR).mTV.empty())
		{
			outFile << (*ivR).roadID << "\t" << (*ivR).mTV.size() << endl;
			for(imTV = (*ivR).mTV.begin(); imTV != (*ivR).mTV.end(); imTV++)
			{
				outFile <<	(*imTV).first.hour << "\t" << (*imTV).first.minute << "\t" << (*imTV).first.second << "\t" << (*imTV).second << endl;
			}
		}
	}
}
	
int	RoadNetwork::readRoadSpeed()
{
	ifstream ifile("speed");
	int roadID;
	int num, i;
	double v;
	while(ifile >> roadID)
	{
		ifile >> num;
		for(i = 0; i < num; i++)
		{
			road_time t;
			ifile >> t.hour;
			ifile >> t.minute;
			ifile >> t.second;
			ifile >> v;
			g.vRoad[roadID].mTV[t] = v;
		}
	}

}

float RoadNetwork::distanceDijkstra(int ID1, int ID2, vector<int> &vRoadList)
{
//	map<int, float> mDistance;
	vector<float> vDistance(g.vNode.size(), INF);
	vector<float>::iterator ivD;
	priority_queue<h> qh;
	vector<int> vPrevious(g.vNode.size(), -1);
	vector<int>::iterator ivP;
	map<int, float>::iterator imNL;

	vDistance[ID1] = 0;
	for(imNL = g.vNode[ID1].mNeighborLength.begin(); imNL != g.vNode[ID1].mNeighborLength.end(); imNL++)
	{
		vDistance[(*imNL).first] = (*imNL).second;
		h hh;
		hh.pif = make_pair((*imNL).first, (*imNL).second);
		qh.push(hh);
		vPrevious[(*imNL).first] = ID1;
	}

	pair<int, float> pu;
	while(!qh.empty())
	{
		pu = qh.top().pif;
		qh.pop();
		for(imNL = g.vNode[pu.first].mNeighborLength.begin(); imNL != g.vNode[pu.first].mNeighborLength.end(); imNL++)
		{
			if(vDistance[(*imNL).first] == INF && (*imNL).first != ID1)
			{
				float d = vDistance[pu.first] + (*imNL).second;
				vDistance[(*imNL).first] = d;
				h hh;
				hh.pif = make_pair((*imNL).first, d);
				qh.push(hh);
				vPrevious[(*imNL).first] = pu.first;
			}
			else if(vDistance[(*imNL).first] > vDistance[pu.first] + (*imNL).second)
			{
				vDistance[(*imNL).first] = vDistance[pu.first] + (*imNL).second;
				vPrevious[(*imNL).first] = pu.first;
			}
		}
	}
	
/*	for(imD = mDistance.begin(); imD != mDistance.end(); imD++)
	{
		if((*imD).second < 1000)
		cout << (*imD).first << "\tDistance:" << (*imD).second << endl;
	}*/
	
//	cout << setprecision(15) << "ID1:" << ID1 << "\t" << g.vNode[ID1].x << "\t" << g.vNode[ID1].y << endl;
//	cout << setprecision(15) << "ID2:" << ID2 << "\t" << g.vNode[ID2].x << "\t" << g.vNode[ID2].y << endl;

//	cout << "Path:" << endl;
	int id = ID2;
	int idtmp;
	vector<int> vRoadListtmp;
	vector<int>::reverse_iterator irvRL;
//	cout << "Distance:" << vDistance[ID2] << endl;
	if(vDistance[ID2] != INF)
	{	
		while(id != ID1)
		{
			idtmp = vPrevious[id];
//			cout << setprecision(15) << idtmp << "\tto\t" << id << "\tDistance:" << vDistance[id] << "\tCoordinate:\t" <<g.vNode[id].x << "\t" << g.vNode[id].y << endl;
			vRoadListtmp.push_back(g.vNode[id].mNeighborRoad[idtmp]);
			id = idtmp;
		}
	}

	for(irvRL = vRoadListtmp.rbegin(); irvRL != vRoadListtmp.rend(); irvRL++)
	{
		vRoadList.push_back(*irvRL);
	}

	return vDistance[ID2];
}
	
/*Find the shortest path between 2 road's 2 end nodes;
 * n11:ID1 of road1, n12:ID2 of road1
 * n21:ID1 of road2, n22:ID2 of road2
 * Calculate the distance from n11 to n21 & n22
 * Find the short one and test the path
 * if the path include n12, then n12 is the shortest
 * else n11 is the shortest
 * Return type:[0]n11&n21 [1]n12&n21
 *			   [2]n11&n22 [3]n127n22
 */
double RoadNetwork::distanceDijkBetween2Pair(int n11, int n12, double rLength, int n21, int n22, int &type, vector<int> &vRoadList)
{
	map<int, float> mDistance;
	map<int, int>	mPrevious;
	
	map<int, float>::iterator	imD;
	map<int, int>::iterator		imP;
	vector<node>::iterator		ivNode;
	map<int, float>::iterator	imNL;

	priority_queue<h> qh;
	mDistance[n11] = 0;

	findNStepNeighbor(n11, 4, mDistance, mPrevious);
	findNStepNeighbor(n12, 4, mDistance, mPrevious);
	findNStepNeighbor(n21, 4, mDistance, mPrevious);
	findNStepNeighbor(n12, 4, mDistance, mPrevious);

	cout << "mDistance size:" << mDistance.size() << endl;

	for(imNL = g.vNode[n11].mNeighborLength.begin(); imNL != g.vNode[n11].mNeighborLength.end(); imNL++)
	{
		mDistance[(*imNL).first] = (*imNL).second;
		h hh;
		hh.pif = make_pair((*imNL).first, (*imNL).second);
		qh.push(hh);
		mPrevious[(*imNL).first] = n11;
	}


	pair<int, float> pu;
	while(!qh.empty())
	{
		pu = qh.top().pif;
		qh.pop();
		for(imNL = g.vNode[pu.first].mNeighborLength.begin(); imNL != g.vNode[pu.first].mNeighborLength.end(); imNL++)
		{
			if(mDistance.find(((*imNL).first)) == mDistance.end())
				continue;

			if(mDistance[(*imNL).first] == INF && (*imNL).first != n11)
			{
				float d = mDistance[pu.first] + (*imNL).second;
				mDistance[(*imNL).first] = d;
				h hh;
				hh.pif = make_pair((*imNL).first, d);
				qh.push(hh);
				mPrevious[(*imNL).first] = pu.first;
			}
			else if(mDistance[(*imNL).first] > mDistance[pu.first] + (*imNL).second)
			{
				mDistance[(*imNL).first] = mDistance[pu.first] + (*imNL).second;
				mPrevious[(*imNL).first] = pu.first;
			}
		}
	}
	
	int id;
	int idtmp ;
	double dist;
	vector<int> vRoadListtmp;
	vector<int>::reverse_iterator irvRL;
	if(mDistance[n21] < mDistance[n22])
	{
		dist = mDistance[n21];
		cout << "NODE2:" << g.vNode[n21].x << "," << g.vNode[n21].y << endl;
		id = n21;
		type = 0;
	}
	else
	{
		dist = mDistance[n22];
		cout << "NODE2:" << g.vNode[n22].x << "," << g.vNode[n22].y << endl;
		id = n22;
		type = 2;
	}

	vector<int> vNodeList;
	if(mDistance[id] != INF)
	{	
		while(id != n11)
		{
			idtmp = mPrevious[id];
			if(idtmp == n12)
			{
				type++;
				dist -= rLength;
			}
			vRoadListtmp.push_back(g.vNode[id].mNeighborRoad[idtmp]);
			id = idtmp;
		}
	}

	for(irvRL = vRoadListtmp.rbegin(); irvRL != vRoadListtmp.rend(); irvRL++)
	{
		vRoadList.push_back(*irvRL);
	}

	return dist;
}
	
double RoadNetwork::distanceAnyNodePair(double x1, double y1, double x2, double y2, vector<int>& vRoadList, vector<double> &vLength)
{
	double sourceX, sourceY, desX, desY;
	vector<int> vRoadListtmp;
	vector<int>::iterator ivR;
	int roadID1, roadID2;
	int sourceID, desID;
	posMatchRoad(x1, y1, roadID1, sourceX, sourceY);
	posMatchRoad(x2, y2, roadID2, desX, desY);

	double ds1, ds2, dd1, dd2;
	distanceToEnds(x1, y1, roadID1, ds1, ds2);
	distanceToEnds(x2, y2, roadID2, dd1, dd2);
		
//	float d = distanceDijkstra(sourceID, desID, vRoadList);
	int c;	//implies source and des ID1 ID2
	double d, dtmp;
	if(roadID1 == roadID2)
	{
		d = sameRoadDist(x1, y1, x2, y2, roadID1);
//		cout << "On the same road " << roadID1;
	}
	else
	{
		cout << "From " << x1 << "," << y1 << " to " << x2 << "," << y2 << endl;
		cout << "Node1 attach to " << sourceX << "," << sourceY << endl;
		cout << "Node2 attach to " << desX << "," << desY << endl;
		cout << "roadID1:" << roadID1 << endl;
		cout << "ID11:" << g.vRoad[roadID1].ID1 << endl;
		cout << "ID12:" << g.vRoad[roadID1].ID2 << endl;
		cout << "roadID2:" << roadID2 << endl;
		cout << "road2 length:" << g.vRoad[roadID2].length << endl;
		cout << "ID21:" << g.vRoad[roadID2].ID1 << endl;
		cout << "ID21 coor:" << g.vNode[g.vRoad[roadID2].ID1].x << "," << g.vNode[g.vRoad[roadID2].ID1].y << endl;
		cout << "ID22:" << g.vRoad[roadID2].ID2 << endl;
		d = distanceDijkBetween2Pair(g.vRoad[roadID1].ID1, g.vRoad[roadID1].ID2, g.vRoad[roadID1].length, g.vRoad[roadID2].ID1, g.vRoad[roadID2].ID2, c, vRoadListtmp);

		distanceToEnds(x1, y1, roadID1, ds1, ds2);
		distanceToEnds(x2, y2, roadID2, dd1, dd2);
		cout << "Distance1:" << d << endl;

		double dstart, dend;

		if(c == 0)
		{
//			cout << "C = 0" << endl;
			d = ds1 + d + dd1;
			dstart = ds1;
			dend = dd1;
		}
		else if(c == 1)
		{
//			cout << "C = 1" << endl;
			d = ds2 + d + dd1;
			dstart = ds2;
			dend = dd1;
		}
		else if(c == 2)
		{
//			cout << "C = 2" << endl;
			d = ds1 + d + dd2;
			dstart = ds1;
			dend = dd2;
		}
		else if(c == 3)
		{
//			cout << "C = 3" << endl;
			d = ds2 + d + dd2;
			dstart = ds2;
			dend = dd2;
		}
		
		cout << "ds1:" << ds1 << endl;
		cout << "ds2:" << ds2 << endl;
		cout << "dd1:" << dd1 << endl;
		cout << "dd2:" << dd2 << endl;
		vLength.push_back(dstart);
		vRoadList.push_back(roadID1);
		for(ivR = vRoadListtmp.begin(); ivR != vRoadListtmp.end(); ivR++)
		{
			vRoadList.push_back(*ivR);
			vLength.push_back(g.vRoad[*ivR].length);
		}
		vRoadList.push_back(roadID2);
		vLength.push_back(dend);
		
		cout << "Distance2:" << d << endl;
//		cout << endl;
	}
	return d;
}

void RoadNetwork::distanceToEnds(double x, double y, int roadID, double &d1, double &d2)
{
	vector<pair<double, double> >::iterator ivpRD;
	vector<double> vLength;
	vector<double>::iterator ivL;
	int i = 0;
	int pos;
	double dp1, dp2;
	double dtmp, dtmp1, dtmp2;	//distance from x,y to two ends of the segment
	double dmin = 9999999;
	double dsum = 0;//sum of dpart
	double dpart;	//distance of each segment
	for(ivpRD = g.vRoad[roadID].vpRoadDetail.begin(); ivpRD != g.vRoad[roadID].vpRoadDetail.end() - 1; ivpRD++, i++)
	{
		dpart = nodeDist((*ivpRD).first, (*ivpRD).second, (*(ivpRD + 1)).first, (*(ivpRD + 1)).second);
		vLength.push_back(dpart);
		dsum += dpart;
		dtmp1 = nodeDist(x, y, (*ivpRD).first, (*ivpRD).second); 
		dtmp2 = nodeDist(x, y, (*(ivpRD + 1)).first, (*(ivpRD + 1)).second);
		dtmp = dtmp1 + dtmp2;
		if(dtmp < dmin)
		{
			dmin = dtmp;
			pos = i;
			dp1 = dtmp1 / dtmp * dpart;
			dp2 = dtmp2 / dtmp * dpart;
		}
	}

//	cout << "dsum:" << dsum << endl;
	d1 = d2 = 0;
//	cout << "Road Length:" << g.vRoad[roadID].length << endl;
//	cout << "Road ID1:" << g.vNode[g.vRoad[roadID].ID1].x << "," << g.vNode[g.vRoad[roadID].ID1].y << endl;
//	cout << "Road ID2:" << g.vNode[g.vRoad[roadID].ID2].x << "," << g.vNode[g.vRoad[roadID].ID2].y << endl;
	for(i = 0; i < pos; i++)
	{
		d1 += g.vRoad[roadID].length * vLength[i] / dsum;
	}
	d1 += g.vRoad[roadID].length * dp1 / dsum;
//	cout << "d1:" << d1 << endl;
	
	d2 = g.vRoad[roadID].length * dp2 / dsum;
	for(i = pos + 1; i < vLength.size(); i++)
	{
		d2 += g.vRoad[roadID].length * vLength[i] / dsum;
	}
//	cout << "d2:" << d2 << endl;
}

double RoadNetwork::pointToRoadDist(double px, double py, double rx1, double ry1, double rx2, double ry2)
{
	double d = 0;

	double dx = rx2 - rx1;
	double dy = ry2 - ry1;

	double k = -((rx1 - px) * dx + (ry1 - py) * dy) / (dx * dx + dy * dy);
	double footX = k * dx + rx1;
	double footY = k * dy + ry1;

	if(footY >= min(ry1, ry2) && footY <= max(ry1, ry2) && footX >= min(rx1, rx2) && footX <= max(rx1, rx2))
		d = sqrt((footX-px)*(footX-px) + (footY-py)*(footY-py));
	else	//The point is not on the line segment
	{
//		double d1 = sqrt((rx1-px)*(rx1-px) + (ry1-py)*(ry1-py));
//		double d2 = sqrt((rx2-px)*(rx2-px) + (ry2-py)*(ry2-py));
//		d = (d1 < d2 ? d1 : d2);
		d = 99999999;
	}
	
	return d;
}

double RoadNetwork::sameRoadDist(double px1, double py1, double px2, double py2, int roadID)
{
	vector<pair<double, double> >::iterator ivpRD;
	double dsum = 0;
	double dpart = 0;
	double dp = nodeDist(px1, py1, px2, py2);
	for(ivpRD = g.vRoad[roadID].vpRoadDetail.begin(); ivpRD != g.vRoad[roadID].vpRoadDetail.end() - 1; ivpRD++)
	{
		dpart = nodeDist((*ivpRD).first, (*ivpRD).second, (*(ivpRD + 1)).first, (*(ivpRD + 1)).second);
		dsum += dpart;
	}

	return g.vRoad[roadID].length * dp / dsum;
}
	
road_time RoadNetwork::addTime(road_time t, int sec)
{
	t.second += sec;
	if(t.second >= 60)
	{
		t.minute += 1;
		t.second -= 60;
	}

	if(t.minute >= 60)
	{
		t.hour += 1;
		t.minute -= 60;
	}

	if(t.hour >= 24)
		t.hour -= 24;

	return t;
}
