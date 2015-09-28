#include "road.h"

#define INF 99999999

typedef struct heap
{
	pair<double, float> pif;//ID, distance
	bool operator < (const struct heap &a) const  
	{  
		return pif.second > a.pif.second; 
	} 
}h;

int RoadNetwork::buildGraph()
{
	if(conf.readConf())
	{
		return -1;
	}
	
	T = 60 * conf.h + conf.m;
	TN = 24 * 60 / (60 * conf.h + conf.m);
	
	readNodeMap();
	cout << "mNodeMap size:" << mNodeMap.size() << endl;
	readRoad();
	readNode();
//	readTrajectory();
	readSpeed();
	outputSpeed();
//	organizeSpeed();
//	readAvgSpeed();
//	readTotalAvgSpeed();
//	readCost();//
	return 0;
}
	
int	RoadNetwork::readNodeMap()
{
	ifstream inNM(conf.nodeMapFilePath.c_str());
	cout << conf.nodeMapFilePath << endl;
	if(!inNM)
	{
		cout << "Cannot open road file" << endl;
		return -1;
	}

	double d1, d2;
	while(inNM >> d1)
	{
		inNM >> d2;
		mNodeMap[d1] = d2;
	}
	inNM.close();

	return 0;
}
	
int RoadNetwork::readRoad()
{
	ifstream inRoadFile(conf.edgeFilePath.c_str());
	cout << conf.edgeFilePath << endl;
	if(!inRoadFile)
	{
		cout << "Cannot open road file" << endl;
		return -1;
	}

	int i;
	string stmp;
	stringstream ss;

	int direction, length;
	double x, y, id, roadID;
	vector<string> vs, vscoor;
	while(getline(inRoadFile, stmp))
	{
		vs = split(stmp, "\t");
		roadInfo ri;
		ss.clear();
		ss.str("");
		ss << vs[0];
		ss >> roadID;
		ri.roadID = roadID;

		ss.clear();
		ss.str("");
		ss << vs[1];
		ss >> direction;
		ri.direction = direction;
		
		ss.clear();
		ss.str("");
		ss << vs[2];
		ss >> length;
		ri.length = length;
		
		ss.clear();
		ss.str("");
		ss << vs[3];
		ss >> id;
		if(mNodeMap.find(id) != mNodeMap.end())
		{
			id = mNodeMap[id];
		}
		ri.ID1 = id;

		ss.clear();
		ss.str("");
		ss << vs[4];
		ss >> id;
		if(mNodeMap.find(id) != mNodeMap.end())
		{
			id = mNodeMap[id];
		}
		ri.ID2 = id;

		ri.isolated = false;

		for(i = 5; i < vs.size(); i++)
		{
			vscoor = split(vs[i],",");
			ss.clear();
			ss.str("");
			ss << vscoor[0];
			ss >> x;
			ss.clear();
			ss.str("");
			ss << vscoor[1];
			ss >> y;
			ri.vpRoadDetail.push_back(make_pair(x, y));
			vscoor.clear();
		}

		g.mRoad[ri.roadID] = ri;
		vs.clear();
	}
	inRoadFile.close();

	ifstream ifIso(conf.isoRoadFilePath.c_str());
	if(ifIso)
	{
		int		roadIDa;
		while(ifIso >> roadID)
			g.mRoad[roadID].isolated = true;
		ifIso.close();
	}
	else
		cout << "Cannot open isolated road file" << endl;

	return 0;
}

int RoadNetwork::readNode()
{
	ifstream inNodeFile(conf.nodeFilePath.c_str());
	cout << conf.nodeFilePath << endl;
	if(!inNodeFile)
	{
		cout << "Cannot open node file" << endl;
		return -1;
	}

	string stmp;
	stringstream ss;

	int type, mainID, nodeNum, i, j, num, itmp, k, p;
	double dtmp;
	double nodeID;
	double x, y;
	map<double, int>::iterator imSNR1, imSNR2;
	vector<int> vRoad;
	vector<int>::iterator ivRoad1, ivRoad2;
	inNodeFile >> nodeNum;
	cout << nodeNum << endl;
	k = 0;
	p = 0;
	for(i = 0; i < nodeNum; i++)
	{
		inNodeFile >> nodeID;
		if(mNodeMap.find(nodeID) == mNodeMap.end())
		{
			node n;
			n.ID = nodeID;
			mIDTrans[n.ID] = k;	//Fill in the ID map
			mRIDTrans[k] = n.ID;	//Fill in the ID map
			k++;
			inNodeFile >> n.type;
			inNodeFile >> num;
			for(j = 0; j < num; j++)	//mainNeighbor Road
			{
				inNodeFile >> dtmp;
			}
			inNodeFile >> n.MainID;
	
			inNodeFile >> num;
			for(j = 0; j < num; j++)	//subID
			{
				inNodeFile >> dtmp;
				n.vSubID.push_back(dtmp);
			}
			inNodeFile >> n.neighborNode;
			n.isolated = false;
			inNodeFile >> num;
			for(j = 0; j < num; j++)	//subNeighbor Road
			{
				inNodeFile >> dtmp;
				if(g.mRoad[dtmp].direction == 0 || g.mRoad[dtmp].direction == 1)
				{
					if(n.ID == g.mRoad[dtmp].ID1)
					{
						n.mNeighborLength[g.mRoad[dtmp].ID2] = g.mRoad[dtmp].length;
						n.mSubNeighborRoad[g.mRoad[dtmp].ID2] = dtmp;
					}
					else
					{
						n.mNeighborLength[g.mRoad[dtmp].ID1] = g.mRoad[dtmp].length;
						n.mSubNeighborRoad[g.mRoad[dtmp].ID1] = dtmp;
					}
				}
				else if(g.mRoad[dtmp].direction == 2)
				{
					if(n.ID == g.mRoad[dtmp].ID1)
					{
						n.mNeighborLength[g.mRoad[dtmp].ID2] = g.mRoad[dtmp].length;
						n.mSubNeighborRoad[g.mRoad[dtmp].ID2] = dtmp;
					}
				}
				else
				{
					if(n.ID == g.mRoad[dtmp].ID2)
					{
						n.mNeighborLength[g.mRoad[dtmp].ID1] = g.mRoad[dtmp].length;
						n.mSubNeighborRoad[g.mRoad[dtmp].ID1] = dtmp;
					}
				}
			}
			inNodeFile >> n.x;
			inNodeFile >> n.y;
			g.mNode[nodeID] = n;
		}
		else
		{
			p++;
			nodeID = mNodeMap[nodeID];
			for(j = 0; j < 5; j++)
				inNodeFile >> dtmp;
			inNodeFile >> num;
			for(j = 0; j < num; j++)
			{
				inNodeFile >> dtmp;
				if(g.mRoad[dtmp].direction == 0 || g.mRoad[dtmp].direction == 1)
				{
					if(nodeID == g.mRoad[dtmp].ID1)
					{
						g.mNode[nodeID].mNeighborLength[g.mRoad[dtmp].ID2] = g.mRoad[dtmp].length;
						g.mNode[nodeID].mSubNeighborRoad[g.mRoad[dtmp].ID2] = dtmp;
					}
					else
					{
						g.mNode[nodeID].mNeighborLength[g.mRoad[dtmp].ID1] = g.mRoad[dtmp].length;
						g.mNode[nodeID].mSubNeighborRoad[g.mRoad[dtmp].ID1] = dtmp;
					}
				}
				else if(g.mRoad[dtmp].direction == 2)
				{
					if(nodeID == g.mRoad[dtmp].ID1)
					{
						g.mNode[nodeID].mNeighborLength[g.mRoad[dtmp].ID2] = g.mRoad[dtmp].length;
						g.mNode[nodeID].mSubNeighborRoad[g.mRoad[dtmp].ID2] = dtmp;
					}
				}
				else
				{
					if(nodeID == g.mRoad[dtmp].ID2)
					{
						g.mNode[nodeID].mNeighborLength[g.mRoad[dtmp].ID1] = g.mRoad[dtmp].length;
						g.mNode[nodeID].mSubNeighborRoad[g.mRoad[dtmp].ID1] = dtmp;
					}
				}
			}
			inNodeFile >> dtmp;
			inNodeFile >> dtmp;
		}

		//Fill the road sNeighborRoad info
		vRoad.clear();
		for(imSNR1 = g.mNode[nodeID].mSubNeighborRoad.begin(); imSNR1 != g.mNode[nodeID].mSubNeighborRoad.end();imSNR1++)
		{
			vRoad.push_back((*imSNR1).second);
		}

		for(ivRoad1 = vRoad.begin(); ivRoad1 != vRoad.end(); ivRoad1++)
		{
			for(ivRoad2 = ivRoad1 + 1; ivRoad2 != vRoad.end(); ivRoad2++)
			{
				g.mRoad[*ivRoad1].sNeighborRoad.insert(*ivRoad2);
				g.mRoad[*ivRoad2].sNeighborRoad.insert(*ivRoad1);
			}
		}
	}
	inNodeFile.close();

	ifstream ifIso(conf.isoNodeFilePath.c_str());
	if(ifIso)
	{
		int	nodeID;
		while(ifIso >> nodeID)
		{
			if(mNodeMap.find(nodeID) != mNodeMap.end())
			{
				nodeID = mNodeMap[nodeID];
			}
			g.mNode[nodeID].isolated = true;
		}
		ifIso.close();
	}
	else
		cout << "Cannot open isolated node file" << endl;
	
	return 0;
}
	
int	RoadNetwork::readTrajectory()
{
	ifstream inTraj(conf.trajectoryFilePath.c_str());
	cout << conf.trajectoryFilePath << endl;
	if(!inTraj)
	{
		cout << "Cannot open trajectory file" << endl;
		return -1;
	}

	string stmp;
	stringstream ss;
	vector<string> vs, vL, vT, vV;
	int i, V, j;
	double T, L;
	j = 0;
	cout << "Read Trajectory" << endl;
	while(getline(inTraj, stmp))
	{
		if(j % 10000 == 0)
			cout << j << endl;
		vs = split(stmp, ",");
		vL = split(vs[4], "|");
		vT = split(vs[8], "|");
		vV = split(vs[9], "|");
		for(i = 0; i < vL.size(); i++)
		{
			ss.clear();
			ss.str("");
			ss << vL[i];
			ss >> L;
			if(L < 0)
				L = -L;
			
			if(g.mRoad.find(L) == g.mRoad.end())
				continue;

			ss.clear();
			ss.str("");
			ss << vT[i];
			ss >> T;

			ss.clear();
			ss.str("");
			ss << vV[i];
			ss >> V;

			g.mRoad[L].mV[T] = V;
		}
		vL.clear();
		vT.clear();
		vV.clear();
		vs.clear();
		j++;
	}
	inTraj.close();

	ofstream ofile("speed");
	ofile << g.mRoad.size() << endl;

	map<double, roadInfo>::iterator imRoad;
	map<double, double>::iterator imV;
	for(imRoad = g.mRoad.begin(); imRoad != g.mRoad.end(); imRoad++)
	{
		ofile << setprecision(15)<< (*imRoad).first << "\t" << (*imRoad).second.mV.size();
		for(imV = (*imRoad).second.mV.begin(); imV != (*imRoad).second.mV.end(); imV++)
		{
			ofile << setprecision(15) << "\t" << (*imV).first << "\t" << (*imV).second;
		}
		ofile << endl;
	}
	ofile.close();
}

void RoadNetwork::testGraph()
{
	map<double, roadInfo>::iterator imRoad;
	vector<pair<double, double> >::iterator ivRD;
	map<double, int>::iterator imV;
/*	for(imRoad = g.mRoad.begin(); imRoad != g.mRoad.end(); imRoad++)
	{
//		cout << setprecision(15) << (*imRoad).first << "\t" << (*imRoad).second.direction << "\t" << (*imRoad).second.length;
		cout << setprecision(15) << (*imRoad).first;
		cout << setprecision(15) << "\tID1:" << (*imRoad).second.ID1 << "\tID2:" << (*imRoad).second.ID2;
		for(ivRD = (*imRoad).second.vpRoadDetail.begin(); ivRD != (*imRoad).second.vpRoadDetail.end(); ivRD++)
		{
			cout << setprecision(15) << "\t" << (*ivRD).first << "," << (*ivRD).second;
		}
		for(imV = (*imRoad).second.mV.begin(); imV != (*imRoad).second.mV.end(); imV++)
		{
			cout << setprecision(15) << "\t" << (*imV).first << "," << (*imV).second;
		}
		cout << endl;
	}
*/
	map<double, node>::iterator imNode;
	map<double, float>::iterator imNL;
	map<double, int>::iterator imSN;
	for(imNode = g.mNode.begin(); imNode != g.mNode.end(); imNode++)
	{
		cout << setprecision(15) << (*imNode).second.ID << "\tLenght:";
		for(imNL = (*imNode).second.mNeighborLength.begin(); imNL != (*imNode).second.mNeighborLength.end(); imNL++)
		{
			cout << setprecision(15) << "\t" << (*imNL).first << "," << (*imNL).second;
		}
		cout << endl << "subneighbor road" << endl;
		for(imSN = (*imNode).second.mSubNeighborRoad.begin(); imSN != (*imNode).second.mSubNeighborRoad.end(); imSN++)
		{
			cout << setprecision(15) << "\t" << (*imSN).first << "," << (*imSN).second;
		}
		cout << endl;
	}
}
	
int	RoadNetwork::readSpeed()
{
	ifstream inS(conf.speedFilePath.c_str());
	cout << conf.speedFilePath << endl;
	if(!inS)
	{
		cout << "Cannot open speed file" << endl;
		return -1;
	}

	cout << "Reading Raw Speed file" << endl;
	map<int,int> mVP;	//speed number, count
	map<int,int>::iterator imVP;
	int lineNum, sNum, i, j, v, h, m, dt;
	double roadID, t;
	stringstream ss;
	inS >> lineNum;
	const char* format = "%Y%m%d%H%M%S";
	struct tm * ptm;
	int e = 0;
	for(i = 0; i < lineNum; i++)
	{
		inS >> roadID;
		inS >> sNum;
/*		if(sNum > e)	//Count speed number on each road
			e = sNum;
		if(mVP.find(sNum) == mVP.end())
			mVP[sNum] = 1;
		else
			mVP[sNum]++;*/
		for(j = 0; j < sNum; j++)
		{
			inS >> t >> h >> m >> v;
			if(v == 0)
				continue;
			dt = 60*h + m;
		//	inS >> t >> v;
			string ct;
			ss.clear();
			ss.str("");
			ss << t;
			ss >> ct;
//			g.mRoad[roadID].mV[t] = v;
			g.mRoad[roadID].mGraV[dt / T].push_back(v);
		}
	}
	inS.close();
	
	map<int, double> mGVT;
	map<int, double>::iterator imGVT;
	map<double, roadInfo>::iterator	imRoad;	
	map<int, vector<double> >::iterator imGraV;
	vector<double>::iterator iv;
	int sum = 0;
	int count = 0;
	int s;
	double avgV;
	for(imRoad = g.mRoad.begin(); imRoad != g.mRoad.end(); imRoad++)
	{
		if(mGVT.find((*imRoad).second.mGraV.size()) == mGVT.end())
		{
			mGVT[(*imRoad).second.mGraV.size()] = 1;
		}
		else
			mGVT[(*imRoad).second.mGraV.size()]++;
/*		if((*imRoad).second.mGraV.size()==24)
		{
			cout << "RoadID:" << (*imRoad).first << endl;
			for(imGraV = (*imRoad).second.mGraV.begin(); imGraV != (*imRoad).second.mGraV.end(); imGraV++)
			{
				cout << (*imGraV).first << ":" << endl;
				for(iv = (*imGraV).second.begin(); iv != (*imGraV).second.end(); iv++)
					cout << *iv << "\t";
				cout << endl;
			}
		}*/
		for(imGraV = (*imRoad).second.mGraV.begin(); imGraV != (*imRoad).second.mGraV.end(); imGraV++)
		{
			count = 0;
			s = 0;
			for(iv = (*imGraV).second.begin(); iv != (*imGraV).second.end(); iv++)
			{
				if(*iv == 0)
					continue;
				count += *iv; 
				s++;
			}

			if(s == 0)
				continue;
			avgV = count / s;
//			if(avgV == 0)
//				avgV = 0.1;
//			(*imRoad).second.mAvgV[(*imGraV).first] = count / (*imGraV).second.size();
			if(avgV == 0)
				continue;
			(*imRoad).second.mAvgV[(*imGraV).first] = avgV;
		}
	}

	for(imGVT = mGVT.begin(); imGVT != mGVT.end(); imGVT++)
	{	
		sum += (*imGVT).second;
		cout << (*imGVT).first << "\t" << (*imGVT).second << endl;
	}
	cout << "Sum:" << sum << endl;
	cout << "Road num:" << g.mRoad.size() << endl;

	ofstream ofile((conf.city + "AvgSpeed").c_str());
	map<int, double>::iterator imAvgV;	
	for(imRoad = g.mRoad.begin(); imRoad != g.mRoad.end(); imRoad++)
	{
		ofile << setprecision(15) << (*imRoad).first << "\t" << (*imRoad).second.mAvgV.size();
		for(imAvgV = (*imRoad).second.mAvgV.begin(); imAvgV != (*imRoad).second.mAvgV.end(); imAvgV++)
		{
			ofile << "\t" << (*imAvgV).first << "\t" << (*imAvgV).second;
		}
		ofile << endl;
	}
	ofile.close();
	cout << "Speed File Distribution:speed number, road number" << endl;
	for(i = 0; i < e + 1; i++)
	{
		if(mVP.find(i) == mVP.end())
			cout << i << "\t" << 0 << endl;
		else
			cout << i << "\t" << mVP[i] << endl;
	}
}

void RoadNetwork::outputSpeed()
{
	map<double, roadInfo>::iterator		imRoad;
	map<int, vector<double> >::iterator imGraV;
	vector<double>::iterator iv;
	stringstream ss;
	string sTN;
	ss.clear();
	ss.str("");
	ss << TN;
	ss >> sTN;
	ofstream oFile((conf.city+"SpeedDistribution"+sTN).c_str());
	cout << "Writing " << conf.city << "SpeedDistribution" << sTN << endl;
	for(imRoad = g.mRoad.begin(); imRoad != g.mRoad.end(); imRoad++)
	{
		for(imGraV = (*imRoad).second.mGraV.begin(); imGraV != (*imRoad).second.mGraV.end(); imGraV++)
		{
			oFile << setprecision(15) << (*imRoad).first << "\t" << (*imGraV).first << "\t" << (*imGraV).second.size();
			for(iv = (*imGraV).second.begin(); iv != (*imGraV).second.end(); iv++)
			{
				oFile << setprecision(15) << "\t" << (*iv);
			}
			oFile << endl;
		}
	}
	oFile.close();
}


void RoadNetwork::organizeSpeed()
{
	map<int, vector<int> > mTNumRoad;	//reverted list,interval number,road set
	readAvgSpeed(mTNumRoad);
	fillVoidSpeedST(mTNumRoad);
}

	
int RoadNetwork::readAvgSpeed(map<int, vector<int> > &mTNumRoad)
{
	ifstream inS(conf.avgSpeedFilePath.c_str());
	cout << conf.avgSpeedFilePath << endl;
	if(!inS)
	{
		cout << "Cannot open average speed file" << endl;
		return -1;
	}
	cout << "Reading Average Speed file" << endl;
	T = 60 * conf.h + conf.m;
	TN = 24 * 60 / (60 * conf.h + conf.m);

	int i, roadID, num, t;
	double v;
	for(i = 0; i < g.mRoad.size(); i++)
	{
		inS >> roadID;
		inS >> num;
		mTNumRoad[num].push_back(roadID);
		while(num != 0)
		{
			num--;
			inS >> t >> v;
			g.mRoad[roadID].mAvgV[t] = v;
		}
	}
	
	inS.close();
	
/*	map<double, roadInfo>::iterator	imRoad;	
	map<int, int>::iterator imAvgV;	
	for(imRoad = g.mRoad.begin(); imRoad != g.mRoad.end(); imRoad++)
	{
		cout << setprecision(15) << (*imRoad).first << "\t" << (*imRoad).second.mAvgV.size();
		for(imAvgV = (*imRoad).second.mAvgV.begin(); imAvgV != (*imRoad).second.mAvgV.end(); imAvgV++)
		{
			cout << "\t" << (*imAvgV).first << "\t" << (*imAvgV).second;
		}
		cout << endl;
	}*/

	return 0;
}
	
void RoadNetwork::fillVoidSpeedST(map<int, vector<int> > &mTNumRoad)
{
	map<double, bool>		mVisited;
	map<int, vector<int> >::iterator imTNR;
	vector<int>::iterator	ivR;
	set<int>::iterator		isSNR, isSNR2, isSNR3;
	map<int, double>::iterator	imAvgV;
	int i, j, in, k;
	int vtmp;
	queue<int>				qRoad;
	bool insertQ;
	cout << "Filling void avg speed" << endl;
	map<double, roadInfo>::iterator imRoad;
	for(in = TN; in > -1; in--)
	{
		for(ivR = mTNumRoad[in].begin(); ivR != mTNumRoad[in].end(); ivR++)
		{
			qRoad.push(*ivR);
		}
	}

	map<int, vector<int> >	mvv;
	map<int, vector<int> >::iterator imvv;
	int roadID;

//	while(!qRoad.empty() && mVisited.size() != g.mRoad.size())
//	while(!qRoad.empty() && mVisited.size() != 294100)
	while(!qRoad.empty() && mVisited.size() != 294120)
	{
		roadID = qRoad.front();
		qRoad.pop();
		insertQ = false;
		if(g.mRoad[roadID].mAvgV.size() == TN)
		{
			mVisited[roadID] = true;
			cout << mVisited.size() << endl;
		}
		else	//Fill itself's void
		{
			for(k = 0; k < TN; k++)
			{
				if(g.mRoad[roadID].mAvgV.find(k) == g.mRoad[roadID].mAvgV.end())
				{
					vtmp = 0;
					j = 0;
					if(g.mRoad[roadID].mAvgV.find((k+1) % TN) != g.mRoad[roadID].mAvgV.end())//next hour
					{
						vtmp += g.mRoad[roadID].mAvgV[(k+1) % TN];
						j++;
					}
					if(g.mRoad[roadID].mAvgV.find((k-1+TN) % TN) != g.mRoad[roadID].mAvgV.end())//previous hour
					{
						vtmp += g.mRoad[roadID].mAvgV[(k-1+TN) % TN];
						j++;
					}
					for(isSNR = g.mRoad[roadID].sNeighborRoad.begin(); isSNR != g.mRoad[roadID].sNeighborRoad.end(); isSNR++)//neighbor roads
					{
						if(g.mRoad[*isSNR].mAvgV.find(k) != g.mRoad[*isSNR].mAvgV.end())
						{
							vtmp += g.mRoad[*isSNR].mAvgV[k];
							j++;
						}
					}
					if(j == 0)
					{
						insertQ = true;			
					}
					else
					{
						g.mRoad[roadID].mAvgV[k] = (int)(vtmp / j);
					}
				}
			}
			if(insertQ)
				qRoad.push(roadID);
		}

	//	if(g.mRoad[roadID].mAvgV.size() > TN * 0.75)
	//	{
			for(isSNR = g.mRoad[roadID].sNeighborRoad.begin(); isSNR != g.mRoad[roadID].sNeighborRoad.end(); isSNR++)	//The neighbor roads of the visiting road
			 {
	//			insertQ = false;
				if(g.mRoad[*isSNR].mAvgV.size() != TN)
				{
					for(i = 0; i < TN; i++)	//traverse the intervals
					{
						if(g.mRoad[*isSNR].mAvgV.find(i) == g.mRoad[*isSNR].mAvgV.end())//empty
						{
							vtmp = 0;
							j = 0;
							if(g.mRoad[*isSNR].mAvgV.find((i+1) % TN) != g.mRoad[*isSNR].mAvgV.end())//next hour
							{
								vtmp += g.mRoad[*isSNR].mAvgV[(i+1) % TN];
								j++;
							}
							if(g.mRoad[*isSNR].mAvgV.find((i-1+TN) % TN) != g.mRoad[*isSNR].mAvgV.end())//previous hour
							{
								vtmp += g.mRoad[*isSNR].mAvgV[(i-1+TN) % TN];
								j++;
							}
							for(isSNR2 = g.mRoad[*isSNR].sNeighborRoad.begin(); isSNR2 != g.mRoad[*isSNR].sNeighborRoad.end(); isSNR2++)//neighbor roads
							{
								if(g.mRoad[*isSNR2].mAvgV.find(i) != g.mRoad[*isSNR2].mAvgV.end())
								{
//								cout << setprecision(15) << *isSNR2 << "\t" << i << endl;
									vtmp += g.mRoad[*isSNR2].mAvgV[i];
									j++;
								}
							}
							if(j == 0)
							{
								insertQ = true;
							}
							if(j != 0)
							{
								g.mRoad[*isSNR].mAvgV[i] = (vtmp / j);
							}
						}
					}
				}
	//			if(insertQ)
	//				qRoad.push(*isSNR);
			}
	//	}
	}

	cout << "Writing Total Avg speed" << endl;
	stringstream ss;
	ss.clear();
	ss.str("");
	ss << TN;
	string sTN;
	ss >> sTN;
	string filename = conf.city + "AvgSpeed" + sTN;
	ofstream ofIR((conf.city + "IsoRoad").c_str());
	ofstream ofIN((conf.city + "IsoNode").c_str());
	ofstream ofCOST((conf.city + "cost" + sTN).c_str());
	ofstream ofile(filename.c_str());
	set<double> sout;
	set<double>::iterator isout;
	for(imRoad = g.mRoad.begin(); imRoad != g.mRoad.end(); imRoad++)
	{
		ofile << setprecision(15) << (*imRoad).first << "\t" << (*imRoad).second.mAvgV.size();
		ofCOST << setprecision(15) << (*imRoad).first << "\t" << (*imRoad).second.mAvgV.size();
		for(imAvgV = (*imRoad).second.mAvgV.begin(); imAvgV != (*imRoad).second.mAvgV.end(); imAvgV++)
		{
//			if((*imAvgV).second == 0)
//				(*imAvgV).second = 0.1;
//			(*imRoad).second.mCost[(*imAvgV).first] = (*imRoad).second.length / ((*imAvgV).second * 3.6);
			ofile << setprecision(15) << "\t" << (*imAvgV).first << "\t" << (*imAvgV).second;
			ofCOST << setprecision(15) << "\t" << (*imAvgV).first << "\t" << (((*imRoad).second.length) / ((*imAvgV).second / 3.6)) / 60;	//Minutes
//			ofCOST << setprecision(15) << "\t" << (*imAvgV).first << "\t" << (*imRoad).second.length / ((*imAvgV).second / 3.6) << "\t" << (*imRoad).second.length << "\t" << (*imAvgV).second;
		}
		ofile << endl;
		ofCOST << endl;
		if((*imRoad).second.mAvgV.size() == 0)
		{
			sout.insert((*imRoad).second.ID1);
			sout.insert((*imRoad).second.ID2);
			ofIR << setprecision(15) << (*imRoad).first << endl;
		}
	}

	for(isout = sout.begin(); isout != sout.end(); isout++)
	{
		ofIN << setprecision(15) << *isout << endl;
	}
	ofile.close();
	ofIR.close();
	ofIN.close();
	ofCOST.close();
}	
	
int RoadNetwork::readCost()
{
	stringstream ss;
	ss.clear();
	ss.str("");
	ss << TN;
	string sTN;
	ss >> sTN;
	ifstream ifCOST((conf.dataPath + conf.city + "cost" + sTN).c_str());
	if(!ifCOST)
	{
		cout << "Cannot open " + conf.city + "cost" + sTN << endl;
		return -1;
	}

	map<int, double>::iterator	imAvgV;
	map<double, roadInfo>::iterator imRoad;
	map<int, double>::iterator imCost;
	int i, j, roadID, sNum, h;
	double t;
	cout << "Reading cost file" << endl;
	cout  << "ROAD SIZE" << g.mRoad.size() << endl;
	for(i = 0; i < g.mRoad.size(); i++)
	{
		ifCOST >> roadID;
		ifCOST >> sNum;
		for(j = 0; j < sNum; j++)
		{
			ifCOST >> h >> t;
			g.mRoad[roadID].mCost[h] = t;
		}
	}
	ifCOST.close();

/*	for(imRoad = g.mRoad.begin(); imRoad != g.mRoad.end(); imRoad++)
	{
		cout << setprecision(15) << (*imRoad).first << "\t" << (*imRoad).second.mCost.size();
		for(imCost = (*imRoad).second.mCost.begin(); imCost != (*imRoad).second.mCost.end(); imCost++)
		{
			cout << "\t" << (*imCost).first << "\t" << (*imCost).second;
		}
		cout << endl;
	}*/
}

int RoadNetwork::readTotalAvgSpeed()
{
	stringstream ss;
	ss.clear();
	ss.str("");
	ss << TN;
	string sTN;
	ss >> sTN;
	string filename = conf.dataPath + conf.city + "AvgSpeed" + sTN;
	ifstream ifile(filename.c_str());

	if(!ifile)
	{
		cout << "Cannot open total average speed file " << filename << endl;
		return -1;
	}
	
	int i, j, num, roadID, t;
	double v;
	cout << "Reading Avg Speed File" << endl;
	for(i = 0; i != g.mRoad.size(); i++)
	{
		ifile >> roadID;
		ifile >> num;
		for(j = 0; j < num; j++)
		{
			ifile >> t;
			ifile >> v;
			g.mRoad[roadID].mAvgV[t] = v;
		}
	}
	ifile.close();
}

double RoadNetwork::nodeDist(double x1, double y1, double x2, double y2)
{
	return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}
	
float RoadNetwork::distanceDijkstraA(double ID1, double ID2, vector<int>& vRoadList)
{
	double nID1 = mIDTrans[ID1];
	double nID2 = mIDTrans[ID2];
	vector<float> vDistance(g.mNode.size(), INF);
	vector<float>::iterator ivD;
	priority_queue<h> qh;
	vector<int> vPrevious(g.mNode.size(), -1);
	vector<int>::iterator ivP;
	map<double, float>::iterator imNL;

	vDistance[nID1] = 0;
	for(imNL = g.mNode[ID1].mNeighborLength.begin(); imNL != g.mNode[ID1].mNeighborLength.end(); imNL++)
	{
		vDistance[mIDTrans[(*imNL).first]] = (*imNL).second;
		h hh;
		hh.pif = make_pair(mIDTrans[(*imNL).first], (*imNL).second + coorDistance((*imNL).first, ID2));
		qh.push(hh);
		vPrevious[mIDTrans[(*imNL).first]] = mIDTrans[ID1];
	}

	pair<double, float> pu;
	int i = 0;
	while(!qh.empty())
	{
		i++;
		pu = qh.top().pif;
		if(pu.first == nID2)
			break;
		qh.pop();
		for(imNL = g.mNode[mRIDTrans[pu.first]].mNeighborLength.begin(); imNL != g.mNode[mRIDTrans[pu.first]].mNeighborLength.end(); imNL++)
		{
			if(vDistance[mIDTrans[(*imNL).first]] == INF && (*imNL).first != ID1)
			{
				float d = vDistance[pu.first] + (*imNL).second;
				vDistance[mIDTrans[(*imNL).first]] = d;
				h hh;
				hh.pif = make_pair(mIDTrans[(*imNL).first], d + coorDistance((*imNL).first, ID2));
				qh.push(hh);
				vPrevious[mIDTrans[(*imNL).first]] = pu.first;
			}
			else if(vDistance[mIDTrans[(*imNL).first]] > vDistance[pu.first] + (*imNL).second)
			{
				float d = vDistance[pu.first] + (*imNL).second;
				vDistance[mIDTrans[(*imNL).first]] = d;
				vPrevious[mIDTrans[(*imNL).first]] = pu.first;
				h hh;
				hh.pif = make_pair(mIDTrans[(*imNL).first], d + coorDistance((*imNL).first, ID2));
				qh.push(hh);
			}
		}
	}
	

	double id = nID2;
	double idtmp;
	vector<int> vRoadListtmp;
	vector<int>::reverse_iterator irvRL;
	cout << "Distance:" << vDistance[nID2] << endl;
	if(vDistance[nID2] != INF)
	{	
		while(id != nID1)
		{
			idtmp = vPrevious[id];
			vRoadListtmp.push_back(g.mNode[mRIDTrans[idtmp]].mSubNeighborRoad[mRIDTrans[id]]);
			id = idtmp;
		}
	}

	for(irvRL = vRoadListtmp.rbegin(); irvRL != vRoadListtmp.rend(); irvRL++)
	{
		vRoadList.push_back(*irvRL);
	}

	return vDistance[nID2];
}
	
double RoadNetwork::shortestTimeDij(double ID1, double ID2, int t1, vector<int>& vRoadList, vector<double>& vRTime, vector<double>& vRTakeTime, double &d)
{
	double nID1 = mIDTrans[ID1];
	double nID2 = mIDTrans[ID2];
	vector<float> vTime(g.mNode.size(), INF);
	vector<float>::iterator ivD;
	vector<double> vT(g.mNode.size(), t1);	//each node's current time
	priority_queue<h> qh;
	vector<int> vPrevious(g.mNode.size(), -1);
	map<double, int>::iterator imNR;
	map<double, roadInfo>::iterator imRoad;

	map<int, double>::iterator imCost;
	double dtmp, ttmp;
	vTime[nID1] = 0;
	int tt = (int)(t1 / T);
	for(imNR = g.mNode[ID1].mSubNeighborRoad.begin(); imNR != g.mNode[ID1].mSubNeighborRoad.end(); imNR++)
	{
/*			cout << "init1:" << g.mRoad[(*imNR).second].mCost[tt] << "\t" << tt << "\t" << (*imNR).second << endl;	//used time
			cout << "cost size:" << g.mRoad[(*imNR).second].mCost.size() <<  endl;
			for(imCost = g.mRoad[(*imNR).second].mCost.begin(); imCost != g.mRoad[(*imNR).second].mCost.end(); imCost++)
				cout << (*imCost).first << "\t" << (*imCost).second << "\t";
			cout << endl;
	for(imRoad = g.mRoad.begin(); imRoad != g.mRoad.end(); imRoad++)
	{
		cout << setprecision(15) << (*imRoad).first << "\t" << (*imRoad).second.mCost.size();
		for(imCost = (*imRoad).second.mCost.begin(); imCost != (*imRoad).second.mCost.end(); imCost++)
		{
			cout << "\t" << (*imCost).first << "\t" << (*imCost).second;
		}
		cout << endl;
	}*/
		if((int)((t1 + g.mRoad[(*imNR).second].mCost[tt]) / T) == tt)	//does not exceed to next interval
		{
			vTime[mIDTrans[(*imNR).first]] = g.mRoad[(*imNR).second].mCost[tt];	//used time
			vT[mIDTrans[(*imNR).first]] += g.mRoad[(*imNR).second].mCost[tt];	//current
			h hh;
			hh.pif = make_pair(mIDTrans[(*imNR).first], g.mRoad[(*imNR).second].mCost[tt]);
			qh.push(hh);
			vPrevious[mIDTrans[(*imNR).first]] = mIDTrans[ID1];
		}
		else
		{
			dtmp = g.mRoad[(*imNR).second].length - g.mRoad[(*imNR).second].mAvgV[tt] / 0.06 * (((tt + 1) * T - t1));
			ttmp = dtmp / (g.mRoad[(*imNR).second].mAvgV[(tt + 1) % TN] / 0.06) + (tt + 1) * T;
			if(ttmp < 0)
			{
//				cout << "init" << endl;
			}
			vTime[mIDTrans[(*imNR).first]] = ttmp - t1;
			h hh;
			hh.pif = make_pair(mIDTrans[(*imNR).first], ttmp - t1);
			qh.push(hh);
			vPrevious[mIDTrans[(*imNR).first]] = mIDTrans[ID1];
			if(ttmp > 24 * 60)
				ttmp -= 24 * 60;
			vT[nID1] = ttmp;
		}
	}

	pair<double, float> pu;
	double ts, te;
	while(!qh.empty())
	{
		pu = qh.top().pif;
		qh.pop();
		
		if(pu.first == nID2)
			break;

		for(imNR = g.mNode[mRIDTrans[pu.first]].mSubNeighborRoad.begin(); imNR != g.mNode[mRIDTrans[pu.first]].mSubNeighborRoad.end(); imNR++)
		{
			if((int)((vT[pu.first] + g.mRoad[(*imNR).second].mCost[(int)(vT[pu.first] / T)]) / T) == (int)(vT[pu.first] / T))
			{
				ttmp = pu.second + g.mRoad[(*imNR).second].mCost[(int)(vT[pu.first] / T)];
//				cout << "ttmp original1:" << ttmp << "\t" << (int)(vT[pu.first]/T ) << "\t" << g.mRoad[(*imNR).second].mCost[(int)(vT[pu.first] / T)] << "\t" << (*imNR).second <<endl;
//				cout << "pu second:" << pu.second << "\t" << g.mRoad[(*imNR).second].mCost[(int)(vT[pu.first] / T)] << endl;
//				cout << "!!ttmp:" << ttmp << endl;
			}
			else
			{
				dtmp = g.mRoad[(*imNR).second].length -  g.mRoad[(*imNR).second].mAvgV[(int)(vT[pu.first] / T)] / 3.6 * (((((int)(vT[pu.first] / T) + 1) * T - vT[pu.first])) * T);
//				cout << "dtmp:" << dtmp << endl;
				ttmp = dtmp / (g.mRoad[(*imNR).second].mAvgV[((int)(vT[pu.first] / T) + 1) % TN] / 3.6) / 60 + (((int)(vT[pu.first] / T) + 1) * T  - t1);
//				cout << "ttmp original2:" << ttmp << endl;
				if(ttmp < 0)
				{
					cout << "length:" <<  g.mRoad[(*imNR).second].length << endl;
					cout << "v:" <<  g.mRoad[(*imNR).second].mAvgV[(int)(vT[pu.first] / T)] / 3.6 << endl;
					cout << "t:" << (((((int)(vT[pu.first] / T) + 1) * T - vT[pu.first])) * 60) << endl;
				}		
			}

//			cout << endl << "ttmp:" << ttmp << endl;
			if(vTime[mIDTrans[(*imNR).first]] == INF && (*imNR).first != ID1)
			{
				vTime[mIDTrans[(*imNR).first]] =  ttmp;
				h hh;
				hh.pif = make_pair(mIDTrans[(*imNR).first], ttmp);
//				cout << "ttmp1:" << ttmp << endl;
				qh.push(hh);
				vPrevious[mIDTrans[(*imNR).first]] = pu.first;
				if(t1 + ttmp > 24*60)
					vT[mIDTrans[(*imNR).first]] = t1 + ttmp - 24*60;
				else
					vT[mIDTrans[(*imNR).first]] = t1 + ttmp;
			}
//			else if(vTime[mIDTrans[(*imNR).first]] > vTime[pu.first] + ttmp)
			else if(vTime[mIDTrans[(*imNR).first]] > ttmp)
			{
//				vTime[mIDTrans[(*imNR).first]] = vTime[pu.first] + ttmp;
				vTime[mIDTrans[(*imNR).first]] = ttmp;
//				cout << "ttmp2:" << ttmp << endl;
				if(t1 + ttmp > 24*60)
					vT[mIDTrans[(*imNR).first]] = t1 + ttmp - 24*60;
				else
					vT[mIDTrans[(*imNR).first]] = t1 + ttmp;
				vPrevious[mIDTrans[(*imNR).first]] = pu.first;
			}
		}
	}
	
	double id = nID2;
	double idtmp;
	vector<int> vRoadListtmp;
	vector<int>::reverse_iterator irvRL;
	vector<double> vRTimetmp;
	vector<double> vRTakeTimetmp;
	vector<double>::reverse_iterator irvRT;
	vector<double>::reverse_iterator irvRTT;
	vT[nID1] = t1;
	cout << "nID2:" << nID2 << endl;
	cout << "Time:" << vTime[nID2] << endl;
//a	if(vTime[nID2] < 0 )
//		return -1;
	double dist = 0;
	if(vTime[nID2] != INF)
	{	
		while(id != nID1)
		{
			idtmp = vPrevious[id];
			dist += g.mRoad[g.mNode[mRIDTrans[idtmp]].mSubNeighborRoad[mRIDTrans[id]]].length;
			vRoadListtmp.push_back(g.mNode[mRIDTrans[idtmp]].mSubNeighborRoad[mRIDTrans[id]]);
			vRTimetmp.push_back(vT[idtmp]);
			vRTakeTimetmp.push_back(g.mRoad[g.mNode[mRIDTrans[idtmp]].mSubNeighborRoad[mRIDTrans[id]]].mCost[(int)(vT[idtmp] / T)]);
			id = idtmp;
		}
	}
	cout << "Distance:" << dist << endl;
	d = dist;

	for(irvRL = vRoadListtmp.rbegin(); irvRL != vRoadListtmp.rend(); irvRL++)
	{
		vRoadList.push_back(*irvRL);
	}

	for(irvRT = vRTimetmp.rbegin(); irvRT != vRTimetmp.rend(); irvRT++)
	{
		vRTime.push_back(*irvRT);
	}
	
	for(irvRTT = vRTakeTimetmp.rbegin(); irvRTT != vRTakeTimetmp.rend(); irvRTT++)
	{
		vRTakeTime.push_back(*irvRTT);
	}

	return vTime[nID2];
}
	
void RoadNetwork::IntSingleFastestPaths(double ID1, double ID2, vector<int>& vRoadList, int t1, int t2, vector<int>& vt)
{
	double nID1 = mIDTrans[ID1];
	double nID2 = mIDTrans[ID2];
	vector<float> vTime(g.mNode.size(), INF);
	vector<float>::iterator ivD;
	priority_queue<h> qh;		//min travel time
	vector<int> vPrevious(g.mNode.size(), -1);
	vector<int>::iterator ivP;
	map<double, float>::iterator imNL;

/*	map<int, >
	int i1 = t1 % T;
	int i2 = t2 % T;
	if(i1 == i2)
	{

	}

	vDistance[nID1] = 0;
	for(imNL = g.mNode[ID1].mNeighborLength.begin(); imNL != g.mNode[ID1].mNeighborLength.end(); imNL++)
	{
		vDistance[mIDTrans[(*imNL).first]] = (*imNL).second;
		h hh;
		hh.pif = make_pair(mIDTrans[(*imNL).first], (*imNL).second);
		qh.push(hh);
	}
*/
}
	
vector<string> RoadNetwork::split(const string &s, const string &seperator)
{
	vector<string> result;
	typedef string::size_type string_size;
	string_size i = 0;
		    
	while(i != s.size())
	{
		int flag = 0;
	    while(i != s.size() && flag == 0)
		{
			flag = 1;
		    for(string_size x = 0; x < seperator.size(); ++x)
			{
				if(s[i] == seperator[x])
				{
					++i;
					flag = 0;
					break;
				}
			}
	    }
					
		flag = 0;
		string_size j = i;
		while(j != s.size() && flag == 0)
		{
			for(string_size x = 0; x < seperator.size(); ++x)
			{
				if(s[j] == seperator[x])
				{
					flag = 1;
					break;
				}
			}
			if(flag == 0) 
				++j;
		}
		 
		if(i != j)
		{
			result.push_back(s.substr(i, j-i));
			i  = j;
		}
	}

	return result;
}

void RoadNetwork::testDijA()
{
	vector<pair<double, double> > vpNode;
//	vpNode = generateNodePair(conf.testNum);
	ifstream ifile("testPair");
	double a,b, t;
	for(int i = 0; i < 10; i++)
	{
		ifile >> a >> b >> t;
		vpNode.push_back(make_pair(a,b));
	}
	ifile.close();

	vector<pair<double, double> >::iterator ivpNode;
	vector<int> vRoadList;
	vector<int>::iterator ivRL;
	int i = 1;
	clock_t start,stop;
    start = clock();
	ofstream ofile((conf.city + "DijResult").c_str());
	vector<double> vD;
	vector<double>::iterator ivD;
	for(ivpNode = vpNode.begin(); ivpNode != vpNode.end(); ivpNode++, i++)
	{
		vRoadList.clear();
		cout << endl << "Test " << i << endl;
		cout << setprecision(15) << (*ivpNode).first << "\t" << (*ivpNode).second << endl;
		cout << setprecision(15)  << "coordinate:" << endl << g.mNode[(*ivpNode).first].x << "\t" << g.mNode[(*ivpNode).first].y << endl << g.mNode[(*ivpNode).second].x << "\t" << g.mNode[(*ivpNode).second].y << endl;
		float d = distanceDijkstraA((*ivpNode).first, (*ivpNode).second, vRoadList);
		vD.push_back(d);
		cout << "distance:" << d << endl;
		for(ivRL = vRoadList.begin(); ivRL != vRoadList.end(); ivRL++)
		{
			cout << *ivRL << "\t" << g.mRoad[*ivRL].length << "\t" << g.mNode[g.mRoad[*ivRL].ID1].x << "\t" << g.mNode[g.mRoad[*ivRL].ID1].y << "\t" << g.mNode[g.mRoad[*ivRL].ID2].x << "\t" << g.mNode[g.mRoad[*ivRL].ID2].y << endl; 
		}
	}
	stop = clock();
//	cout << setprecision(5) <<  "Time per query:" << ((float)(stop - start) / vpNode.size()) / CLOCKS_PER_SEC << endl;

	i = 1;
	ofile << setprecision(5) <<  "Time per query:" << ((float)(stop - start) / vpNode.size()) / CLOCKS_PER_SEC << endl;
	for(ivpNode = vpNode.begin(), ivD = vD.begin(); ivpNode != vpNode.end(); ivpNode++, i++, ivD++)
	{
		ofile << endl << "Test " << i << endl;
		ofile << setprecision(15) << "From nodeID:" << (*ivpNode).first << "\tto nodeID:" << (*ivpNode).second << endl;
		ofile << setprecision(15)  << "coordinate:" << endl << g.mNode[(*ivpNode).first].x << "\t" << g.mNode[(*ivpNode).first].y << endl << g.mNode[(*ivpNode).second].x << "\t" << g.mNode[(*ivpNode).second].y << endl;
		ofile << "Distance:" << *ivD << endl;
	}
	ofile.close();
}

void RoadNetwork::testTimeDij()
{
	vector<pair<double, double> > vpNode;
//	vpNode = generateNodePair(conf.testNum);
	vector<pair<double, double> >::iterator ivpNode;
	vector<int> vRoadList;
	vector<int>::iterator ivRL;
	vector<int> vTime;
	vector<int>::iterator ivTime;
	srand((unsigned)time(0));
	int i;
/*	for(i = 0; i < conf.testNum; i++)
	{
		vTime.push_back(rand() % (24 * 60));
	}*/

	double a,b, t;
	ifstream ifile("testPair");
	for(i = 0; i < 10; i++)
	{
		ifile >> a >> b >> t;
		vpNode.push_back(make_pair(a,b));
		vTime.push_back(t);
	}
	ifile.close();
	i = 1;
	vector<double> vRTime, vRTakeTime;
	vector<double>::iterator ivRT, ivRTT;
	clock_t start,stop;
    start = clock();
	ofstream ofile((conf.city + "DijTimeResult").c_str());
	vector<double> vD;
	vector<double> vT;
	vector<double>::iterator ivD, ivT;
	double d;
	for(ivpNode = vpNode.begin(), ivTime = vTime.begin(); ivpNode != vpNode.end(); ivpNode++, i++, ivTime++)
	{
		vRoadList.clear();
		float dtmp = distanceDijkstraA((*ivpNode).first, (*ivpNode).second, vRoadList);
		if(dtmp > 1000000)
		{
			vT.push_back(0);
			vD.push_back(dtmp);
			cout << "d > 1000000" << endl;
			continue;
		}
		vRoadList.clear();
		vRTime.clear();
		vRTakeTime.clear();
		t = shortestTimeDij((*ivpNode).first, (*ivpNode).second, *ivTime, vRoadList, vRTime, vRTakeTime, d);
		if (t <= 0)
		{
			cout << "t < 0" << endl;
			vT.push_back(0);
			vD.push_back(dtmp);
			continue;
		}
		cout << endl << "Test " << i << endl;
		cout << setprecision(15) << (*ivpNode).first << "\t" << (*ivpNode).second << endl;
		cout << setprecision(15)  << "coordinate:" << endl << g.mNode[(*ivpNode).first].x << "\t" << g.mNode[(*ivpNode).first].y << endl << g.mNode[(*ivpNode).second].x << "\t" << g.mNode[(*ivpNode).second].y << endl;
		cout << "input time:" << (int)((*ivTime) / 60) << "h " << (*ivTime) % 60 << "min" << endl;

		cout << "Time:" << (int)(t/60) << "hour " << (int)((int)(t)%60) << "minutes" << endl;
		cout << "Distance:" << d << endl;
		vT.push_back(t);
		vD.push_back(d);
		for(ivRL = vRoadList.begin(), ivRT = vRTime.begin(), ivRTT = vRTakeTime.begin(); ivRL != vRoadList.end(); ivRL++, ivRT++, ivRTT++)
		{
			cout << *ivRL << "\t" << g.mRoad[*ivRL].length << "\tstart time:" << (int)((*ivRT) / 60) <<"h " << (int)((int)(*ivRT) % 60) << "min "<< endl;
			cout << setprecision(7)<< "\ttake time:" << *ivRTT << "min\t" << g.mNode[g.mRoad[*ivRL].ID1].x << "\t" << g.mNode[g.mRoad[*ivRL].ID1].y << "\t" << g.mNode[g.mRoad[*ivRL].ID2].x << "\t" << g.mNode[g.mRoad[*ivRL].ID2].y << endl; 
		}
	}
    stop = clock();

	i = 1;
//	ofile << setprecision(5) <<  "Time per query:" << ((float)(stop - start) / vpNode.size()) / CLOCKS_PER_SEC << endl;
	for(ivpNode = vpNode.begin(), ivD = vD.begin(), ivT = vT.begin(), ivTime = vTime.begin(); ivpNode != vpNode.end(); ivpNode++, i++, ivD++, ivT++, ivTime++)
	{
		ofile << endl << "Test " << i << endl;
		ofile << setprecision(15) << "From nodeID:" << (*ivpNode).first << "\tto nodeID:" << (*ivpNode).second << endl;
		ofile << setprecision(15)  << "coordinate:" << endl << g.mNode[(*ivpNode).first].x << "\t" << g.mNode[(*ivpNode).first].y << endl << g.mNode[(*ivpNode).second].x << "\t" << g.mNode[(*ivpNode).second].y << endl;
		ofile << "Start time:" << (int)((*ivTime) / 60) << "h " << (*ivTime) % 60 << "min" << endl;
		ofile << setprecision(7) << "Take Time:" << (int)(*ivT/60) << "hour " << (int)((int)(*ivT)%60) << "minutes" << endl;
		ofile << "Distance:" << *ivD << endl;
	}
	ofile.close();
}
	
vector<pair<double, double> > RoadNetwork::generateNodePair(int N)
{
	vector<pair<double, double> > vpNode;
	int i;
	int id1, id2;
	bool b = true;
	srand((unsigned)time(0));
	cout << "node size:" << g.mNode.size() << endl;
	for(i = 0; i < N; i++)
	{
		b = true;
		while(b)
		{
			id1 = rand() % g.mNode.size();
			id2 = rand() % g.mNode.size();
			if(mRIDTrans[id1] == 0 || mRIDTrans[id2] == 0)
				continue;
			if(!g.mNode[mNodeMap[mRIDTrans[id1]]].isolated && !g.mNode[mNodeMap[mRIDTrans[id2]]].isolated)
			{
				b = false;
			}
		}
		vpNode.push_back(make_pair(mRIDTrans[id1], mRIDTrans[id2]));
	}

	return vpNode;
}

double RoadNetwork::rad(double d)
{
	return d * PI / 180.0;
}

double RoadNetwork::coorDistance(double ID1, double ID2)
{
	double lat1	= g.mNode[ID1].x;
	double lng1	= g.mNode[ID1].y;
	double lat2	= g.mNode[ID2].x;
	double lng2	= g.mNode[ID2].y;
	double radLat1 = rad(lat1);
	double radLat2 = rad(lat2);
	double radLng1 = rad(lng1);
	double radLng2 = rad(lng2);
	double s = acos(sin(radLat1)*sin(radLat2)+cos(radLat1)*cos(radLat2)*cos(radLng1-radLng2));
	s = s * EARTH_RADIUS;
	return s * 1000;
}
