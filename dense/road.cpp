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

typedef struct travelTimeInfoHeap
{
	pair<double, travelTimeInfo> pif;
	bool operator < (const struct travelTimeInfoHeap &a) const  
	{  
		return pif.second.min > a.pif.second.min; 
	} 
}tth;

int RoadNetwork::buildGraph()
{
	if(conf.readConf())
	{
		return -1;
	}
	
	T = 60 * 60 * conf.h + 60 * conf.m;
	TN = 24 * 60 * 60 / (60 * 60 * conf.h + 60 * conf.m);
	
	readNodeMap();
	cout << "mNodeMap size:" << mNodeMap.size() << endl;
	readRoad();
	readNode();
//	extractTrajectoryTime();
//	readTrajectory();
//	readSpeed();	//To write the speed file, uncomment the code
//	outputSpeed();
//	speedRawClassify();
//	organizeSpeed();
//	readTotalAvgSpeed();
//	readCost();//
//	driverAttach();
//	driverAvg();
//	readDriver();
//	extractBriefTrajectory();
//	testDriver();
//	readRC();	
	testISFP();
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
	map<string, double>	mDD;	//map the original car ID to driverID
	double dID;
	int k = 0;
	while(getline(inTraj, stmp))
	{
		if(j % 10000 == 0)
			cout << j << endl;
		vs = split(stmp, ",");
		string oid = vs[2];
		if(mDD.find(oid) == mDD.end())
		{
			mDD[oid] = k;
			dID = k;
			driver d;
			d.driverID = dID;
			vDriver.push_back(d);
			k++;
		}
		else
		{
			dID = mDD[oid];
		}
		
		vL = split(vs[4], "|");
		vT = split(vs[8], "|");
		vV = split(vs[9], "|");
		for(i = 1; i < vL.size(); i++)
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

			if(V <= 0)
				continue;

			g.mRoad[L].mV[T].push_back(V);
			vDriver[dID].vRoad.push_back(L);
			vDriver[dID].vTime.push_back(T);
			vDriver[dID].vSpeed.push_back(V);
		}
		vL.clear();
		vT.clear();
		vV.clear();
		vs.clear();
		j++;
	}
	inTraj.close();

	cout << "j:" << j << "\tdriver number:" << vDriver.size() << endl;
	cout << "Writing Raw Speed File" << endl;
	ofstream ofile(("../data/" + conf.city + "/" + conf.city + "Speed").c_str());
	ofile << g.mRoad.size() << endl;

	map<double, roadInfo>::iterator imRoad;
	map<double, vector<double> >::iterator	imV;
	vector<double>::iterator		iv;
	for(imRoad = g.mRoad.begin(); imRoad != g.mRoad.end(); imRoad++)
	{
		int num = 0;
		for(imV = (*imRoad).second.mV.begin(); imV != (*imRoad).second.mV.end(); imV++)
		{
			num += (*imV).second.size();
		}
		ofile << setprecision(15) << (*imRoad).first << "\t" << num;
		for(imV = (*imRoad).second.mV.begin(); imV != (*imRoad).second.mV.end(); imV++)
		{
			for(iv = (*imV).second.begin(); iv != (*imV).second.end(); iv++)
			{
				ofile << setprecision(15) << "\t" << (*imV).first << "\t" << *iv;
			}
		}
		ofile << endl;
	}
	ofile.close();
	
	cout << "Writing Driver Raw Speed File" << endl;
	ofstream ofDS(("../data/" + conf.city + "/" + conf.city + "DriverSpeed").c_str());
	ofDS << vDriver.size() << endl;
	vector<driver>::iterator	ivDriver;
	vector<int>::iterator		ivRoad;
	vector<double>::iterator	ivTime;
	vector<int>::iterator		ivSpeed;
	map<int, vector<double> >::iterator imRVtmp;	
	for(ivDriver = vDriver.begin(); ivDriver != vDriver.end(); ivDriver++)
	{
		ofDS << setprecision(15) << (*ivDriver).driverID << "\t" << (*ivDriver).vRoad.size();
		for(ivRoad = (*ivDriver).vRoad.begin(), ivTime = (*ivDriver).vTime.begin(), ivSpeed = (*ivDriver).vSpeed.begin(); ivRoad != (*ivDriver).vRoad.end(); ivRoad++, ivTime++, ivSpeed++)
		{
			ofDS << "\t" << *ivRoad << "\t" << *ivTime << "\t" << *ivSpeed;
		}
		ofDS << endl;
	}
	ofDS.close();
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
	ifstream inS((conf.dataPath+conf.city+ "/"+conf.city+"TrajectoryTime").c_str());
	if(!inS)
	{
		cout << "Cannot open speed file" << endl;
		return -1;
	}

	cout << "Reading Trajectory Time file" << endl;
	map<int,int> mVP;	//speed number, count
	map<int,int>::iterator imVP;
	int lineNum, sNum, i, j, NO;
	double roadID, t, v, cost, dt;
	stringstream ss;
	inS >> lineNum;
	for(i = 0; i < lineNum; i++)
	{
		inS >> NO;
		inS >> sNum;
		for(j = 0; j < sNum; j++)
		{
			inS >> roadID >> dt >> cost >> v;
			if(cost <= 0 || v <= 0)
				continue;
			g.mRoad[roadID].mGraV[(int)dt/T].push_back(v);
			g.mRoad[roadID].mVC[(int)dt/T] = 0;
			g.mRoad[roadID].mGraC[(int)dt/T].push_back(cost);
		}
	}
	inS.close();
	
	map<int, double> mGVT;
	map<int, double>::iterator imGVT;
	map<double, roadInfo>::iterator	imRoad;	
	map<int, vector<double> >::iterator imGraV;
	map<int, vector<double> >::iterator imGraC;		
	vector<double>::iterator iv;
	int sum = 0;
	double count = 0;
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
		
		for(i = 0; i < T; i++)			//Init mVC to 0
			(*imRoad).second.mVC[i] = 0;

		for(imGraC = (*imRoad).second.mGraC.begin(); imGraC != (*imRoad).second.mGraC.end(); imGraC++)
		{
			count = 0;
			s = 0;
			for(iv = (*imGraC).second.begin(); iv != (*imGraC).second.end(); iv++)
			{
				count += *iv;
				s++;
			}
			if(0 == s)
				continue;
			(*imRoad).second.mCost[(*imGraC).first] = (double)count/s;
			(*imRoad).second.mAvgV[(*imGraC).first] = (double)(*imRoad).second.length / ((double)count/s);
		}
	}
	
	for(imGVT = mGVT.begin(); imGVT != mGVT.end(); imGVT++)
	{	
		sum += (*imGVT).second;
		cout << (*imGVT).first << "\t" << (*imGVT).second << endl;
	}
	cout << "Road num:" << g.mRoad.size() << endl;

	cout << "Writing cost raw file" << endl;
	string sTN;
	ss.clear();
	ss.str("");
	ss << TN;
	ss >> sTN;
	ofstream ofile((conf.dataPath + conf.city+"/"+conf.splitType+"/"+conf.city+"AvgCost"+sTN+"Raw").c_str());
	map<int, double>::iterator imCost;	
	for(imRoad = g.mRoad.begin(); imRoad != g.mRoad.end(); imRoad++)
	{
		ofile << setprecision(15) << (*imRoad).first << "\t" << (*imRoad).second.mCost.size();
		for(imCost = (*imRoad).second.mCost.begin(); imCost != (*imRoad).second.mCost.end(); imCost++)
		{
			ofile << "\t" << (*imCost).first << "\t" << (*imCost).second << "\t" << (*imRoad).second.mAvgV[(*imCost).first];
		}
		ofile << endl;
	}
	ofile.close();
	cout << "Speed File Distribution:speed number, road number" << endl;
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
	stringstream ss;
	string sTN;
	ss.clear();
	ss.str("");
	ss << TN;
	ss >> sTN;
	ifstream inS((conf.dataPath + conf.city+"/"+conf.splitType+"/"+conf.city+"AvgCost"+sTN+"Raw").c_str());
//	cout << conf.avgSpeedFilePath << endl;
	if(!inS)
	{
		cout << "Cannot open average cost raw file" << endl;
		return -1;
	}
	cout << "Reading Average Cost Raw file" << endl;
	T = 60 * 60 * conf.h + 60 * conf.m;
	TN = 24 * 60 * 60 / (60 * 60 * conf.h + 60 * conf.m);

	int i, roadID, num, t;
	double c, v;
	for(i = 0; i < g.mRoad.size(); i++)
	{
		inS >> roadID;
		inS >> num;
		mTNumRoad[num].push_back(roadID);
		while(num != 0)
		{
			num--;
			inS >> t >> c >> v;
//			g.mRoad[roadID].mAvgV[t] = v;
			g.mRoad[roadID].mCost[t] = c;
		}
	}
	inS.close();
	
	return 0;
}
	
void RoadNetwork::fillVoidSpeedST(map<int, vector<int> > &mTNumRoad)
{
	map<double, bool>		mVisited;
	map<int, vector<int> >::iterator imTNR;
	vector<int>::iterator	ivR;
	set<int>::iterator		isSNR, isSNR2, isSNR3;
	map<int, double>::iterator	imAvgV;
	map<int, double>::iterator	imCost;
	int i, j, in, k;
	double vtmp;
	double ctmp;
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
	while(!qRoad.empty() && mVisited.size() != 294120)
//	while(!qRoad.empty() && mVisited.size() != 294120)
	{
		roadID = qRoad.front();
		qRoad.pop();
		insertQ = false;
		if(g.mRoad[roadID].mCost.size() == TN)
		{
			mVisited[roadID] = true;
			cout << mVisited.size() << endl;
		}
		else	//Fill itself's void
		{
			for(k = 0; k < TN; k++)
			{
				if(g.mRoad[roadID].mCost.find(k) == g.mRoad[roadID].mCost.end())
				{
//					vtmp = 0;
					ctmp = 0;
					j = 0;
					if(g.mRoad[roadID].mCost.find((k+1) % TN) != g.mRoad[roadID].mCost.end())//next hour
					{
//						vtmp += g.mRoad[roadID].mAvgV[(k+1) % TN];
						ctmp += g.mRoad[roadID].mCost[(k+1) % TN];
						j++;
					}
					if(g.mRoad[roadID].mCost.find((k-1+TN) % TN) != g.mRoad[roadID].mCost.end())//previous hour
					{
//						vtmp += g.mRoad[roadID].mAvgV[(k-1+TN) % TN];
						ctmp += g.mRoad[roadID].mCost[(k-1+TN) % TN];
						j++;
					}
					for(isSNR = g.mRoad[roadID].sNeighborRoad.begin(); isSNR != g.mRoad[roadID].sNeighborRoad.end(); isSNR++)//neighbor roads
					{
						if(g.mRoad[*isSNR].mCost.find(k) != g.mRoad[*isSNR].mCost.end())
						{
//							vtmp += g.mRoad[*isSNR].mAvgV[k];
							ctmp += g.mRoad[*isSNR].mCost[k];
							j++;
						}
					}
					if(j == 0)
					{
						insertQ = true;			
					}
					else
					{
//						g.mRoad[roadID].mAvgV[k] = vtmp / j;
						g.mRoad[roadID].mCost[k] = ctmp / j;
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
				if(g.mRoad[*isSNR].mCost.size() != TN)
				{
					for(i = 0; i < TN; i++)	//traverse the intervals
					{
						if(g.mRoad[*isSNR].mCost.find(i) == g.mRoad[*isSNR].mCost.end())//empty
						{
							ctmp = 0;
//							vtmp = 0;
							j = 0;
							if(g.mRoad[*isSNR].mCost.find((i+1) % TN) != g.mRoad[*isSNR].mCost.end())//next hour
							{
//								vtmp += g.mRoad[*isSNR].mAvgV[(i+1) % TN];
								ctmp += g.mRoad[*isSNR].mCost[(i+1) % TN];
								j++;
							}
							if(g.mRoad[*isSNR].mCost.find((i-1+TN) % TN) != g.mRoad[*isSNR].mCost.end())//previous hour
							{
//								vtmp += g.mRoad[*isSNR].mAvgV[(i-1+TN) % TN];
								ctmp += g.mRoad[*isSNR].mCost[(i-1+TN) % TN];
								j++;
							}
							for(isSNR2 = g.mRoad[*isSNR].sNeighborRoad.begin(); isSNR2 != g.mRoad[*isSNR].sNeighborRoad.end(); isSNR2++)//neighbor roads
							{
								if(g.mRoad[*isSNR2].mCost.find(i) != g.mRoad[*isSNR2].mCost.end())
								{
//								cout << setprecision(15) << *isSNR2 << "\t" << i << endl;
//									vtmp += g.mRoad[*isSNR2].mAvgV[i];
									ctmp += g.mRoad[*isSNR2].mCost[i];
									j++;
								}
							}
							if(j == 0)
							{
								insertQ = true;
							}
							if(j != 0)
							{
//								g.mRoad[*isSNR].mAvgV[i] = vtmp / j;
								g.mRoad[*isSNR].mCost[i] = ctmp / j;
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
//	string filename = conf.city + "AvgSpeed" + sTN;
	ofstream ofIR((conf.city + "IsoRoad").c_str());
	ofstream ofIN((conf.city + "IsoNode").c_str());
//	ofstream ofCOST((conf.city + "cost" + sTN).c_str());
	ofstream ofile((conf.dataPath+conf.city+"/"+conf.splitType+conf.MS+"/"+conf.city+"AvgSpeed"+sTN).c_str());
	ofstream ofCOST((conf.dataPath+conf.city+"/"+conf.splitType+conf.MS+"/"+conf.city+"Cost"+sTN).c_str());
	ofstream ofavg((conf.dataPath+conf.city+"/"+conf.splitType+"/"+conf.city+"AvgCost"+sTN+"Raw").c_str());
	ofstream ofavg2((conf.dataPath+conf.city+"/"+conf.splitType+conf.MS+"/"+conf.city+"AvgCost"+sTN).c_str());
	set<double> sout;
	set<double>::iterator isout;
	for(imRoad = g.mRoad.begin(); imRoad != g.mRoad.end(); imRoad++)
	{
		ofile << setprecision(15) << (*imRoad).first << "\t" << (*imRoad).second.mCost.size();
		ofCOST << setprecision(15) << (*imRoad).first << "\t" << (*imRoad).second.mCost.size();
		ofavg << setprecision(15) << (*imRoad).first << "\t" << (*imRoad).second.mCost.size();
		ofavg2 << setprecision(15) << (*imRoad).first << "\t" << (*imRoad).second.mCost.size();
		for(imCost = (*imRoad).second.mCost.begin(); imCost != (*imRoad).second.mCost.end(); imCost++)
		{
			ofile << setprecision(15) << "\t" << (*imCost).first << "\t" << (double)(*imRoad).second.length / (*imCost).second;
			ofCOST << setprecision(15) << "\t" << (*imCost).first << "\t" << (*imCost).second;
			ofavg << setprecision(15) << "\t" << (*imCost).first << "\t" << (*imCost).second << "\t" << (double)(*imRoad).second.length/(*imCost).second;
			ofavg2 << setprecision(15) << "\t" << (*imCost).first << "\t" << (*imCost).second << "\t" << (double)(*imRoad).second.length/(*imCost).second;
			
		}
		ofile << endl;
		ofCOST << endl;
		ofavg << endl;
		ofavg2 << endl;
		if((*imRoad).second.mCost.size() == 0)
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
	ofavg.close();
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
	ifstream ifCOST((conf.dataPath + conf.city + "/"+conf.splitType+conf.MS+"/"+conf.city+"AvgCost" + sTN).c_str());
	if(!ifCOST)
	{
		cout << "Cannot open " + conf.city + "cost" + sTN << endl;
		return -1;
	}

	map<int, double>::iterator	imAvgV;
	map<double, roadInfo>::iterator imRoad;
	map<int, double>::iterator imCost;
	int i, j, roadID, sNum, t;
	double c, v;
	cout << "Reading Avg Cost file" << endl;
	cout  << "ROAD SIZE" << g.mRoad.size() << endl;
	for(i = 0; i < g.mRoad.size(); i++)
	{
		ifCOST >> roadID;
		ifCOST >> sNum;
		for(j = 0; j < sNum; j++)
		{
			ifCOST >> t >> c >> v;
			g.mRoad[roadID].mCost[t] = c;
			g.mRoad[roadID].mAvgV[t] = v;
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
			g.mRoad[roadID].mVC[t] = 0;
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
	
double RoadNetwork::FastestSingleTimePoint(double ID1, double ID2, int startTime, vector<int> &vRoadList, vector<double> &vRTime, vector<double> & vRTakeTime, double &distance)
{
	double nID1 = mIDTrans[ID1];
	double nID2 = mIDTrans[ID2];
	vector<float>	vTime(g.mNode.size(), INF);	//used time
	vector<double>	vCurrentTime(g.mNode.size(), startTime);
	vector<double>	vPrevious(g.mNode.size(), -1);	//previous node

	priority_queue<h>	qh;
	vector<float>::iterator			ivD;
	map<double, int>::iterator		imNR;
	map<double, roadInfo>::iterator	imRoad;
	map<int, double>::iterator		imCost;

	int timeSlot = (int)(startTime / T);
	vTime[nID1] = 0;
	for(imNR = g.mNode[ID1].mSubNeighborRoad.begin(); imNR != g.mNode[ID1].mSubNeighborRoad.end(); imNR++)
	{
		double cost = g.mRoad[(*imNR).second].mCost[timeSlot];
		if(startTime + cost > (timeSlot + 1) * T)	//exceed the interval
		{
			double v1 = g.mRoad[(*imNR).second].mAvgV[timeSlot];
			double v2 = g.mRoad[(*imNR).second].mAvgV[(timeSlot+1)%TN];
			double l1 = v1 * ((timeSlot+1)*T - startTime);
			double l2 = g.mRoad[(*imNR).second].length - l1;
			double cost2 = l2 / v2;
			double tTime = cost2 + (timeSlot+1)*T - startTime;
			vTime[mIDTrans[(*imNR).first]] = tTime;
			vPrevious[mIDTrans[(*imNR).first]] = nID1;

			h hh;
			hh.pif = make_pair(mIDTrans[(*imNR).first], tTime);
			qh.push(hh);
			double eTime = cost2 + (timeSlot+1)*T;
			if(eTime >= 24*60*60)
				eTime -= 24*60*60;
			vCurrentTime[(*imNR).first] = eTime;
		}
		else
		{
			vTime[mIDTrans[(*imNR).first]] = cost;
			vCurrentTime[mIDTrans[(*imNR).first]] += cost;
			vPrevious[mIDTrans[(*imNR).first]] = nID1;
			h hh;
			hh.pif = make_pair(mIDTrans[(*imNR).first], cost);
			qh.push(hh);
		}
	}

	pair<double, float> pu;
	while(!qh.empty())
	{
		pu = qh.top().pif;
		qh.pop();

		if(pu.first == nID2)
			break;
			
		double	currentTime = vCurrentTime[pu.first];
		double	currentUsedTime = pu.second;
		int		timeSlot = (int)(currentTime / T);
		for(imNR = g.mNode[mRIDTrans[pu.first]].mSubNeighborRoad.begin(); imNR != g.mNode[mRIDTrans[pu.first]].mSubNeighborRoad.end(); imNR++)
		{
			double	cost = g.mRoad[(*imNR).second].mCost[timeSlot];
			double	tTime;	//used Time
			double	eTime;	//End Time
			if(currentTime + cost > (timeSlot + 1) * T)	//exceed the interval
			{
				double v1 = g.mRoad[(*imNR).second].mAvgV[timeSlot];
				double v2 = g.mRoad[(*imNR).second].mAvgV[(timeSlot+1)%TN];
				double l1 = v1 * ((timeSlot+1)*T - currentTime);
				double l2 = g.mRoad[(*imNR).second].length - l1;
				double cost2 = l2 / v2;
				tTime = cost2 + (timeSlot+1)*T - startTime;//used Time
				eTime = cost2 + (timeSlot+1)*T;
			}
			else
			{
				tTime = cost + currentUsedTime;
				eTime = cost + currentTime;
			}

			if(vTime[mIDTrans[(*imNR).first]] == INF && (*imNR).first != ID1)
			{
				vTime[mIDTrans[(*imNR).first]] = tTime;
				h hh;
				hh.pif = make_pair(mIDTrans[(*imNR).first], tTime);
				qh.push(hh);
				vPrevious[mIDTrans[(*imNR).first]] = pu.first;
				if(eTime >= 24*60*60)
					vCurrentTime[mIDTrans[(*imNR).first]] = eTime - 24*60*60;
				else
					vCurrentTime[mIDTrans[(*imNR).first]] = eTime;
			}
			else if(vTime[mIDTrans[(*imNR).first]] > tTime)
			{
				vTime[mIDTrans[(*imNR).first]] = tTime;
				vPrevious[mIDTrans[(*imNR).first]] = pu.first;
				h hh;
				hh.pif = make_pair(mIDTrans[(*imNR).first], tTime);
				qh.push(hh);
				if(eTime >= 24*60*60)
					vCurrentTime[mIDTrans[(*imNR).first]] = eTime - 24*60*60;
				else
					vCurrentTime[mIDTrans[(*imNR).first]] = eTime;
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
	vCurrentTime[nID1] = startTime;
	cout << "nID2:" << nID2 << endl;
	cout << "Time:" << vTime[nID2] << endl;
	double dist = 0;
	if(vTime[nID2] != INF)
	{	
		while(id != nID1)
		{
			idtmp = vPrevious[id];
			dist += g.mRoad[g.mNode[mRIDTrans[idtmp]].mSubNeighborRoad[mRIDTrans[id]]].length;
			vRoadListtmp.push_back(g.mNode[mRIDTrans[idtmp]].mSubNeighborRoad[mRIDTrans[id]]);
			vRTimetmp.push_back(vCurrentTime[idtmp]);
			vRTakeTimetmp.push_back(g.mRoad[g.mNode[mRIDTrans[idtmp]].mSubNeighborRoad[mRIDTrans[id]]].mCost[(int)(vCurrentTime[idtmp] / T)]);
			id = idtmp;
		}
	}
	else
	{
		cout << "Cannot Reach" << endl;
	}
	cout << "Distance:" << dist << endl;
	distance = dist;

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

void RoadNetwork::IntervalSingleFastestPaths(double ID1, double ID2, vector<int>& vRoadList, int t1, int t2, vector<int>& vt)
{
	priority_queue<tth>	qtth;
	map<double, int>::iterator			imSNR;
	vector<pair<int, int> > vpTI;
	vector<pair<int, int> >::iterator	ivpTI;
	for(imSNR = g.mNode[ID1].mSubNeighborRoad.begin(); imSNR != g.mNode[ID1].mSubNeighborRoad.end(); imSNR++)
	{
		vpTI.clear();
		cout << "roadID:" << (*imSNR).second << endl;
		if(getTimeInterval(t1, t2, (*imSNR).second, vpTI))
		{
			for(ivpTI = vpTI.begin(); ivpTI != vpTI.end(); ivpTI++)
			{
				cout << (*ivpTI).first << "\t" << (*ivpTI).second << endl;
			}
			cout << endl;
		}
	}
}

void RoadNetwork::testISFP()
{
	readCost();
	vector<int> vRoadList;
	vector<int> vt;
	cout << "Testing ISFP" << endl;
	IntervalSingleFastestPaths(76162675, 76162690,vRoadList, 83200, 3000, vt);
}

int RoadNetwork::getTimeInterval(int t1, int t2, double roadID, vector<pair<int, int> > &vpTI)
{
	if(conf.splitType == "Slot")
	{
		if(t2 < t1)
			t2 += 24*60*60;
		int ta = (int)(t1/T);
		int tb = (int)(t2/T);
		if(ta == tb)
			return 0;
		else
		{
			vector<int> vT;
			vector<int>::iterator ivT1, ivT2;
			vT.push_back(t1);
			for(int i = ta+1; i <=tb; i++)
			{
				vT.push_back(i*T - g.mRoad[roadID].mCost[i%TN]);
				vT.push_back(i*T);
			}
			vT.push_back(t2);
			if(vT[0] > vT[1])
				vT.erase(vT.begin()+1);
			for(ivT1 = vT.begin(), ivT2 = vT.begin()+1; ivT2 !=vT.end(); ivT1++, ivT2++)
			{
				if(*ivT2 > 24*60*60)
					*ivT2 -= 24*60*60;
				vpTI.push_back(make_pair(*ivT1, *ivT2));
			}
			return vpTI.size();
		} 
	}
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
	readCost();
	vector<pair<double, double> > vpNode;
	vpNode = generateNodePair(conf.testNum);
	vector<pair<double, double> >::iterator ivpNode;
	vector<int> vRoadList;
	vector<int>::iterator ivRL;
	vector<int> vTime;
	vector<int>::iterator ivTime;
	srand((unsigned)time(0));
	int i;
	for(i = 0; i < conf.testNum; i++)
	{
		vTime.push_back(rand() % (24 * 60 * 60));
	}

	double a,b, t;
	ifstream ifile("testPair");
/*	for(i = 0; i < 10; i++)
	{
		ifile >> a >> b >> t;
		vpNode.push_back(make_pair(a,b));
		vTime.push_back(t);
	}*/
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
	ofstream oPair("testPair");
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
		cout << setprecision(15) << (*ivpNode).first << "\t" << (*ivpNode).second << "\t" << (*ivTime) << endl;
		t = FastestSingleTimePoint((*ivpNode).first, (*ivpNode).second, *ivTime, vRoadList, vRTime, vRTakeTime, d);
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
		cout << "input time:" << (int)((*ivTime) / 3600) << "h " << (int)(((*ivTime) % 3600) / 60) << "min\t" << ((*ivTime)%3600)%60 << " second" << endl;

		oPair << (*ivpNode).first << "\t" << (*ivpNode).second << "\t" << (*ivTime) << endl;
		cout << "Time:" << (int)(t/3600) << "hour " << (int)(((int)(t)%3600)/60) << "minutes\t" << ((*ivTime)%3600)%60 << " second"<< endl;
		cout << "Distance:" << d << endl;
		vT.push_back(t);
		vD.push_back(d);
		for(ivRL = vRoadList.begin(), ivRT = vRTime.begin(), ivRTT = vRTakeTime.begin(); ivRL != vRoadList.end(); ivRL++, ivRT++, ivRTT++)
		{
			cout << *ivRL << "\t" << g.mRoad[*ivRL].length << "\tstart time:" << (int)((*ivRT) / 3600) <<"h " << (int)(((int)(*ivRT) % 3600)/60) << "min "<< ((*ivTime)%3600)%60 << " second" << endl;
			cout << setprecision(7)<< "\ttake time:" << *ivRTT << "second\t" << g.mNode[g.mRoad[*ivRL].ID1].x << "\t" << g.mNode[g.mRoad[*ivRL].ID1].y << "\t" << g.mNode[g.mRoad[*ivRL].ID2].x << "\t" << g.mNode[g.mRoad[*ivRL].ID2].y << endl; 
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
	
void RoadNetwork::speedRawClassify()
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
	ofstream ofAvg(("../data/" + conf.city + "/" + conf.city+"SpeedAvg"+sTN).c_str());
	ofstream ofDev(("../data/" + conf.city + "/" + conf.city+"SpeedDev"+sTN).c_str());
	cout << "Writing " << conf.city << "Speed classification" << endl;
	int i, count, span, min, max;
	vector<double> vtmp;
	vector<double>::iterator ivtmp;
	double v;
	
	for(imRoad = g.mRoad.begin(); imRoad != g.mRoad.end(); imRoad++)
	{
		for(imGraV = (*imRoad).second.mGraV.begin(); imGraV != (*imRoad).second.mGraV.end(); imGraV++)
		{
			if((*imGraV).second.size() <= 4)
			{
				i = 0;
				count = 0;
				for(iv = (*imGraV).second.begin(); iv != (*imGraV).second.end(); iv++)
				{
					i++;
					count += *iv;
				}
				v = (double)(count) / (double)(i);
				ofAvg << setprecision(15) << (*imRoad).first << "\t" << (*imGraV).first << "\t" << v << endl;
				g.mRoad[(*imRoad).first].mVC[(*imGraV).first] = 0;
			}
			else
			{
				max = 0;
				min = 99999999;
				for(iv = (*imGraV).second.begin(); iv != (*imGraV).second.end(); iv++)
				{
					if(*iv > max)
						max = *iv;
					if(*iv < min)
						min = *iv;
				}
				span = max - min;
				if(span == 0)
				{
					ofAvg << setprecision(15) << (*imRoad).first << "\t" << (*imGraV).first << max << endl;
					g.mRoad[(*imRoad).first].mVC[(*imGraV).first] = 0;
					continue;
				}

				vtmp.clear();
				for(iv = (*imGraV).second.begin(); iv != (*imGraV).second.end(); iv++)
				{
					vtmp.push_back((double)(*iv - min) / (double)(span));
				}
				double sum = accumulate(vtmp.begin(), vtmp.end(), 0.0);
				double mean =  sum / vtmp.size();
				double accum  = 0.0;
				for(ivtmp = vtmp.begin(); ivtmp != vtmp.end(); ivtmp++) 
				{
					accum  += (*ivtmp - mean) * (*ivtmp - mean);
				}
				double dev = accum / (double)(vtmp.size());
				ofDev << setprecision(15) << (*imRoad).first << "\t" << (*imGraV).first << "\t" << mean << "\t" << dev << endl;
				int CID;	//this CID is temporal, just for init use
				if(dev < 0.23)
				{
					CID = (int)(mean / 0.1 - 1) * 23 + (int)(dev / 0.01); 
				}
				else
					CID = (int)(mean / 0.1 - 1) * 23 + 23; 

				if(mRCU.find(CID) != mRCU.end())
				{
					mRCU[CID].vpRT.push_back(make_pair((*imRoad).first, (*imGraV).first));
				}
				else
				{
					rcu r;
					r.avg = (double)((int)(mean/0.1)) / 10;
					r.dev = (double)((int)(dev/0.01)) / 100;
					r.vpRT.push_back(make_pair((*imRoad).first, (*imGraV).first));
					mRCU[CID] = r;
				}
			}
		}
	}
	ofAvg.close();
	ofDev.close();

	vector<pair<int, int> >::iterator ivpRT;
	ofstream ofRC(("../data/" + conf.city + "/" + conf.city+"RC"+sTN).c_str());
	ofRC << mRCU.size() << endl;
	int j = 0;
	for(i = 0; i <= 200; i++)
	{
		if(mRCU.find(i) != mRCU.end())
		{
			j++;
			ofRC << j << "\t" << mRCU[i].avg << "\t" << mRCU[i].dev << "\t" << mRCU[i].vpRT.size();
			for(ivpRT = mRCU[i].vpRT.begin(); ivpRT != mRCU[i].vpRT.end(); ivpRT++)
			{
				ofRC  << "\t" << (*ivpRT).first << "\t" << (*ivpRT).second;
			}
			ofRC << endl;
		}
	}

	ofRC.close();
}
	
void RoadNetwork::driverAttach()
{
	readSpeed();
	readRC();
	calMinSpan();
	readDriverSpeed();
}

void RoadNetwork::calMinSpan()
{
	map<double, roadInfo>::iterator imRoad;
	map<int, int>::iterator			imVC;
	vector<double>::iterator		iv;
	int max, min;
	int count = 0;
	bool b = true;
	for(imRoad = g.mRoad.begin(); imRoad != g.mRoad.end(); imRoad++)
	{
		b = true;
		for(imVC = (*imRoad).second.mVC.begin(); imVC != (*imRoad).second.mVC.end(); imVC++)
		{
			if((*imVC).second != 0)
			{
				if(b)
				{
					b = false;
					count ++;
				}
				max = 0;
				min = 99999999;
				for(iv = (*imRoad).second.mGraV[(*imVC).first].begin(); iv != (*imRoad).second.mGraV[(*imVC).first].end(); iv++)
				{
					if(*iv > max)
						max = *iv;
					if(*iv < min)
						min = *iv;
				}
				(*imRoad).second.mSpanV[(*imVC).first] = max - min;
				(*imRoad).second.mMinV[(*imVC).first] = min;
			}
		}
	}
	
	stringstream ss;
	string sTN;
	ss.clear();
	ss.str("");
	ss << TN;
	ss >> sTN;
	ofstream ofile(("../data/" + conf.city + "/" + conf.city + "MS" + sTN).c_str());
	map<int, double>::iterator	imMinV;
	map<int, double>::iterator	imSpanV;
	ofile << count << endl;
	for(imRoad = g.mRoad.begin(); imRoad != g.mRoad.end(); imRoad++)
	{
		if((*imRoad).second.mMinV.size() != 0)
		{
			ofile << setprecision(15) << (*imRoad).first << "\t" << (*imRoad).second.mMinV.size();
			for(imMinV = (*imRoad).second.mMinV.begin(), imSpanV = (*imRoad).second.mSpanV.begin(); imMinV != (*imRoad).second.mMinV.end(); imMinV++, imSpanV++)
			{
				ofile << "\t" << (*imMinV).first << "\t" << (*imMinV).second << "\t" << (*imSpanV).second;
			}
			ofile << endl;
		}
	}
	ofile.close();
}
	
void RoadNetwork::readRC()
{
	stringstream ss;
	string sTN;
	ss.clear();
	ss.str("");
	ss << TN;
	ss >> sTN;
	ifstream iFRC(("../data/" + conf.city + "/" + conf.city + "RC" + sTN).c_str());
	int n1, n2;
	iFRC >> n1;
	int i, j;
	int rid, t, type;
	for(i = 0; i < n1; i++)
	{
		rcu r;
		iFRC >> type >> r.avg >> r.dev;
		iFRC >> n2;
		for(j = 0; j < n2; j++)
		{
			iFRC >> rid >> t;
			r.vpRT.push_back(make_pair(rid, t));
			g.mRoad[rid].mVC[t] = type;
		}
		mRCU[type] = r;
	}

/*	map<double, roadInfo>::iterator		imRoad;
	map<int, int>::iterator				imVC;
	for(imRoad = g.mRoad.begin(); imRoad != g.mRoad.end(); imRoad++)
	{
		cout << setprecision(15) << (*imRoad).first;
		for(imVC = (*imRoad).second.mVC.begin(); imVC != (*imRoad).second.mVC.end(); imVC++)
		{
			cout << "\t" << (*imVC).first << "\t" << (*imVC).second;
		}
		cout << endl;
	}*/
}

int RoadNetwork::readDriverSpeed()
{
	ifstream inDS(("../data/"+conf.city+"/"+conf.city+"DriverSpeedTime").c_str());
	if(!inDS)
	{
		cout << "Cannot open driver speed time file" << endl;
		return -1;
	}

	string stmp;
	stringstream ss;
	vector<string> vs, vL, vT, vV;
	int i, V, j, k, lnum, dnum, il, hour, min, sec;
	double L;
	j = 0;
	k = 0;
	map<double, double>	mDD;	//map the original car ID to driverID
	double dID, oid;
	vector<double>::iterator iv;
	cout << "Reading driver speed time" << endl;
	inDS >> lnum;
	for(il = 0; il < lnum; il++)
	{
		inDS >> oid >> dnum;
		if(mDD.find(oid) == mDD.end())
		{
			mDD[oid] = k;
			dID = k;
			driver d;
			d.driverID = dID;
			vDriver.push_back(d);
			k++;
		}
		else
		{
			dID = mDD[oid];
		}

		for(i = 0; i < dnum; i++)
		{
			inDS >> L >> hour >> min >> sec >> V;
			if(V == 0)
				continue;
			if(g.mRoad.find(L) == g.mRoad.end())
				continue;
			int ttmp = hour * 60 * 60 + min * 60 + sec;
			int cid = g.mRoad[L].mVC[(int)(ttmp / T)];
			if(cid != 0)
			{
				double vr = (double)((double)V-g.mRoad[L].mMinV[(int)(ttmp/T)]) / g.mRoad[L].mSpanV[(int)(ttmp/T)];
				if(vr < 0)
				{
					cout << vr << setprecision(15) << "\t" << V << "\t" << g.mRoad[L].mMinV[(int)(ttmp/T)] << "\t" << g.mRoad[L].mSpanV[(int)(ttmp/T)] << endl;
					cout << "roadID:" << L << "\tTimeslot:" << (int)(ttmp/T)<< "\t" << hour << "\t" << min << "\t" << ttmp << "\t" << T << endl;
					for(iv = g.mRoad[L].mGraV[(int)(ttmp/T)].begin(); iv != g.mRoad[L].mGraV[(int)(ttmp/T)].end(); iv++)
						cout << *iv << "\t";
					cout << endl;
				}

				vDriver[dID].mRVtmp[cid].push_back(vr);
			}
		}
		vL.clear();
		vT.clear();
		vV.clear();
		vs.clear();
		j++;
	}
	inDS.close();

	string sTN;
	ss.clear();
	ss.str("");
	ss << TN;
	ss >> sTN;
	ofstream ofile(("../data/" + conf.city + "/" + conf.city + "DriverRCDetail" + sTN).c_str());
	ofile << vDriver.size() << endl;

	vector<driver>::iterator ivDriver;
	map<int, vector<double> >::iterator imRVtmp;	
	for(ivDriver = vDriver.begin(); ivDriver != vDriver.end(); ivDriver++)
	{
		ofile << setprecision(15) << (*ivDriver).driverID << "\t" << (*ivDriver).mRVtmp.size();
		for(imRVtmp = (*ivDriver).mRVtmp.begin(); imRVtmp != (*ivDriver).mRVtmp.end(); imRVtmp++)
		{
			ofile << setprecision(15) << "\t" << (*imRVtmp).first << "\t" << (*imRVtmp).second.size();
			for(iv = (*imRVtmp).second.begin(); iv != (*imRVtmp).second.end(); iv++)
			{
				ofile << "\t" << *iv;
			}
		}
		ofile << endl;
	}
	ofile.close();
	return 0;
}
	
int	RoadNetwork::driverAvg()
{
	readRC();
	stringstream ss;
	string sTN;
	ss.clear();
	ss.str("");
	ss << TN;
	ss >> sTN;
	ifstream ifDRC(("../data/" + conf.city + "/" + conf.city + "DriverRCDetail" + sTN).c_str());
	if(!ifDRC)
	{
		cout << "Cannot open driver RC detail file" << endl;
		return -1;
	}

	int num1, num2, num3, i, j, k, t;
	double v, vtmp;
	ifDRC >> num1;
	for(i = 0 ; i < num1; i++)
	{
		driver d;
		ifDRC >> d.driverID;
		ifDRC >> num2;
		for(j = 0; j < num2; j++)
		{
			ifDRC >> t;
			ifDRC >> num3;
			v = 0;
			for(k = 0; k < num3; k++)
			{
				ifDRC >> vtmp;
				v += vtmp;
			}
			v = v / num3;
			d.mRV[t] = v;
		}
		vDriver.push_back(d);
	}
	ifDRC.close();

	cout << "Writing driver avg speed file" << endl;
	ofstream ofile(("../data/" + conf.city + "/" + conf.city + "DriverRC" + sTN).c_str());
	vector<driver>::iterator ivD;
	map<int, double>::iterator imRV;
	ofile << vDriver.size() << "\t" << mRCU.size() << endl;
	for(ivD = vDriver.begin(); ivD != vDriver.end(); ivD++)
	{
		ofile << (*ivD).driverID << "\t" << (*ivD).mRV.size();
		for(imRV = (*ivD).mRV.begin(); imRV != (*ivD).mRV.end(); imRV++)
		{
			ofile << "\t" << (*imRV).first << "\t" << (*imRV).second;
		}
		ofile << endl;
	}
}
	
void RoadNetwork::testDriverFastest()
{
	readDriver();
	//readTestSet();
	//tests
}
	
void RoadNetwork::readDriver()
{
	readTotalAvgSpeed();
	readMinSpan();
	readRC();
	readDriverProfile();
}

int	RoadNetwork::readMinSpan()
{
	stringstream ss;
	string sTN;
	ss.clear();
	ss.str("");
	ss << TN;
	ss >> sTN;
	ifstream ifMS(("../data/" + conf.city + "/" + conf.city + "MS" + sTN).c_str());
	if(!ifMS)
	{
		cout << "Cannot open min span file" << endl;
		return -1;
	}

	int num1, num2, i, j, t, min, span;
	double roadID;
	ifMS >> num1;
	for(i = 0 ; i < num1; i++)
	{
		ifMS >> roadID >> num2;
		for(j = 0; j < num2; j++)
		{
			ifMS >> t >> min >> span;
			g.mRoad[roadID].mMinV[t] = min;
			g.mRoad[roadID].mSpanV[t] = span;
		}
	}
	ifMS.close();
}
	
int	RoadNetwork::readDriverProfile()
{
	stringstream ss;
	string sTN;
	ss.clear();
	ss.str("");
	ss << TN;
	ss >> sTN;

	ifstream ifDRCF(("../data/" + conf.city + "/" + conf.city + "DriverRC" + sTN + "Final").c_str());
	if(!ifDRCF)
	{
		cout << "Cannot open driver RC Final file" << endl;
		return -1;
	}

	cout << "Reading Driver RC Final file" << endl;
	int num1, num2, i, j, t;
	double v;
	int driverID;
/*	ifDRCF >> num1;
	for(i = 0 ; i < num1; i++)	//For 24 data format
	{
		driver d;
		ifDRCF >> d.driverID;
		ifDRCF >> num2;
		for(j = 0; j < num2; j++)
		{
			ifDRCF >> t;
			ifDRCF >> v;
			d.mRV[t] = v;
		}
		vDriver.push_back(d);
	}*/

	ifDRCF >> num1 >> num2;	//For 48 data format
	cout << "vDriver size:" << vDriver.size() << endl;
	cout << num1 << "\t" << num2 << endl;
	for(i = 0 ; i <= num1; i++)		
	{
		driver d;
		d.driverID = i;
//		ifDRCF >> d.driverID;
		for(j = 0; j < num2; j++)
		{
			ifDRCF >> v;
			d.mRV[j+1] = v;
	//		cout << "i:" << i << "j:" << j << endl;
		}
		vDriver.push_back(d);
	}
	cout << "vDriver size:" << vDriver.size() << endl;
	ifDRCF.close();
	
	ifstream ifDRC(("../data/" + conf.city + "/" + conf.city + "DriverRC" + sTN).c_str());
	if(!ifDRC)
	{
		cout << "Cannot open driver RC file" << endl;
		return -1;
	}

	cout << "Reading Driver RC file" << endl;
	ifDRC >> num1;
	for(i = 0 ; i < num1; i++)
	{
//		driver d;
		ifDRC >> driverID;
		ifDRC >> num2;
		for(j = 0; j < num2; j++)
		{
			ifDRC >> t;
			ifDRC >> v;
			vDriver[driverID].mRV[t] = v;
		}
//		vDriver.push_back(d);
	}
	cout << "vDriver size:" << vDriver.size() << endl;
	ifDRC.close();
	
	return 0;
}
	
double	RoadNetwork::shortestTimeDijDriver(double ID1, double ID2, int t1, vector<int>& vRoadList, vector<double>& vRTime, vector<double>& vRTakeTime, double &d, double driverID)
{
	map<int, map<int, double> > mmCost;	//roadID,t,cost for CID>0
	map<int, map<int, double> > mmV;	//roadID,t,V for CID>0
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
	double cost;
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
		cost = getDriverCost(driverID, (*imNR).second, tt, mmCost, mmV);
		if((int)((t1 + cost) / T) == tt)	//does not exceed to next interval
		{
			vTime[mIDTrans[(*imNR).first]] = cost;	//used time
			vT[mIDTrans[(*imNR).first]] += cost;	//current
			h hh;
			hh.pif = make_pair(mIDTrans[(*imNR).first], cost);
			qh.push(hh);
			vPrevious[mIDTrans[(*imNR).first]] = mIDTrans[ID1];
		}
		else
		{
			double v1 = getDriverV(driverID, (*imNR).second, tt, mmV);
			double v2 = getDriverV(driverID, (*imNR).second, (tt + 1) % TN, mmV);
			dtmp = g.mRoad[(*imNR).second].length - v1 * (((tt + 1) * T - t1));
			ttmp = dtmp / v2 + (tt + 1) * T;
			if(ttmp < 0)
			{
//				cout << "init" << endl;
			}
			vTime[mIDTrans[(*imNR).first]] = ttmp - t1;
			h hh;
			hh.pif = make_pair(mIDTrans[(*imNR).first], ttmp - t1);
			qh.push(hh);
			vPrevious[mIDTrans[(*imNR).first]] = mIDTrans[ID1];
			if(ttmp > 24 * 60 * 60)
				ttmp -= 24 * 60 * 60;
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
			cost = getDriverCost(driverID, (*imNR).second, (int)(vT[pu.first]/T), mmCost, mmV);
			if((int)((vT[pu.first] +  cost) / T) == (int)(vT[pu.first] / T))
			{
				ttmp = pu.second + cost;
//				cout << "ttmp original1:" << ttmp << "\t" << (int)(vT[pu.first]/T ) << "\t" << g.mRoad[(*imNR).second].mCost[(int)(vT[pu.first] / T)] << "\t" << (*imNR).second <<endl;
//				cout << "pu second:" << pu.second << "\t" << g.mRoad[(*imNR).second].mCost[(int)(vT[pu.first] / T)] << endl;
//				cout << "!!ttmp:" << ttmp << endl;
			}
			else
			{
				double v1 = getDriverV(driverID, (*imNR).second, (int)(vT[pu.first]/T), mmV);
				double v2 = getDriverV(driverID, (*imNR).second, (int)(vT[pu.first]/T) % TN, mmV);
				dtmp = g.mRoad[(*imNR).second].length -  v1 * (((((int)(vT[pu.first] / T) + 1) * T - vT[pu.first])) * T);
//				cout << "dtmp:" << dtmp << endl;
				ttmp = dtmp / v2 + (((int)(vT[pu.first] / T) + 1) * T  - t1);
//				cout << "ttmp original2:" << ttmp << endl;
				if(ttmp < 0)
				{
					cout << "length:" <<  g.mRoad[(*imNR).second].length << endl;
					cout << "v:" <<  v1 << endl;
					cout << "t:" << ((((int)(vT[pu.first] / T) + 1) * T - vT[pu.first]))  << endl;
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
				if(t1 + ttmp > 24*60*60)
					vT[mIDTrans[(*imNR).first]] = t1 + ttmp - 24*60*60;
				else
					vT[mIDTrans[(*imNR).first]] = t1 + ttmp;
			}
//			else if(vTime[mIDTrans[(*imNR).first]] > vTime[pu.first] + ttmp)
			else if(vTime[mIDTrans[(*imNR).first]] > ttmp)
			{
//				vTime[mIDTrans[(*imNR).first]] = vTime[pu.first] + ttmp;
				vTime[mIDTrans[(*imNR).first]] = ttmp;
//				cout << "ttmp2:" << ttmp << endl;
				if(t1 + ttmp > 24*60*60)
					vT[mIDTrans[(*imNR).first]] = t1 + ttmp - 24*60*60;
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
	
double	RoadNetwork::getDriverCost(int driverID, int roadID, int t, map<int, map<int, double> > &mmCost, map<int, map<int, double> > &mmV)
{
	if(g.mRoad[roadID].mVC[t] == 0)
		return g.mRoad[roadID].mCost[t];
	else
	{
		double v;
		bool exist = false;
		if(mmCost.find(roadID) != mmCost.end())
		{
			if(mmCost[roadID].find(t) != mmCost[roadID].end())
			{
				exist = true;
				return mmCost[roadID][t];
			}
		}

		if(!exist)
		{
			v = vDriver[driverID].mRV[g.mRoad[roadID].mVC[t]] * g.mRoad[roadID].mSpanV[t] + g.mRoad[roadID].mMinV[t];
			mmV[roadID][t] = v;
			mmCost[roadID][t] = g.mRoad[roadID].length / v;
		}
		return g.mRoad[roadID].length / v;
	}
}
	
double  RoadNetwork::getDriverV(int driverID, int roadID, int t, map<int, map<int, double> > &mmV)
{
	if(g.mRoad[roadID].mVC[t] == 0)
		return g.mRoad[roadID].mAvgV[t];
	else
	{
		double v;
		bool exist = false;
		if(mmV.find(roadID) != mmV.end())
		{
			if(mmV[roadID].find(t) != mmV[roadID].end())
			{
				exist = true;
				return  mmV[roadID][t];
			}
		}

		if(!exist)
		{
			v = vDriver[driverID].mRV[g.mRoad[roadID].mVC[t]] * g.mRoad[roadID].mSpanV[t] + g.mRoad[roadID].mMinV[t];
		}
		return v;
	}
}
	
int RoadNetwork::extractBriefTrajectory()
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
	vector<string> vs, vT, vL, vV;
	vector<string>::iterator ivV;
	int i, V, j;
	double T, L;
	j = 0;
	cout << "Extracting Trajectory Brief" << endl;
	map<string, double>	mDD;	//map the original car ID to driverID
	double dID;
	int k = 0, count = 0;
	int v;
	while(getline(inTraj, stmp))
	{
		if(j % 10000 == 0)
			cout << j << endl;
		j++;
		vs = split(stmp, ",");
		
		vV = split(vs[9], "|");
		bool c = false;
		for(ivV = vV.begin(); ivV != vV.end(); ivV++)
		{
			ss.clear();
			ss.str("");
			ss << *ivV;
			ss >> v;
			if(v < 0)
			{
				c = true;
				break;
			}
		}
		if(c)
		{
			cout << "v<0" << endl;
			continue;
		}
		
		string oid = vs[2];
		if(mDD.find(oid) == mDD.end())
		{
			mDD[oid] = k;
			dID = k;
			driver d;
			d.driverID = dID;
			vDriver.push_back(d);
			k++;
		}
		else
		{
			dID = mDD[oid];
		}

		TB tb;
		
		vL = split(vs[4], "|");
		vT = split(vs[8], "|");

		for(i = 0; i < vL.size(); i++)
		{
			ss.clear();
			ss.str("");
			ss << vL[i];
			ss >> L;
//			if(L < 0)
//				L = -L;
			tb.vRoad.push_back(L);
			
			if(g.mRoad.find(L) == g.mRoad.end())
				continue;

			ss.clear();
			ss.str("");
			ss << vT[i];
			ss >> T;
		}

		ss.clear();
		ss.str("");
		ss << vs[26];
		ss >> tb.sTimeStamp;

		ss.clear();
		ss.str("");
		ss << vs[27];
		ss >> tb.eTimeStamp;

		ss.clear();
		ss.str("");
		ss << vs[23];
		ss >> tb.sx;
		ss.clear();
		ss.str("");
		ss << vs[22];
		ss >> tb.sy;
		
		ss.clear();
		ss.str("");
		ss << vs[25];
		ss >> tb.ex;
		ss.clear();
		ss.str("");
		ss << vs[24];
		ss >> tb.ey;

		vT.clear();
		vL.clear();
		vs.clear();
		vDriver[dID].vTB.push_back(tb);
	}
	inTraj.close();

	ofstream ofile(("../data/" + conf.city + "/" + conf.city + "DriverTrajectory").c_str());
	ofile << vDriver.size() << endl;
	cout << "Writing driver trajectory" << endl;
	vector<driver>::iterator	ivDriver;
	vector<TB>::iterator		ivTB;
	vector<int>::iterator		ivRoad;
	for(ivDriver = vDriver.begin(); ivDriver != vDriver.end(); ivDriver++)
	{
		ofile << (*ivDriver).driverID << "\t" << (*ivDriver).vTB.size() << endl;
		for(ivTB = (*ivDriver).vTB.begin(); ivTB != (*ivDriver).vTB.end(); ivTB++)
		{ 
			ofile << setprecision(15) << (*ivTB).sTimeStamp << "\t" << (*ivTB).eTimeStamp << "\t" << (*ivTB).sx << "\t" << (*ivTB).sy << "\t" << (*ivTB).ex << "\t" << (*ivTB).ey << "\t" << (*ivTB).vRoad.size();
			for(ivRoad = (*ivTB).vRoad.begin(); ivRoad != (*ivTB).vRoad.end(); ivRoad++)
			{
				ofile << "\t" << *ivRoad;
			}
			ofile << endl;
		}
	}
	ofile.close();
}
	
int	RoadNetwork::readBriefTrajectory()
{
	ifstream inBT((conf.dataPath + conf.city + "/" + conf.city + "DriverTrajectoryTime").c_str());
	if(!inBT)
	{
		cout << "Cannot open brief trajectory file" << endl;
		return -1;
	}
	cout << "Reading Brief Trajectory File" << endl;

	int driverNum, tNum, rNum, i, j, k, driverID, hour, min, sec, roadID;
	inBT >> driverNum;
	for(i = 0; i < driverNum; i++)
	{
		inBT >> driverID >> tNum;
//		vector<TB> vTB;
		for(j = 0; j < tNum; j++)
		{
			TB tb;
			tb.id = j;
			inBT >> hour >> min >> sec;
			tb.sTime = hour*60*60 + min*60 +sec;
			inBT >> hour >> min >> sec;
			tb.eTime = hour*60*60 + min*60 +sec;
			inBT >> tb.sx >> tb.sy >> tb.ex >> tb.ey;
			inBT >> rNum;
			vector<int> vRoad;
			for(k = 0; k < rNum; k++)
			{
				inBT >> roadID;
				vRoad.push_back(roadID);
			}
			tb.vRoad = vRoad;
			vDriver[driverID].vTB.push_back(tb);
		}
	}
	inBT.close();
}
	
double RoadNetwork::testDriver()
{
	readDriver();
	readCost();
	readBriefTrajectory();
	vector<driver>::iterator	ivDriver;
	vector<TB>::iterator		ivTB;
	int i = 0;
	bool diff;
	double driverD = 0;
	double avgD = 0;
	int j = 0;
	int dn=0;
	int an=0;
	for(ivDriver = vDriver.begin(); ivDriver != vDriver.end(); ivDriver++)
	{
//		if(i > 50)
//			break;
		for(ivTB = (*ivDriver).vTB.begin(); ivTB != (*ivDriver).vTB.end(); ivTB++)
		{
			i++;
			cout << "driverID:" << (*ivDriver).driverID << "\tTid:" << (*ivTB).id << endl;
			double t1 = recreatePathTimeDriver((*ivDriver).driverID, (*ivTB).id, diff, i);
			double t2 = recreatePathTimeAvg((*ivDriver).driverID, (*ivTB).id);
			double sTime = (*ivTB).sTime;
			double eTime = (*ivTB).eTime;
			if(t1 < sTime)
				t1 += 24*60*60;
			if(t2 < sTime)
				t2 += 24*60*60;
			if(eTime < sTime)
				eTime += 24*60*60;
			cout << endl << "Test " << i << endl;
			if(!diff)
				cout << "Driver and Avg are the same" << endl;
			else
			{
				j++;
				cout << "Different" << endl;
	//			driverD += fabs(t1 - eTime);
			}
			cout << "Driver use time:" << t1 - sTime << " seconds" << endl;
			cout << "Avg use time:" << t2 - sTime << " seconds" << endl;
			cout << "Original trajectory use time:" << eTime - sTime << endl << endl;
			int dd = fabs(t1 - sTime - (eTime - sTime));
			int ad = fabs(t2 - sTime - (eTime - sTime));
			if(dd < ad)
				dn++;
			else if(dd > ad)
				an++;
			avgD += fabs((double)(t2 - eTime)/(double)(eTime-sTime));
			driverD += fabs((double)(t1 - eTime)/(double)(eTime-sTime));
		}
	}
	cout << "i:" << i << "\tj:" << j << endl;
	cout << driverD  << "\t" << avgD << endl;
	cout << endl << "Driver avg diff :" << driverD / i << endl;
	cout << "Avg avg diff :" << avgD / i<< endl;
	cout << "Driver near time:" << dn << "\tAvg near time:" << an << endl;
}
	
double RoadNetwork::recreatePathTimeDriver(int driverID, int trajectoryID, bool &diff, int testNO)
{
	vector<int>::iterator	ivRoad;
	int sTime = vDriver[driverID].vTB[trajectoryID].sTime;
	double currentTime = sTime;
	int roadID;
	diff = false;
	map<int, map<int, double> > mmCost;
	map<int, map<int, double> > mmV;
	for(ivRoad = vDriver[driverID].vTB[trajectoryID].vRoad.begin(); ivRoad != vDriver[driverID].vTB[trajectoryID].vRoad.end(); ivRoad++)
	{
		int timeSlot = (int)(currentTime/T);
		if(*ivRoad < 0)
			roadID = -(*ivRoad);
		else
			roadID = *ivRoad;
		if(g.mRoad[roadID].mVC[timeSlot] == 0)
		{
			double cost = g.mRoad[roadID].mCost[timeSlot];
			if(currentTime + cost > (timeSlot + 1) * T)
			{
				double v = g.mRoad[roadID].mAvgV[timeSlot];
				double l1 = v * ((timeSlot+1)*T - currentTime);
				double l2 = g.mRoad[roadID].length - l1;
				double cost2;
			
				if(g.mRoad[(timeSlot + 1) % TN].mVC[(timeSlot+1)%TN] == 0)
				{
					cost2 = l2 / g.mRoad[roadID].mAvgV[(timeSlot+1) % TN];				
				}
				else
				{
					cost2 = l2 / getDriverV(driverID, roadID, (timeSlot+1)%TN, mmV);
				}
				currentTime = (timeSlot+1)*T + cost2;
			}
			else
			{
				currentTime += cost;
			}
		}
		else
		{
			diff = true;
			double cost = getDriverCost(driverID, roadID, timeSlot, mmCost, mmV);
			if(currentTime + cost > (timeSlot + 1) * T)
			{
				double v = g.mRoad[roadID].mAvgV[timeSlot];
				double l1 = v * ((timeSlot+1)*T - currentTime);
				double l2 = g.mRoad[roadID].length - l1;
				double cost2;
				if(g.mRoad[(timeSlot + 1) % TN].mVC[(timeSlot+1)%TN] == 0)
				{
					cost2 = l2 / g.mRoad[roadID].mAvgV[(timeSlot+1) % TN];
				}
				else
				{
					cost2 = l2 / getDriverV(driverID, roadID, (timeSlot+1)%TN, mmV);
				}
				currentTime = (timeSlot+1)*T + cost2;
			}
			else
			{
				currentTime += cost;
			}
		}
	}

	if(currentTime >= 24*60*60)
		currentTime -= 24*60*60;

	return currentTime;
}

double RoadNetwork::recreatePathTimeAvg(int driverID, int trajectoryID)
{
	vector<int>::iterator	ivRoad;
	int sTime = vDriver[driverID].vTB[trajectoryID].sTime;
	double currentTime = sTime;
	int roadID;
	map<int, map<int, double> > mmCost;
	map<int, map<int, double> > mmV;
	for(ivRoad = vDriver[driverID].vTB[trajectoryID].vRoad.begin(); ivRoad != vDriver[driverID].vTB[trajectoryID].vRoad.end(); ivRoad++)
	{
		int timeSlot = (int)(currentTime/T);
		if(*ivRoad < 0)
			roadID = -(*ivRoad);
		else
			roadID = *ivRoad;
		double cost = g.mRoad[roadID].mCost[timeSlot];
		if(currentTime + cost > (timeSlot + 1) * T)
		{
			double v = g.mRoad[roadID].mAvgV[timeSlot];
			double l1 = v * ((timeSlot+1)*T - currentTime);
			double l2 = g.mRoad[roadID].length - l1;
			double cost2;
			cost2 = l2 / g.mRoad[roadID].mAvgV[(timeSlot+1) % TN];
			currentTime = (timeSlot+1)*T + cost2;
		}
		else
		{
			currentTime += cost;
		}
	}

	if(currentTime >= 24*60*60)
		currentTime -= 24*60*60;

	return currentTime;
}

int	RoadNetwork::extractTrajectoryTime()
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
	vector<int>		vRoad;
	vector<double>	vTime;
	vector<int>		vSpeed;
	vector<int>::iterator		ivRoad;
	vector<double>::iterator	ivTime;
	vector<int>::iterator		ivSpeed;
	int i, V, j;
	double T, L;
	j = 0;
	cout << "Read Trajectory" << endl;
	int k = 0;
	ofstream ofile(("../data/" + conf.city + "/" + conf.city + "TrajectoryTimestamp").c_str());

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

			if(V <= 0)
				continue;

			vRoad.push_back(L);
			vTime.push_back(T);
			vSpeed.push_back(V);
		}
		
		ofile << j << "\t" << vRoad.size();
		for(ivRoad = vRoad.begin(), ivTime = vTime.begin(), ivSpeed = vSpeed.begin(); ivRoad != vRoad.end(); ivRoad++, ivTime++, ivSpeed++)
		{
			ofile << setprecision(15) << "\t" << *ivRoad << "\t"  << g.mRoad[*ivRoad].length << "\t" << *ivTime << "\t" << *ivSpeed;
		}
		ofile << "\t" << vs[27] << endl;
		vL.clear();
		vT.clear();
		vV.clear();
		vs.clear();
		vRoad.clear();
		vTime.clear();
		vSpeed.clear();
		j++;
	}
	inTraj.close();
	ofile.close();
}
	
