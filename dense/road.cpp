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
	
	readRoad();
	readNode();
//	readTrajectory();
//	readSpeed();
//	organizeSpeed();
//	readAvgSpeed();
	readCost();//
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
		ri.ID1 = id;

		ss.clear();
		ss.str("");
		ss << vs[4];
		ss >> id;
		ri.ID2 = id;

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

	int nodeID, type, mainID, nodeNum, i, j, num, itmp;
	double dtmp;
	double x, y;
	map<double, int>::iterator imSNR1, imSNR2;
	vector<int> vRoad;
	vector<int>::iterator ivRoad1, ivRoad2;
	inNodeFile >> nodeNum;
	cout << nodeNum << endl;
	for(i = 0; i < nodeNum; i++)
	{
		node n;
		inNodeFile >> n.ID;
		mIDTrans[n.ID] = i;	//Fill in the ID map
		mRIDTrans[i] = n.ID;	//Fill in the ID map
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
		inNodeFile >> dtmp;

		inNodeFile >> num;
		vRoad.clear();
		for(j = 0; j < num; j++)	//subNeighbor Road
		{
			inNodeFile >> dtmp;
			vRoad.push_back(dtmp);
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
		g.mNode[n.ID] = n;

		//Fill the road sNeighborRoad info
		for(ivRoad1 = vRoad.begin(); ivRoad1 != vRoad.end(); ivRoad1++)
		{
			for(ivRoad2 = ivRoad1 + 1; ivRoad2 != vRoad.end(); ivRoad2++)
			{
				g.mRoad[*ivRoad1].sNeighborRoad.insert(*ivRoad2);
				g.mRoad[*ivRoad2].sNeighborRoad.insert(*ivRoad1);
			}
		}
/*		for(imSNR1 = n.mSubNeighborRoad.begin(); imSNR1 != n.mSubNeighborRoad.end(); imSNR1++)
		{
			for(imSNR2 = n.mSubNeighborRoad.begin(); imSNR2 != n.mSubNeighborRoad.end(); imSNR2++)
			{
				if((*imSNR1).second != (*imSNR2).second)
				{
					g.mRoad[(*imSNR1).second].sNeighborRoad.insert((*imSNR2).second);
					g.mRoad[(*imSNR1).second].sNeighborRoad.insert(n.ID);
					
					g.mRoad[(*imSNR2).second].sNeighborRoad.insert((*imSNR1).second);
				}
			}
		}*/
	}
	inNodeFile.close();

/*	map<double, node>::iterator imNode;
	for(imNode = g.mNode.begin(); imNode != g.mNode.end(); imNode++)
	{
		for(imSNR1 = (*imNode).second.mSubNeighborRoad.begin(); imSNR1 != (*imNode).second.mSubNeighborRoad.end(); imSNR1++)
		{
			for(imSNR2 = (*imNode).second.mSubNeighborRoad.begin(); imSNR2 != (*imNode).second.mSubNeighborRoad.end(); imSNR2++)
			{
				if((*imSNR1).second != (*imSNR2).second)
				{
					g.mRoad[(*imSNR1).second].sNeighborRoad.insert((*imSNR2).second);
					g.mRoad[(*imSNR2).second].sNeighborRoad.insert((*imSNR1).second);
				}
			}
		}
	}*/

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
	
void RoadNetwork::outputSpeed()
{
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
	char* format = "%Y%m%d%H%M%S";
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
			for(iv = (*imGraV).second.begin(); iv != (*imGraV).second.end(); iv++)
			{
				count += *iv; 
			}
			avgV = count / (*imGraV).second.size();
			if(avgV == 0)
				avgV = 0.1;
//			(*imRoad).second.mAvgV[(*imGraV).first] = count / (*imGraV).second.size();
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
	while(!qRoad.empty() && mVisited.size() != 298483)
//	while(!qRoad.empty() && mVisited.size() != 298000)
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
			if((*imAvgV).second == 0)
				(*imAvgV).second = 0.1;
//			(*imRoad).second.mCost[(*imAvgV).first] = (*imRoad).second.length / ((*imAvgV).second * 3.6);
			ofile << setprecision(15) << "\t" << (*imAvgV).first << "\t" << (*imAvgV).second;
			ofCOST << setprecision(15) << "\t" << (*imAvgV).first << "\t" << (*imRoad).second.length / ((*imAvgV).second / 3.6);
//			ofCOST << setprecision(15) << "\t" << (*imAvgV).first << "\t" << (*imRoad).second.length / ((*imAvgV).second / 3.6) << "\t" << (*imRoad).second.length << "\t" << (*imAvgV).second;
		}
		ofile << endl;
		ofCOST << endl;
		if((*imRoad).second.mAvgV.size() == 0)
		{
			for(isSNR = (*imRoad).second.sNeighborRoad.begin();isSNR != (*imRoad).second.sNeighborRoad.end(); isSNR++)
			{
				sout.insert(*isSNR);
			}
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
	for(i = 0; i < g.mRoad.size(); i++)
	{
		ifCOST >> roadID;
		ifCOST >> sNum;
		cout << roadID << "\t" << sNum;
		for(j = 0; j < sNum; j++)
		{
			ifCOST >> h >> t;
			cout << "\t" << h << "\t" << t;
			g.mRoad[roadID].mCost[h] = t;
		}
		cout << endl;
	}
	ifCOST.close();

	for(imRoad = g.mRoad.begin(); imRoad != g.mRoad.end(); imRoad++)
	{
		cout << setprecision(15) << (*imRoad).first << "\t" << (*imRoad).second.mCost.size();
		for(imCost = (*imRoad).second.mCost.begin(); imCost != (*imRoad).second.mCost.end(); imCost++)
		{
			cout << "\t" << (*imCost).first << "\t" << (*imCost).second;
		}
		cout << endl;
	}
}

double RoadNetwork::nodeDist(double x1, double y1, double x2, double y2)
{
	return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}
	
float RoadNetwork::distanceDijkstra(double ID1, double ID2, vector<double>& vRoadList)
{
	double nID1 = mIDTrans[ID1];
	double nID2 = mIDTrans[ID2];
	cout << setprecision(15) << "ID1:" << ID1 << "\t" << nID1 << endl;
	cout << setprecision(15) << "nID1" << nID1 << "\t" << mRIDTrans[nID1] << endl;
	cout << setprecision(15) << "ID2:" << ID2 << "\t" << nID2 << endl;
	cout << setprecision(15) << "nID2" << nID2 << "\t" << mRIDTrans[nID2] << endl;
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
		hh.pif = make_pair(mIDTrans[(*imNL).first], (*imNL).second);
		qh.push(hh);
		vPrevious[mIDTrans[(*imNL).first]] = mIDTrans[ID1];
	}

	pair<double, float> pu;
	while(!qh.empty())
	{
		pu = qh.top().pif;
		qh.pop();
		for(imNL = g.mNode[mRIDTrans[pu.first]].mNeighborLength.begin(); imNL != g.mNode[mRIDTrans[pu.first]].mNeighborLength.end(); imNL++)
		{
			if(vDistance[mIDTrans[(*imNL).first]] == INF && (*imNL).first != ID1)
			{
				float d = vDistance[pu.first] + (*imNL).second;
				vDistance[mIDTrans[(*imNL).first]] = d;
				h hh;
				hh.pif = make_pair(mIDTrans[(*imNL).first], d);
				qh.push(hh);
				vPrevious[mIDTrans[(*imNL).first]] = pu.first;
			}
			else if(vDistance[mIDTrans[(*imNL).first]] > vDistance[pu.first] + (*imNL).second)
			{
				vDistance[mIDTrans[(*imNL).first]] = vDistance[pu.first] + (*imNL).second;
				vPrevious[mIDTrans[(*imNL).first]] = pu.first;
			}
		}
	}
	
	double id = nID2;
	double idtmp;
	vector<double> vRoadListtmp;
	vector<double>::reverse_iterator irvRL;
	cout << "Distance:" << vDistance[nID2] << endl;
	if(vDistance[nID2] != INF)
	{	
		while(id != nID1)
		{
//			cout << id << endl;
			idtmp = vPrevious[id];
			vRoadListtmp.push_back(g.mNode[mRIDTrans[idtmp]].mSubNeighborRoad[mRIDTrans[id]]);
//			cout << g.mNode[mRIDTrans[id]].mNeighborLength[mRIDTrans[idtmp]] << "\t";
//			cout << g.mNode[mRIDTrans[id]].mSubNeighborRoad[mRIDTrans[idtmp]] << endl;

			id = idtmp;
		}
	}

	for(irvRL = vRoadListtmp.rbegin(); irvRL != vRoadListtmp.rend(); irvRL++)
	{
		vRoadList.push_back(*irvRL);
	}

	return vDistance[nID2];
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

