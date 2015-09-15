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
	
	readRoad();
	readNode();
//	readTrajectory();
	readSpeed();
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
		g.mNode[n.ID] = n;
	}
	inNodeFile.close();

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
	map<double, int>::iterator imV;
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

int	RoadNetwork::readSpeed()
{
	ifstream inS(conf.speedFilePath.c_str());
	cout << conf.speedFilePath << endl;
	if(!inS)
	{
		cout << "Cannot open speed file" << endl;
		return -1;
	}
	
	int lineNum, sNum, i, j, v;
	double roadID, t;
	stringstream ss;
	inS >> lineNum;
	char* c = "%Y%m%d%H%M%S";
	for(i = 0; i < lineNum; i++)
	{
		inS >> roadID;
		inS >> sNum;
		for(j = 0; j < sNum; j++)
		{
			inS >> t >> v;
			string ct;
			ss.clear();
			ss.str("");
			ss << t;
			ss >> ct;
			g.mRoad[roadID].mV[t] = v;
			struct tm tm;
			strptime(ct.c_str(), c, &tm) ;
//			time_t time = strToTime(ct.c_str(), c, tm);
//			cout << ctime(&time);
			cout << tm.tm_hour << "\t" << tm.tm_min << endl;
		}
	}
	inS.close();
}

void RoadNetwork::testGraph()
{
	map<double, roadInfo>::iterator imRoad;
	vector<pair<double, double> >::iterator ivRD;
	map<double, int>::iterator imV;
	for(imRoad = g.mRoad.begin(); imRoad != g.mRoad.end(); imRoad++)
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

/*	map<int, node>::iterator imNode;
	map<double, float>::iterator imNL;
	map<double, int>::iterator imSN;
	for(imNode = g.mNode.begin(); imNode != g.mNode.end(); imNode++)
	{
		cout << setprecision(15) << (*imNode).second.ID << "\tLenght:";
		for(imNL = (*imNode).second.mNeighborLength.begin(); imNL != (*imNode).second.mNeighborLength.end(); imNL++)
		{
			cout << setprecision(15) << "\t" << (*imNL).first << "," << (*imNL).second;
		}
		cout << "\tsubneighbor road" << endl;
		for(imSN = (*imNode).second.mSubNeighborRoad.begin(); imSN != (*imNode).second.mSubNeighborRoad.end(); imSN++)
		{
			cout << setprecision(15) << "\t" << (*imSN).first << "," << (*imSN).second;
		}
		cout << endl;
	}*/
}
	
void RoadNetwork::organizeSpeed(int gra)
{
	
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

time_t RoadNetwork::strToTime(const char* date,char* format, struct tm &tm)
{
	struct tm t;
	strptime(date,format, &t);
	time_t ft=mktime(&t);
	return ft;
}

