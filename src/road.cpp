#include "road.h"

#define INF 99999999

typedef struct heap
{
	pair<int, float> pif;
	bool operator < (const struct heap &a) const  
	{  
		return pif.second > a.pif.second; 
	} 
}h;

float RoadNetwork::distanceDijkstra(int ID1, int ID2)
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
	
	cout << setprecision(15) << "ID1:" << ID1 << "\t" << g.vNode[ID1].x << "\t" << g.vNode[ID1].y << endl;
	cout << setprecision(15) << "ID2:" << ID2 << "\t" << g.vNode[ID2].x << "\t" << g.vNode[ID2].y << endl;

	cout << "Path:" << endl;
	int id = ID2;
	int idtmp;
	cout << "Distance:" << vDistance[ID2] << endl;
	if(vDistance[ID2] != INF)
	{	
		while(id != ID1)
		{
			idtmp = vPrevious[id];
			cout << setprecision(15) << idtmp << "\tto\t" << id << "\tDistance:" << vDistance[id] << "\tCoordinate:\t" <<g.vNode[id].x << "\t" << g.vNode[id].y << endl;
			id = idtmp;
		}
	}

	return vDistance[ID2];
}

int RoadNetwork::buildGraph()
{
	Conf conf;
	if(conf.readConf())
	{
		return 1;
	}
	
	ifstream ifile(conf.filepath.c_str());
	cout << conf.filepath << endl;
	if(!ifile)
	{
		cout << "Cannot open data file" << endl;
		return 1;
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
		g.vNode.push_back(n);
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
			}
		}

		g.vRoad.push_back(ri);
	}

	return 0;
}
	
void RoadNetwork::testGraph()
{
	vector<node>::iterator ivNode;
	map<int, float>::iterator imNL;	//ID, length
	vector<pair<double, double> >::iterator ivpRD;

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
	}
}
