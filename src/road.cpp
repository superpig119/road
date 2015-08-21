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
	map<int, float> mDistance;
	priority_queue<h> qh;
	for(int i = 0; i < g.vNode.size(); i++)
	{
		if(i == ID1)
		{
			mDistance[i] = 0;
			continue;
		}

		if(g.vNode[ID1].mNeighborLength.find(i) != g.vNode[ID1].mNeighborLength.end())
		{
			mDistance[i] = g.vNode[ID1].mNeighborLength[i];
			h hh;
			hh.pif = make_pair(i, g.vNode[ID1].mNeighborLength[i]);
			qh.push(hh);
			cout << i << "\t" << g.vNode[ID1].mNeighborLength[i] << endl;
		}
	}

	pair<int, float> pu;
	map<int, float>::iterator imNL;
	while(!qh.empty())
	{
		pu = qh.top().pif;
		qh.pop();
		for(imNL = g.vNode[pu.first].mNeighborLength.begin(); imNL != g.vNode[pu.first].mNeighborLength.end(); imNL++)
		{
			if(mDistance.find((*imNL).first) == mDistance.end())
			{
				float d = mDistance[pu.first] + (*imNL).second;
				mDistance[(*imNL).first] = d;
				h hh;
				hh.pif = make_pair((*imNL).first, d);
				qh.push(hh);
			}
			else if(mDistance[(*imNL).first] > mDistance[pu.first] + (*imNL).second)
			{
				mDistance[(*imNL).first] = mDistance[pu.first] + (*imNL).second;
			}
		}
	}
	
	map<int, float>::iterator imD;
	for(imD = mDistance.begin(); imD != mDistance.end(); imD++)
	{
		if((*imD).second < 1000)
		cout << (*imD).first << "\tDistance:" << (*imD).second << endl;
	}
	
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
