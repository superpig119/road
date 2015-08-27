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

		g.vRoad.push_back(ri);
	}
    cout << setprecision(15) << "minX:" << minX << "\tmaxX:" << maxX << "\tminY:" << minY << "\tmaxY:" << maxY << endl;
    cout << "Total node number:" << count << endl;
	return 0;
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
		qt->AddNode(sn);
	}
		
	cout << "Building QuadTree with Roads" << endl;
	for(ivRoad = g.vRoad.begin(); ivRoad != g.vRoad.end(); ivRoad++)
	{
		for(ivpRD = (*ivRoad).vpRoadDetail.begin();ivpRD != (*ivRoad).vpRoadDetail.end(); ivpRD++)
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
    
void RoadNetwork::matchTrajectory(taxiTrajectory tt)
{
	vector<trajectoryUnit>::iterator ivTU;
    Quadtree * qtt;
    for(ivTU = tt.vTU.begin(); ivTU != tt.vTU.end(); ivTU++)
    {
        qtt = qt->getRegion((*ivTU).x, (*ivTU).y);
        cout << qtt->vSimpleNode.size() << endl;
    }
}
