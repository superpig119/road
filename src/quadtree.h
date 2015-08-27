//
//  quadtree.h
//  road
//
//  Created by Thor on 27/08/2015.
//  Copyright (c) 2015 Thor. All rights reserved.
//

#ifndef road_quadtree_h
#define road_quadtree_h

#include <vector>

using namespace std;

typedef struct SIMPLENODE
{
    double x, y;
    vector<int> vRoadList;
}simpleNode;

class Quadtree {
public:
	Quadtree(double x, double y, double width, double height, int level, int maxLevel);
    
    ~Quadtree();
    
    void AddNode(simpleNode node);
    vector<simpleNode>	GetNodeAt(double x, double y);
    void Clear();
    
private:
    double	x;
    double	y;
    double  width;
    double  height;
    int		level;
    int		maxLevel;
    vector<simpleNode> vSimpleNode;
    
    Quadtree	*parent;
    Quadtree	*NW;
    Quadtree 	*NE;
    Quadtree 	*SW;
    Quadtree 	*SE;
    
    bool contains(Quadtree* child, simpleNode node);
};

#endif
