//
//  quadtree.cpp
//  road
//
//  Created by Thor on 27/08/2015.
//  Copyright (c) 2015 Thor. All rights reserved.
//
#include "quadtree.h"

using namespace std;

Quadtree::Quadtree(double _x, double _y, double _width, double _height, int _level, int _maxLevel) :
x		(_x),
y		(_y),
width	(_width),
height	(_height),
level	(_level),
maxLevel(_maxLevel)
{
    if (level == maxLevel)
        return;
    
    NW = new Quadtree(x, y, width / 2.0f, height / 2.0f, level+1, maxLevel);
    NE = new Quadtree(x + width / 2.0f, y, width / 2.0f, height / 2.0f, level+1, maxLevel);
    SW = new Quadtree(x, y + height / 2.0f, width / 2.0f, height / 2.0f, level+1, maxLevel);
    SE = new Quadtree(x + width / 2.0f, y + height / 2.0f, width / 2.0f, height / 2.0f, level+1, maxLevel);
}

Quadtree::~Quadtree()
{
    if (level == maxLevel)
        return;
    
    delete NW;
    delete NE;
    delete SW;
    delete SE;
}

void Quadtree::AddNode(simpleNode *node)
{
    if (level == maxLevel)
    {
        vSimpleNode.push_back(node);
        return;
    }
    
    if (contains(NW, node))
    {
        NW->AddNode(node);
        return;
    }
    else if (contains(NE, node))
    {
        NE->AddNode(node);
        return;
    }
    else if (contains(SW, node))
    {
        SW->AddNode(node);
        return;
    }
    else if (contains(SE, node))
    {
        SE->AddNode(node);
        return;
    }
    if (contains(this, node))
        vSimpleNode.push_back(node);
    
}

vector<simpleNode*> Quadtree::GetNodeAt(double _x, double _y)
{
    if (level == maxLevel)
        return vSimpleNode;
    
    vector<simpleNode*> returnNodes, childReturnNodes;
    if (!vSimpleNode.empty())
    {
        return  returnNodes;
    }
    
    if (_x > x + width / 2.0f && _x < x + width)
    {
        if (_y > y + height / 2.0f && _y < y + height)
        {
            childReturnNodes = SE->GetNodeAt(_x, _y);
            returnNodes.insert(returnNodes.end(), childReturnNodes.begin(), childReturnNodes.end());
            return returnNodes;
        }
        else if (_y > y && _y <= y + height / 2.0f)
        {
            childReturnNodes = NE->GetNodeAt(_x, _y);
            returnNodes.insert(returnNodes.end(), childReturnNodes.begin(), childReturnNodes.end());
            return returnNodes;
        }
    }
    else if (_x > x && _x <= x + width / 2.0f)
    {
        if (_y > y + height / 2.0f && _y < y + height)
        {
            childReturnNodes = SW->GetNodeAt(_x, _y);
            returnNodes.insert(returnNodes.end(), childReturnNodes.begin(), childReturnNodes.end());
            return returnNodes;
        }
        else if (_y > y && _y <= y + height / 2.0f)
        {
            childReturnNodes = NW->GetNodeAt(_x, _y);
            returnNodes.insert(returnNodes.end(), childReturnNodes.begin(), childReturnNodes.end());
            return returnNodes;
        }
    }
    
    return returnNodes;
}

void Quadtree::Clear()
{
    if (level == maxLevel)
    {
        vSimpleNode.clear();
        return;
    }
    else
    {
        NW->Clear();
        NE->Clear();
        SW->Clear();
        SE->Clear();
    }
    
    if (!vSimpleNode.empty())
    {
        vSimpleNode.clear();
    }
}

bool Quadtree::contains(Quadtree *child, simpleNode *node)
{
    return	 !(node->x < child->x ||
               node->y < child->y ||
               node->x > child->x + child->width  ||
               node->y > child->y + child->height);
//               node->x + node->width < child->x ||
//               node->y + node->height < child->y ||
//               node->x + node->width > child->x + child->width ||
//               node->y + node->height > child->y + child->height);
}
