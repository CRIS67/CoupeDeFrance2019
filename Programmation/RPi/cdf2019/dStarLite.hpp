#ifndef DSTARTLITE_H_INCLUDED
#define DSTARTLITE_H_INCLUDED

#include <iostream>
#include <map>
#include <vector>
#include <cstdlib>
#include <utility>
#include <algorithm>
#include <cmath>
#include <bits/stdc++.h>
#include <limits>
#include "mapGeneration.hpp"


const double infinity = std::numeric_limits<double>::infinity(); // infinity value

struct Node
{
	float costG; // cost of the path as in A*
	float costRHS; // lookahead value
	float costH; // cost from current to the start (because we start from the goal in D* Lite)
	std::pair<int,int> coord; // coordinates of the node in the map
	bool isObstacle = false; // if the node is an obstacle is set to true
	std::pair<float, float> key {-1,-1}; // default node key

};


typedef std::map< std::pair<float, float>, Node> priorityList; // Each node will be indentified by their key
typedef std::map< std::pair<int, int>, Node> mappedNodes; // all the nodes in the map

// FUNCTION PROTOTYPES
// DStarLite
float distance(int x1, int y1, int x2, int y2);
bool compareKeys(std::pair<float, float> k1, std::pair<float,float> k2);
std::pair<float,float> topKey(priorityList& uList);
Node top(priorityList& uList);
Node pop(priorityList& uList);
void update( std::pair<float,float> oldKey, std::pair<float,float> newKey, priorityList& uList);
float distance2(Node node1, Node node2);

std::pair<float, float> calculateKey(Node node, Node startNode);
void initialize(std::vector<std::vector<int>>& randomMap, mappedNodes& knownNodes, priorityList& uList, Node startNode, Node goalNode);
void updateNode( Node node, priorityList& uList, mappedNodes& knownNodes,std::pair<int,int> startCoord, Node goal);
void computeShortestPath(priorityList& uList, mappedNodes& knownNodes, std::pair<int,int> startCoord, Node goalNode);
float minSuccessor(Node node, mappedNodes& knownNodes);
void updateAdjacents(Node currentNode, priorityList& uList, mappedNodes& knownNodes, std::pair<int,int> startCoord, Node goalNode);
void findPath(std::vector<std::vector<int>>& randomMap, mappedNodes& knownNodes, Node currentNode, Node goalNode);
Node bestNode(Node currentNode ,mappedNodes& knownNodes);
void updateMap(mappedNodes& knownNodes, std::vector<std::vector<int>>& randomMap, priorityList& priorList, std::pair<int,int> startCoord, Node goalNode);
std::vector<Node> getPath(std::vector<std::vector<int>>& randomMap, mappedNodes& knownNodes, Node currentNode, Node goalNode);

// DEBUG
void printKnownNode(mappedNodes& knownNodes);
void printNodesAndKeys(mappedNodes& knownNodes);
#endif // DSTARTLITE_H_INCLUDED
