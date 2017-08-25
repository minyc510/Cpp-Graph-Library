//Min Chang
//Github: Minyc510

#include "Graph.h"
#include <iostream>

Graph::Graph() {}
Graph::~Graph() {}

bool Graph::addNode(int data, std::string name) {
  //If node already exists, return false
  if (nodeMap.find(name) != nodeMap.end()) { return false; }

  //Else, Dynamically Allocate a new Node and put it in 'nodeMap'
  Node* newNode = new Node(data, name);
  nodeMap.emplace(name, newNode);

  return true;
}

bool Graph::addEdge(std::string fromNode, std::string toNode, int weight) {
  //If one of the nodes don't exist, return false
  if (nodeMap.find(fromNode) == nodeMap.end()) { return false; }
  if (nodeMap.find(toNode) == nodeMap.end()) { return false; }

  //Else add neighbor
  nodeMap[fromNode]->addNeighbor(toNode, weight);
  nodeMap[toNode]->getSetRef().insert(fromNode);
  return true;
}

//Default edge weight is 1
bool Graph::addEdge(std::string fromNode, std::string toNode) {
  return addEdge(fromNode, toNode, 1);
}

bool Graph::deleteNode(std::string targetNode) {
  //If node does not exist, return false
  if (nodeMap.find(targetNode) == nodeMap.end()) { return false; }

  //For each Node in getSetRef(), remove targetNode from Node's getMapPtr()
  std::set<std::string>& setReference = (nodeMap[targetNode]->getSetRef());
  for (auto iter : setReference) {
    (nodeMap[iter]->getMapPtr())->erase(targetNode);
  }

  //Remove targetNode from it's neighbors "getSetRef()"
  for (auto iter : *(nodeMap[targetNode]->getMapPtr())) {
    nodeMap[iter.first]->getSetRef().erase(targetNode);
  }

  //Deallocate Node, remove it from nodeMap
  delete nodeMap[targetNode];
  nodeMap.erase (targetNode);
  return true;
}

bool Graph::deleteEdge(std::string fromNode, std::string toNode, int weight) {
  //If one of the nodes don't exist or no such edge exists, return false
  if (nodeMap.find(fromNode) == nodeMap.end()) { return false; }
  if (nodeMap.find(toNode) == nodeMap.end()) { return false; }
  std::unordered_map<std::string, std::multiset<int>>& neighborMapRef = *(nodeMap[fromNode]->getMapPtr());
  if (neighborMapRef.find(toNode) == neighborMapRef.end()) { return false; }

  //Delete JUST ONE key of 'weight' from multiset
  std::multiset<int>& multiSet = neighborMapRef[toNode];
  std::multiset<int>::iterator it(multiSet.find(weight));
  if (it!=multiSet.end()) { multiSet.erase(it); }

  //If that was the last edge from fromNode to toNode, delete that (key,value) pair from getMapPtr()
  if (multiSet.empty()) {
    neighborMapRef.erase(toNode);
  }

  return true;
}

std::vector<std::string> Graph::getNeighbors(std::string name) {
  std::vector<std::string> returnVec;

  std::unordered_map<std::string, std::multiset<int>>* mapPtr = nodeMap[name]->getMapPtr();
  for (auto it : *mapPtr) {
    returnVec.push_back(it.first);
  }

  return returnVec;
}

bool Graph::deleteNeighbors(std::string name) {
  if (nodeMap.find(name) == nodeMap.end()) { return false; }

  std::vector<std::string> neighbors = getNeighbors(name);
  for (auto neighbor : neighbors) {
    deleteNode(neighbor);
  }
  return true;
}

std::set<std::string> Graph::explore(std::string name) {
  std::set<std::string> reachable; //Will contain all nodes reachable from the passed Node
  exploreHelper(reachable, name);
  return reachable;
}
//Recursive Function
void Graph::exploreHelper(std::set<std::string> &visited, std::string v) {
  visited.insert(v);
  std::vector<std::string> neighbors = getNeighbors(v);

  for (auto neighbor : neighbors) {
    if (visited.find(neighbor) == visited.end())
      exploreHelper(visited, neighbor);
  }
}

/* Temporary Function, useful for debugging.
void Graph::printInfo() {
  std::cout << "\n\nGraph Info: " << std::endl;
  //For Every Node
  for (auto iterA : nodeMap) {
    std::cout << "[" << iterA.first << "] ";
    //For Every Neighbor of Node
    for (auto iterB : *(iterA.second->getMapPtr())) {
      std::cout << "("<< iterB.first << "): ";
      //Print Each Edge of Neighbor
      for (auto weight : iterB.second) {
        std::cout << weight << ", ";
      }
    }
    std::cout << "\n\n";
  }
}
*/
