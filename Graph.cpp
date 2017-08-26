//Min Chang
//Github: Minyc510

#include "Graph.h"
#include <algorithm>
#include <iostream>
#include <sstream>
#include <limits> //Simulate infinity
#include <queue>

Graph::Graph() {}

Graph::Graph(bool directed) { this->directed = directed; }

Graph::~Graph() {}

bool Graph::addNode(int data, std::string name) {
  //If node already exists, return false
  if (nodeMap.find(name) != nodeMap.end()) { return false; }

  //Else, Dynamically Allocate a new Node and put it in 'nodeMap'
  Node* newNode = new Node(data, name);
  nodeMap.emplace(name, newNode);

  return true;
}

bool Graph::addNode(std::string name) {
  return addNode(1, name);
}

//Given a vector of (int, string) pairs, insert each pair as a node
void Graph::addNodes(std::vector<std::string> nodes) {
  for (auto node : nodes) {
    addNode(node);
  }
}

//Given a vector of (int, string) pairs, insert each pair as a node
void Graph::addNodes(std::vector<std::pair<int, std::string>> nodes) {
  for (auto nodePair : nodes) {
    addNode(nodePair.first, nodePair.second);
  }
}

bool Graph::addEdge(std::string fromNode, std::string toNode, int weight) {
  //If one of the nodes don't exist, return false
  if (nodeMap.find(fromNode) == nodeMap.end()) { return false; }
  if (nodeMap.find(toNode) == nodeMap.end()) { return false; }

  //Else add neighbor
  nodeMap[fromNode]->addNeighbor(toNode, weight);
  nodeMap[toNode]->getSetRef().insert(fromNode);

  //If the Graph is undirected, also add the "Inverse-Edge"
  if (!directed) {
    nodeMap[toNode]->addNeighbor(fromNode, weight);
    nodeMap[fromNode]->getSetRef().insert(toNode);
  }

  return true;
}

//Default edge weight is 1
bool Graph::addEdge(std::string fromNode, std::string toNode) {
  return addEdge(fromNode, toNode, 1);
}

bool Graph::deleteNode(std::string targetNode) {
  //If node does not exist, return false
  if (nodeMap.find(targetNode) == nodeMap.end()) { return false; }

  //For each Node N in getSetRef(), remove targetNode from N's getMapPtr()
  //getSetRef() will have all Nodes that have an edge to targetNode
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

  //If the Graph is undirected, also delete the "Inverse-Edge"
  if (!directed) {
    //Delete JUST ONE key of 'weight' from multiset
    multiSet = neighborMapRef[fromNode];
    std::multiset<int>::iterator it2(multiSet.find(weight));
    if (it2!=multiSet.end()) { multiSet.erase(it2); }

    //If that was the last edge from toNode to fromNode, delete that (key,value) pair from getMapPtr()
    if (multiSet.empty()) {
      neighborMapRef.erase(fromNode);
    }
  }

  return true;
}

bool Graph::deleteEdge(std::string fromNode, std::string toNode) {
  return deleteEdge(fromNode, toNode, 1);
}

//Returns a list of the names of neighbors
std::vector<std::string> Graph::neighborNames(std::string name) {
  std::vector<std::string> returnVec;

  std::unordered_map<std::string, std::multiset<int>>* mapPtr = nodeMap[name]->getMapPtr();
  for (auto it : *mapPtr) {
    returnVec.push_back(it.first);
  }

  return returnVec;
}

//Returns a list of the names of neighbors along with the lowest edge weight to each neighbor
std::vector<std::pair<std::string, int>> Graph::neighborDistMin(std::string name) {
  std::vector<std::pair<std::string, int>> returnVec;

  std::unordered_map<std::string, std::multiset<int>>* mapPtr = nodeMap[name]->getMapPtr();
  for (auto it : *mapPtr) {
    std::pair<std::string, int> tempPair(it.first, *std::min_element(it.second.begin(),it.second.end()));
    returnVec.push_back(tempPair);
  }

  return returnVec;
}

//Returns a list of the names of neighbors along with the highest edge weight to each neighbor
std::vector<std::pair<std::string, int>> Graph::neighborDistMax(std::string name) {
  std::vector<std::pair<std::string, int>> returnVec;

  std::unordered_map<std::string, std::multiset<int>>* mapPtr = nodeMap[name]->getMapPtr();
  for (auto it : *mapPtr) {
    std::pair<std::string, int> tempPair(it.first, *std::max_element(it.second.begin(),it.second.end()));
    returnVec.push_back(tempPair);
  }

  return returnVec;
}

bool Graph::deleteNeighbors(std::string name) {
  if (nodeMap.find(name) == nodeMap.end()) { return false; }

  std::vector<std::string> neighbors = neighborNames(name);
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

void Graph::exploreHelper(std::set<std::string> &visited, std::string v) {
  visited.insert(v);
  std::vector<std::string> neighbors = neighborNames(v);

  for (auto neighbor : neighbors) {
    if (visited.find(neighbor) == visited.end())
      exploreHelper(visited, neighbor);
  }
}

std::vector<std::string> Graph::reachableNames(std::string name) {
  std::vector<std::string> returnVec;
  std::set<std::string> reachable = explore(name);
  for (std::string name : reachable) {
    returnVec.push_back(name);
  }
  return returnVec;
}

//BFS: Returns vector of Nodes and their distances from targeNode, in order
std::vector<std::pair<std::string, int>> Graph::BFS(std::string targetNode) {
  int infinity = std::numeric_limits<int>::max(); //Simulated infinity
  std::unordered_map<std::string, int> dist; //Holds the shortest distance to each Node from targetNode
  std::vector<std::pair<std::string, int>> returnVec;

  //If targetNode does not exist, return an empty vector
  if (nodeMap.find(targetNode) == nodeMap.end()) { return returnVec; }

  //For all Nodes N, set dist[N] to infinity
  for (auto iter : nodeMap) {
    dist.emplace(iter.first, infinity);
  }

  //BFS
  dist[targetNode] = 0;
  std::queue<std::string> Q;
  Q.push(targetNode);

  while (!Q.empty()) {
    std::string currNode = Q.front();
    Q.pop();
    returnVec.push_back(std::make_pair (currNode, dist[currNode]));
    //For all Neighbors N of currNode
    std::vector<std::string> neighborsCurr = neighborNames(currNode);
    for (auto N : neighborsCurr) {
      if (dist[N] == infinity) {
        Q.push(N);
        dist[N] = dist[currNode] + 1;
      }
    }
  }

  return returnVec;
}

//Djiktras
std::vector<std::pair<std::string, int>> Graph::Dijktras(std::string sourceNode) {
  int infinity = std::numeric_limits<int>::max(); //Simulated infinity
  std::unordered_map<std::string, int> dist; //Holds the shortest distance to each Node from targetNode
  std::unordered_map<std::string, std::string> prev; //Holds the previous node of current node from the source
  std::vector<std::pair<std::string, int>> returnVec;

  if (nodeMap.find(sourceNode) == nodeMap.end()) { return returnVec; }

  //For all Nodes N, set their distance from source to infinity, all prevs are null
  for (auto iter : nodeMap) {
    dist[iter.first] = infinity;
    prev[iter.first] = ""; //Empty string serves as null
  }
  dist[sourceNode] = 0;

  //Min-Heap of Pairs, where .first is the shortest distance from source and .second is the name
  //C++ will use the first value of pair as the comparison
  std::priority_queue<std::pair<int, std::string>,
  std::vector<std::pair<int, std::string>>,
  std::greater<std::pair<int, std::string>> > minHeap;

  for (auto iter : nodeMap) {
    minHeap.push(std::make_pair(dist[iter.first], iter.first));
  }

  //while pQ not empty
  while (!minHeap.empty()) {
    std::string currNode = minHeap.top().second;
    minHeap.pop();

    //for all neighbors N of currNode
    std::vector<std::string> neighborsCurr = neighborNames(currNode);
    for (std::string N : neighborsCurr) {
      std::unordered_map<std::string, std::multiset<int>>* mapPtr = nodeMap[currNode]->getMapPtr();
      //*((*mapPtr())[N]).begin()
      int distanceToN = dist[currNode] + *((*mapPtr)[N]).begin();
      //if dist[N] > dist[curr] + length(curr,N)
      if (dist[N] > distanceToN) {
        //dist [N] = dist[curr] + length(curr,N)
        dist[N] = distanceToN;
        //prev[N] = [curr]
        prev[N] = currNode;
      }
    }
  }

  for (auto iter : dist) {
    returnVec.push_back(std::make_pair (iter.first, iter.second));
  }

  return returnVec;






}

// Temporary Function, useful for debugging.
std::string Graph::getInfo() {
  std::stringstream ss;
  ss << "\n\nGraph Info: " << std::endl;
  //For Every Node
  for (auto iterA : nodeMap) {
    ss << "[" << iterA.first << "] ";
    //For Every Neighbor of Node
    for (auto iterB : *(iterA.second->getMapPtr())) {
      ss << "("<< iterB.first << "): ";
      //Print Each Edge of Neighbor
      for (auto weight : iterB.second) {
        ss << weight << ", ";
      }
    }
    ss << "\n\n";
  }
  return ss.str();
}
