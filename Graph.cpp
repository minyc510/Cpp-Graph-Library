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
  std::unordered_set<std::string>& setReference = (nodeMap[targetNode]->getSetRef());
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

//Modified DE
bool Graph::deleteEdge(std::string fromNode, std::string toNode, int weight) {
  //If one of the nodes don't exist or no such edge exists, return false
  if (nodeMap.find(fromNode) == nodeMap.end()) { return false; }
  if (nodeMap.find(toNode) == nodeMap.end()) { return false; }
  std::unordered_map<std::string, std::unordered_multiset<int>>& neighborMapRef = *(nodeMap[fromNode]->getMapPtr());
  if (neighborMapRef.find(toNode) == neighborMapRef.end()) { return false; }

  //Delete weight from unordered_multiset
  std::unordered_multiset<int>& set = neighborMapRef[toNode];
  set.erase(weight);

  //If that was the last edge from fromNode to toNode, delete that (key,value) pair from getMapPtr()
  if (set.empty()) {
    neighborMapRef.erase(toNode);
  }

  //If the Graph is undirected, also delete the "Inverse-Edge"
  if (!directed) {
	  std::unordered_map<std::string, std::unordered_multiset<int>>& neighborMapRef1 = *(nodeMap[toNode]->getMapPtr());

	  //Delete weight from unordered_multiset
	  std::unordered_multiset<int>& set1 = neighborMapRef1[fromNode];
	  set1.erase(weight);

	  //If that was the last edge from fromNode to toNode, delete that (key,value) pair from getMapPtr()
	  if (set1.empty()) { neighborMapRef1.erase(fromNode);}
  }

  return true;
}

bool Graph::deleteEdge(std::string fromNode, std::string toNode) {
  return deleteEdge(fromNode, toNode, 1);
}

//NEIGHBORNAMES: Returns a list of the names of neighbors
std::vector<std::string> Graph::neighborNames(std::string name) {
  std::vector<std::string> returnVec;

  std::unordered_map<std::string, std::unordered_multiset<int>>* neighborMapPtr = nodeMap[name]->getMapPtr();
  for (auto it : *neighborMapPtr) {
    returnVec.push_back(it.first);
  }

  return returnVec;
}

//GET EDGES
std::vector< std::tuple<std::string, std::string, int> > Graph::getEdges() {
  std::vector< std::tuple<std::string, std::string, int> > edgeVec;

  //For all Nodes K in nodeMap:
  for (auto iter : nodeMap) {
    auto K = iter.second; //K is a Node*
    //For all neighbors N of K
    for (auto iter1 : *(K->getMapPtr())) {
      auto tempSet = iter1.second; //tempSet is an unordered_multiset
      //For all weights from K to N, add it to the edgeVec
      for (int i : tempSet) {
        std::tuple<std::string, std::string, int> tempTuple(iter.first, iter1.first, i);
        edgeVec.push_back(tempTuple);
      }
    }
  }

  return edgeVec;
}

//NEIGHBORDISTMIN:  Returns a list of the names of neighbors along with the lowest edge weight to each neighbor
std::vector<std::pair<std::string, int>> Graph::neighborDistMin(std::string name) {
  std::vector<std::pair<std::string, int>> returnVec;

  std::unordered_map<std::string, std::unordered_multiset<int>>* neighborMapPtr = nodeMap[name]->getMapPtr();
  for (auto it : *neighborMapPtr) {
    std::pair<std::string, int> tempPair(it.first, *std::min_element(it.second.begin(),it.second.end()));
    returnVec.push_back(tempPair);
  }

  return returnVec;
}

//Returns a list of the names of neighbors along with the highest edge weight to each neighbor
std::vector<std::pair<std::string, int>> Graph::neighborDistMax(std::string name) {
  std::vector<std::pair<std::string, int>> returnVec;

  std::unordered_map<std::string, std::unordered_multiset<int>>* neighborMapPtr = nodeMap[name]->getMapPtr();
  for (auto it : *neighborMapPtr) {
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

//reachableNames: Returns a list of Nodes reachable from a given sourceNode
std::vector<std::string> Graph::reachableNames(std::string sourceNode) {
  std::vector<std::string> returnVec;
  std::set<std::string> reachable = explore(sourceNode);
  for (std::string name : reachable) {
    returnVec.push_back(name);
  }
  return returnVec;
}

//reachableDists: Returns a list of Nodes and their distances from a given sourceNode (uses BFS)
std::vector<std::pair<std::string, int>> Graph::reachableDists(std::string sourceNode) {
  int infinity = std::numeric_limits<int>::max(); //Simulated infinity
  std::unordered_map<std::string, int> dist; //Holds the shortest distance to each Node from sourceNode
  std::vector<std::pair<std::string, int>> returnVec;

  //If sourceNode does not exist, return an empty vector
  if (nodeMap.find(sourceNode) == nodeMap.end()) { return returnVec; }

  //For all Nodes N, set dist[N] to infinity
  for (auto iter : nodeMap) {
    dist.emplace(iter.first, infinity);
  }

  //BFS
  dist[sourceNode] = 0;
  std::queue<std::string> Q;
  Q.push(sourceNode);

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

//connected: Is the Graph connected?
bool Graph::connected() {
  if (nodeMap.empty()) { return true;} //An empty Graph is trivially connected
  //Run explore on a random Node
  auto it =  nodeMap.begin();
  std::set<std::string> tempSet = explore(it->first);
  //Is the set of Nodes reachable == # of all Nodes in the Graph?
  return (tempSet.size() == nodeMap.size());
}

//BFS
std::vector<std::string> Graph::BFS(std::string sourceNode, std::string targetNode) {
  //If either Node DNE, return an empty vector
  std::vector<std::string> pathVec;
  if (nodeMap.find(sourceNode) == nodeMap.end()) { return pathVec; }
  if (nodeMap.find(targetNode) == nodeMap.end()) { return pathVec; }

  //prevMap[X] will contain the Node previous to X. Also keeps track of which Nodes have been visited.
  std::unordered_map<std::string, std::string> prevMap;
  prevMap.emplace(sourceNode, "");

  //BFS
  std::queue<std::string> Q;
  Q.push(sourceNode);

  while (!Q.empty()) {
    std::string currNode = Q.front();
    Q.pop();
    //For all Neighbors N of currNode
    std::vector<std::string> neighborsCurr = neighborNames(currNode);
    for (std::string N : neighborsCurr) {
      if (prevMap.find(N) == prevMap.end()) {
        Q.push(N);
        prevMap.emplace(N, currNode);
      }
    }
  }

  //If the targetNode was not found return an empty vector
  if (prevMap.find(targetNode) == prevMap.end()) { return pathVec; }

  //Use prevMap to get the path from Target back to Source
  std::string curr = targetNode;
  pathVec.push_back(curr);
  while (true) {
    curr = prevMap[curr];
    if (curr == "") { break; }
    pathVec.push_back(curr);
  }

  //Reverse pathVec so the Node's are in order from Source to Target
  std::reverse(pathVec.begin(), pathVec.end());

  return pathVec;
}

//DFS - Returns the path from sourceNode to targetNode
std::vector<std::string> Graph::DFS(std::string sourceNode, std::string targetNode) {
  //If either Node DNE, return an empty vector
  std::vector<std::string> pathVec;
  if (nodeMap.find(sourceNode) == nodeMap.end()) { return pathVec; }
  if (nodeMap.find(targetNode) == nodeMap.end()) { return pathVec; }

  //prevMap[X] will contain the Node previous to X. Also keeps track of which Nodes have been visited.
  std::unordered_map<std::string, std::string> prevMap;
  prevMap.emplace(sourceNode, "");

  //Recursive Kick-Off
  DFShelper(sourceNode, targetNode, prevMap);

  //If the targetNode was not found return an empty vector
  if (prevMap.find(targetNode) == prevMap.end()) { return pathVec; }

  //Use prevMap to get the path from Target back to Source
  std::string curr = targetNode;
  pathVec.push_back(curr);
  while (true) {
    curr = prevMap[curr];
    if (curr == "") { break; }
    pathVec.push_back(curr);
  }

  //Reverse pathVec so the Node's are in order from Source to Target
  std::reverse(pathVec.begin(), pathVec.end());

  return pathVec;
}

//DFS - Recursive Function, modifies prevMap
void Graph::DFShelper(std::string currentNode, std::string targetNode, std::unordered_map<std::string, std::string> &prevMap) {
  if (currentNode == targetNode) { return; }

  std::vector<std::string> neighbors = neighborNames(currentNode);
  for (std::string neighbor : neighbors) {
    //If this neighbor has not been visited, add it to the prevMap and recurse on it
    if (prevMap.find(neighbor) == prevMap.end()) {
      prevMap.emplace(neighbor, currentNode);
      DFShelper(neighbor, targetNode, prevMap);
    }
  }
}

//Dijktras
std::vector<std::string> Graph::Dijktras(std::string sourceNode, std::string targetNode) {
  int infinity = std::numeric_limits<int>::max(); //Simulated infinity
  std::unordered_map<std::string, int> dist; //Holds the shortest distance to each Node from targetNode
  std::unordered_map<std::string, std::string> prevMap; //Holds the previous node of current node from the source
  std::vector<std::string> pathVec;

  if (nodeMap.find(sourceNode) == nodeMap.end()) { return pathVec; }

  //For all Nodes N, set their distance from source to infinity, all prevs are null
  for (auto iter : nodeMap) {
    dist[iter.first] = infinity;
    prevMap[iter.first] = ""; //Empty string serves as null
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
      std::unordered_map<std::string, std::unordered_multiset<int>>* neighborMapPtr = nodeMap[currNode]->getMapPtr();
      int distanceToN = dist[currNode] + *((*neighborMapPtr)[N]).begin();
      if (dist[N] > distanceToN) {
        dist[N] = distanceToN;
        prevMap[N] = currNode;
      }
    }
  }

  //Use prevMap to get the path from Target back to Source
  std::string curr = targetNode;
  pathVec.push_back(curr);
  while (true) {
    curr = prevMap[curr];
    if (curr == "") { break; }
    pathVec.push_back(curr);
  }

  //Reverse pathVec so the Node's are in order from Source to Target
  std::reverse(pathVec.begin(), pathVec.end());

  return pathVec;
}

//Djiktras
std::unordered_map<std::string, int> Graph::Dijktras(std::string sourceNode) {
  int infinity = std::numeric_limits<int>::max(); //Simulated infinity
  std::unordered_map<std::string, int> dist; //Holds the shortest distance to each Node from targetNode
  std::unordered_map<std::string, std::string> prev; //Holds the previous node of current node from the source
  std::unordered_map<std::string, int> returnMap; //Holds the distance to all nodes reachable from sourceNode

  if (nodeMap.find(sourceNode) == nodeMap.end()) { return returnMap; }

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
      std::unordered_map<std::string, std::unordered_multiset<int>>* neighborMapPtr = nodeMap[currNode]->getMapPtr();
      int distanceToN = dist[currNode] + *((*neighborMapPtr)[N]).begin();
      if (dist[N] > distanceToN) {
        dist[N] = distanceToN;
        prev[N] = currNode;
      }
    }
  }

  for (auto iter : dist) {
    if (iter.second != infinity)
      returnMap.emplace(iter.first, iter.second);
  }
  return returnMap;
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
