// Min Chang
// Github: Minyc510

#include "Graph.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>  //Simulate infinity
#include <queue>
#include <sstream>

Graph::Graph() {}

Graph::Graph(bool directed) { this->directed = directed; }

/// Copy Constructor
Graph::Graph(const Graph& original) {
  // Copy over boolean's
  directed = original.directed;

  // Add all nodes in original to new Graph
  for (const auto& iter : original.nodeMap_) {
    int data = iter.second->getData();
    std::string name = iter.first;

    Node* newNode = new Node(data, name);
    nodeMap_.emplace(name, newNode);
  }

  // Add all edges in original to new Graph
  std::vector<std::tuple<std::string, std::string, double>> edgeVec =
      original.getEdges();
  for (const auto& edge : edgeVec) {
    std::string nodeA = std::get<0>(edge);
    std::string nodeB = std::get<1>(edge);
    double weight = std::get<2>(edge);

    this->addEdge(nodeA, nodeB, weight);
  }
}

/// Construct from File - When calling need to cast to string ie Graph
/// G(string("file.txt"));
Graph::Graph(const std::string& inputFileName) {
  // Open .txt file
  std::ifstream file(inputFileName);
  char specialChar = '%';
  char separator = '^';
  std::string line;

  // If the file is invalid, stop.
  if (!file.is_open()) {
    return;
  }

  // Read Header
  getline(file, line);
  if (line ==
      specialChar +
          std::string("PERSISTANT GRAPH: DIRECTED (Do not edit this line)")) {
    directed = true;
  } else if (line ==
             specialChar +
                 std::string(
                     "PERSISTANT GRAPH: UNDIRECTED (Do not edit this line)")) {
    directed = false;
  } else {
    return;
  }  // Corrupt File

  getline(file, line);
  if (line != "---------------------------------------------------------") {
    return;
  }  // Corrupt File

  // Read Node Header
  getline(file, line);
  if (line != specialChar + std::string("NODES (Do not edit this line):")) {
    return;
  }  // Corrupt File

  // Read Nodes
  getline(file, line);
  while (line[0] != specialChar) {
    // Split up Node name and Node data using the separator character
    std::string nodeName = line.substr(0, line.find(separator));
    std::string dataString = line.substr(line.find(separator) + 1);
    double nodeData = std::stod(dataString);

    // Add Node to Graph, read next line
    addNode(nodeData, nodeName);
    getline(file, line);
  }

  // Read Edges
  if (line != specialChar + std::string("EDGES (Do not edit this line):")) {
    return;
  }  // Corrupt File
  while (getline(file, line)) {
    // Split up Edge into sourceNode, targetNode, and weight
    std::string sourceNode = line.substr(0, line.find(separator));
    line = line.substr(line.find(separator) + 1);
    std::string targetNode = line.substr(0, line.find(separator));
    std::string weightString = line.substr(line.find(separator) + 1);
    double weight = std::stod(weightString);

    std::cout << sourceNode << " " << targetNode << " " << weight << std::endl;

    // Add Edge to Graph
    addEdge(sourceNode, targetNode, weight);
  }
}

Graph::~Graph() {
  for (auto iter : nodeMap_) {
    delete iter.second;
  }
}

bool Graph::addNode(double data, const std::string& name) {
  // If node already exists, return false
  if (nodeMap_.find(name) != nodeMap_.end()) {
    return false;
  }

  // Else, Dynamically Allocate a new Node and put it in 'nodeMap'
  Node* newNode = new Node(data, name);
  nodeMap_.emplace(name, newNode);

  return true;
}

bool Graph::addNode(const std::string& name) { return addNode(1, name); }

/// Given a vector of strings, insert each string as a Node
void Graph::addNodes(const std::vector<std::string>& nodes) {
  for (const auto& node : nodes) {
    addNode(node);
  }
}

/// Given a vector of (double, string) pairs, insert each pair as a Node
void Graph::addNodes(const std::vector<std::pair<double, std::string>>& nodes) {
  for (const auto& nodePair : nodes) {
    addNode(nodePair.first, nodePair.second);
  }
}

bool Graph::addEdge(const std::string& fromNode, const std::string& toNode,
                    double weight) {
  // If one of the nodes don't exist, return false
  if (nodeMap_.find(fromNode) == nodeMap_.end()) {
    return false;
  }
  if (nodeMap_.find(toNode) == nodeMap_.end()) {
    return false;
  }

  // Else add neighbor
  nodeMap_[fromNode]->addNeighbor(toNode, weight);
  nodeMap_[toNode]->getSetRef().insert(fromNode);

  // If the Graph is undirected, also add the "Inverse-Edge"
  if (!directed) {
    nodeMap_[toNode]->addNeighbor(fromNode, weight);
    nodeMap_[fromNode]->getSetRef().insert(toNode);
  }

  return true;
}

/// Default edge weight is 1
bool Graph::addEdge(const std::string& fromNode, const std::string& toNode) {
  return addEdge(fromNode, toNode, 1.0);
}

/// Add Edge using a 3-tuple (nodeA,nodeB,weight)
bool Graph::addEdge(const std::tuple<std::string, std::string, double>& edge) {
  return addEdge(std::get<0>(edge), std::get<1>(edge), std::get<2>(edge));
}

bool Graph::deleteNode(const std::string& targetNode) {
  // If node does not exist, return false
  if (nodeMap_.find(targetNode) == nodeMap_.end()) {
    return false;
  }

  // For each Node N in getSetRef(), remove targetNode from N's getMapPtr()
  // getSetRef() will have all Nodes that have an edge to targetNode
  std::unordered_set<std::string>& setReference =
      (nodeMap_[targetNode]->getSetRef());
  for (auto iter : setReference) {
    (nodeMap_[iter]->getMapPtr())->erase(targetNode);
  }

  // Remove targetNode from it's neighbors "getSetRef()"
  for (auto iter : *(nodeMap_[targetNode]->getMapPtr())) {
    nodeMap_[iter.first]->getSetRef().erase(targetNode);
  }

  // Deallocate Node, remove it from nodeMap
  delete nodeMap_[targetNode];
  nodeMap_.erase(targetNode);
  return true;
}

bool Graph::deleteEdge(const std::string& fromNode, const std::string& toNode,
                       double weight) {
  // If one of the nodes don't exist or no such edge exists, return false
  if (nodeMap_.find(fromNode) == nodeMap_.end()) {
    return false;
  }
  if (nodeMap_.find(toNode) == nodeMap_.end()) {
    return false;
  }
  std::unordered_map<std::string, std::multiset<double>>& neighborMapRef =
      *(nodeMap_[fromNode]->getMapPtr());
  if (neighborMapRef.find(toNode) == neighborMapRef.end()) {
    return false;
  }

  // Delete weight from multiset
  std::multiset<double>& set = neighborMapRef[toNode];
  set.erase(weight);

  // If that was the last edge from fromNode to toNode, delete that (key,value)
  // pair from getMapPtr()
  if (set.empty()) {
    neighborMapRef.erase(toNode);
  }

  // If the Graph is undirected, also delete the "Inverse-Edge"
  if (!directed) {
    std::unordered_map<std::string, std::multiset<double>>& neighborMapRef1 =
        *(nodeMap_[toNode]->getMapPtr());

    // Delete weight from multiset
    std::multiset<double>& set1 = neighborMapRef1[fromNode];
    set1.erase(weight);

    // If that was the last edge from fromNode to toNode, delete that
    // (key,value) pair from getMapPtr()
    if (set1.empty()) {
      neighborMapRef1.erase(fromNode);
    }
  }

  return true;
}

bool Graph::deleteEdge(const std::string& fromNode, const std::string& toNode) {
  return deleteEdge(fromNode, toNode, 1.0);
}

/// connected: Returns true if the Graph is connected, for undirected Graphs.
bool Graph::connected() const {
  if (nodeMap_.empty()) {
    return true;
  }  // An empty Graph is trivially connected

  // Run explore on a random Node
  auto it = nodeMap_.begin();
  std::unordered_set<std::string> tempSet = explore(it->first);
  // Is the set of Nodes reachable == # of all Nodes in the Graph?
  return (tempSet.size() == nodeMap_.size());
}

/// weaklyConnected: Returns true if the Graph is weakly-connected, for directed
/// Graphs.
// A directed graph is called weakly connected if replacing all of its
// directed edges with undirected edges produces a connected (undirected) graph.
bool Graph::weaklyConnected() const {
  if (nodeMap_.empty()) {
    return true;
  }  // An empty Graph is trivially connected

  // Create a copy of this graph
  Graph modifiedCopy(*this);
  // Replace all directed edges with undirected edges (ie for all edges <A,B,w>
  // add <B,A,w>)
  std::vector<std::tuple<std::string, std::string, double>> edgeVec =
      modifiedCopy.getEdges();
  for (auto edge : edgeVec) {
    std::string nodeA = std::get<0>(edge);
    std::string nodeB = std::get<1>(edge);
    double weight = std::get<2>(edge);
    modifiedCopy.addEdge(nodeB, nodeA, weight);
  }

  // Test if the modified copy is connected
  return modifiedCopy.connected();
}

/// stronglyConnected: Returns true if the Graph is strongly-connected, for
/// directed Graphs.
// A directed graph is called strongly connected if
// there is a path in each direction between each pair of vertices of the graph.
bool Graph::stronglyConnected() const {
  // DFS on arbitraryNode. If arbitraryNode can't reach all other Nodes, return
  // false.
  std::string arbitraryNode = nodeMap_.begin()->first;
  std::unordered_set<std::string> tempSet = explore(arbitraryNode);
  if (tempSet.size() != nodeMap_.size()) {
    return false;
  }
  // DFS on same arbitraryNode on the transpose of the Graph. If it can reach
  // all other Nodes, the Graph is stronglyConnected.
  Graph T = transpose();
  std::unordered_set<std::string> tempSet1 = T.explore(arbitraryNode);
  std::cout << "***" << tempSet1.size() << std::endl;
  return (tempSet1.size() == nodeMap_.size());
}

/// transpose: Returns a Graph object with reversed edges of the original Graph.
Graph Graph::transpose() const {
  // Create a new Graph object.
  Graph graph(directed);

  // Add all existing nodes to the new Graph
  for (auto iter : nodeMap_) {
    double data = iter.second->getData();
    graph.addNode(data, iter.first);
  }

  // For all edges A,B,w in the original, add B,A,w to the copy
  std::vector<std::tuple<std::string, std::string, double>> edgeVec =
      this->getEdges();
  for (auto edge : edgeVec) {
    std::string nodeA = std::get<0>(edge);
    std::string nodeB = std::get<1>(edge);
    double weight = std::get<2>(edge);

    graph.addEdge(nodeB, nodeA, weight);
  }

  return graph;
}

/// neighborNames: Returns a list of the names of neighbors
std::vector<std::string> Graph::neighborNames(
    const std::string& sourceNode) const {
  std::vector<std::string> returnVec;

  auto it = nodeMap_.find(sourceNode);
  if (it == nodeMap_.end()) {
    return returnVec;
  }
  std::unordered_map<std::string, std::multiset<double>>* neighborMapPtr =
      (it->second)->getMapPtr();
  for (auto it : *neighborMapPtr) {
    returnVec.push_back(it.first);
  }

  return returnVec;
}

/// neighborDistMin: Returns a list of the names of neighbors along with the
/// lowest edge weight to each neighbor
std::vector<std::pair<std::string, double>> Graph::neighborDistMin(
    const std::string& sourceNode) const {
  std::vector<std::pair<std::string, double>> returnVec;
  auto it = nodeMap_.find(sourceNode);
  if (it == nodeMap_.end()) {
    return returnVec;
  }

  std::unordered_map<std::string, std::multiset<double>>* neighborMapPtr =
      (it->second)->getMapPtr();
  for (auto it : *neighborMapPtr) {
    std::pair<std::string, double> tempPair(
        it.first, *std::min_element(it.second.begin(), it.second.end()));
    returnVec.push_back(tempPair);
  }

  return returnVec;
}

/// neighborDistMax: Returns a list of the names of neighbors along with the
/// highest edge weight to each neighbor
std::vector<std::pair<std::string, double>> Graph::neighborDistMax(
    const std::string& sourceNode) const {
  std::vector<std::pair<std::string, double>> returnVec;
  auto it = nodeMap_.find(sourceNode);
  if (it == nodeMap_.end()) {
    return returnVec;
  }

  std::unordered_map<std::string, std::multiset<double>>* neighborMapPtr =
      (it->second)->getMapPtr();
  for (auto it : *neighborMapPtr) {
    std::pair<std::string, double> tempPair(
        it.first, *std::max_element(it.second.begin(), it.second.end()));
    returnVec.push_back(tempPair);
  }

  return returnVec;
}

/// deleteNeighbors: Removes all neighbors of sourceNode along with all the
/// edges associated with the neighbors.
bool Graph::deleteNeighbors(const std::string& sourceNode) {
  if (nodeMap_.find(sourceNode) == nodeMap_.end()) {
    return false;
  }

  std::vector<std::string> neighbors = neighborNames(sourceNode);
  for (auto neighbor : neighbors) {
    deleteNode(neighbor);
  }
  return true;
}

/// explore: Returns a set of Nodes reachable from sourceNode
std::unordered_set<std::string> Graph::explore(
    const std::string& sourceNode) const {
  // Will contain all nodes reachable from the passed Node
  std::unordered_set<std::string> reachable;
  exploreHelper(reachable, sourceNode);
  return reachable;
}

void Graph::exploreHelper(std::unordered_set<std::string>& visited,
                          const std::string& v) const {
  visited.insert(v);
  std::vector<std::string> neighbors = neighborNames(v);

  for (auto neighbor : neighbors) {
    if (visited.find(neighbor) == visited.end())
      exploreHelper(visited, neighbor);
  }
}

/// reachableNames: Returns a list of Nodes reachable from a given sourceNode
std::vector<std::string> Graph::reachableNames(
    const std::string& sourceNode) const {
  std::vector<std::string> returnVec;
  std::unordered_set<std::string> reachable = explore(sourceNode);
  for (std::string name : reachable) {
    returnVec.push_back(name);
  }
  return returnVec;
}

/// reachableDists: Returns a list of Nodes and their distances from a given
/// sourceNode (uses BFS)
std::vector<std::pair<std::string, double>> Graph::reachableDists(
    const std::string& sourceNode) const {
  double infinity = std::numeric_limits<double>::max();  // Simulated infinity
  std::unordered_map<std::string, double>
      dist;  // Holds the shortest distance to each Node from sourceNode
  std::vector<std::pair<std::string, double>> returnVec;

  // If sourceNode does not exist, return an empty vector
  if (nodeMap_.find(sourceNode) == nodeMap_.end()) {
    return returnVec;
  }

  // For all Nodes N, set dist[N] to infinity
  for (const auto& iter : nodeMap_) {
    dist.emplace(iter.first, infinity);
  }

  // BFS
  dist[sourceNode] = 0;
  std::queue<std::string> Q;
  Q.push(sourceNode);

  while (!Q.empty()) {
    std::string currNode = Q.front();
    Q.pop();
    returnVec.push_back(std::make_pair(currNode, dist[currNode]));
    // For all Neighbors N of currNode
    std::vector<std::string> neighborsCurr = neighborNames(currNode);
    for (const auto& N : neighborsCurr) {
      if (dist[N] == infinity) {
        Q.push(N);
        dist[N] = dist[currNode] + 1;
      }
    }
  }

  return returnVec;
}

/// pathCheck: Returns true if there is a (directed) path from fromNode to
/// toNode.
bool Graph::pathCheck(const std::string& sourceNode,
                      const std::string& targetNode) const {
  std::unordered_set<std::string> reachable = explore(sourceNode);
  return (reachable.find(targetNode) != reachable.end());
}

/// BFS: Returns the shortest unweighted path from sourceNode to targetNode
std::vector<std::string> Graph::BFS(const std::string& sourceNode,
                                    const std::string& targetNode) const {
  // If either Node DNE, return an empty vector
  std::vector<std::string> pathVec;
  if (nodeMap_.find(sourceNode) == nodeMap_.end()) {
    return pathVec;
  }
  if (nodeMap_.find(targetNode) == nodeMap_.end()) {
    return pathVec;
  }

  // prevMap[X] will contain the Node previous to X. Also keeps track of which
  // Nodes have been visited.
  std::unordered_map<std::string, std::string> prevMap;
  prevMap.emplace(sourceNode, "");

  // BFS
  std::queue<std::string> Q;
  Q.push(sourceNode);

  while (!Q.empty()) {
    std::string currNode = Q.front();
    Q.pop();
    // For all Neighbors N of currNode
    std::vector<std::string> neighborsCurr = neighborNames(currNode);
    for (std::string N : neighborsCurr) {
      if (prevMap.find(N) == prevMap.end()) {
        Q.push(N);
        prevMap.emplace(N, currNode);
      }
    }
  }

  // If the targetNode was not found return an empty vector
  if (prevMap.find(targetNode) == prevMap.end()) {
    return pathVec;
  }

  // Use prevMap to get the path from Target back to Source
  std::string curr = targetNode;
  pathVec.push_back(curr);
  while (true) {
    curr = prevMap[curr];
    if (curr == "") {
      break;
    }
    pathVec.push_back(curr);
  }

  // Reverse pathVec so the Node's are in order from Source to Target
  std::reverse(pathVec.begin(), pathVec.end());

  return pathVec;
}

/// DFS: Returns the shortest unweighted path from sourceNode to targetNode
std::vector<std::string> Graph::DFS(const std::string& sourceNode,
                                    const std::string& targetNode) const {
  // If either Node DNE, return an empty vector
  std::vector<std::string> pathVec;
  if (nodeMap_.find(sourceNode) == nodeMap_.end()) {
    return pathVec;
  }
  if (nodeMap_.find(targetNode) == nodeMap_.end()) {
    return pathVec;
  }

  // prevMap[X] will contain the Node previous to X. Also keeps track of which
  // Nodes have been visited.
  std::unordered_map<std::string, std::string> prevMap;
  prevMap.emplace(sourceNode, "");

  // Recursive Kick-Off
  DFShelper(sourceNode, targetNode, prevMap);

  // If the targetNode was not found return an empty vector
  if (prevMap.find(targetNode) == prevMap.end()) {
    return pathVec;
  }

  // Use prevMap to get the path from Target back to Source
  std::string curr = targetNode;
  pathVec.push_back(curr);
  while (true) {
    curr = prevMap[curr];
    if (curr == "") {
      break;
    }
    pathVec.push_back(curr);
  }

  // Reverse pathVec so the Node's are in order from Source to Target
  std::reverse(pathVec.begin(), pathVec.end());

  return pathVec;
}

/// DFS - Recursive Function, modifies prevMap
void Graph::DFShelper(
    const std::string& currentNode, const std::string& targetNode,
    std::unordered_map<std::string, std::string>& prevMap) const {
  if (currentNode == targetNode) {
    return;
  }

  std::vector<std::string> neighbors = neighborNames(currentNode);
  for (std::string neighbor : neighbors) {
    // If this neighbor has not been visited, add it to the prevMap and recurse
    // on it
    if (prevMap.find(neighbor) == prevMap.end()) {
      prevMap.emplace(neighbor, currentNode);
      DFShelper(neighbor, targetNode, prevMap);
    }
  }
}

/// Dijktras: Returns the shorted weighted path from sourceNode to targetNode
std::vector<std::string> Graph::Dijktras(const std::string& sourceNode,
                                         const std::string& targetNode) const {
  double infinity = std::numeric_limits<double>::max();  // Simulated infinity
  std::unordered_map<std::string, double>
      dist;  // Holds the shortest distance to each Node from targetNode
  std::unordered_map<std::string, std::string>
      prevMap;  // Holds the previous node of current node from the source
  std::vector<std::string> pathVec;

  if (nodeMap_.find(sourceNode) == nodeMap_.end()) {
    return pathVec;
  }

  // For all Nodes N, set their distance from source to infinity, all prevs are
  // null
  for (const auto& iter : nodeMap_) {
    dist[iter.first] = infinity;
    prevMap[iter.first] = "";  // Empty string serves as null
  }
  dist[sourceNode] = 0;

  // Min-Heap of Pairs, where .first is the shortest distance from source and
  // .second is the name C++ will use the first value of pair as the comparison
  std::priority_queue<std::pair<double, std::string>,
                      std::vector<std::pair<double, std::string>>,
                      std::greater<std::pair<double, std::string>>>
      minHeap;

  for (const auto& iter : nodeMap_) {
    minHeap.push(std::make_pair(dist[iter.first], iter.first));
  }

  // while pQ not empty
  while (!minHeap.empty()) {
    std::string currNode = minHeap.top().second;
    minHeap.pop();

    // for all neighbors N of currNode
    std::vector<std::string> neighborsCurr = neighborNames(currNode);
    for (const std::string& N : neighborsCurr) {
      std::unordered_map<std::string, std::multiset<double>>* neighborMapPtr =
          nodeMap_.at(currNode)->getMapPtr();
      double distanceToN = dist[currNode] + *((*neighborMapPtr)[N]).begin();
      if (dist[N] > distanceToN) {
        dist[N] = distanceToN;
        prevMap[N] = currNode;
      }
    }
  }

  // Use prevMap to get the path from Target back to Source
  std::string curr = targetNode;
  pathVec.push_back(curr);
  while (true) {
    curr = prevMap[curr];
    if (curr == "") {
      break;
    }
    pathVec.push_back(curr);
  }

  // Reverse pathVec so the Node's are in order from Source to Target
  std::reverse(pathVec.begin(), pathVec.end());

  return pathVec;
}

/// Djiktras: Returns an unordered_map where keys are Node names and values are
/// the shortest weighted distance to that Node from sourceNode
std::unordered_map<std::string, double> Graph::Dijktras(
    const std::string& sourceNode) const {
  double infinity = std::numeric_limits<double>::max();  // Simulated infinity
  std::unordered_map<std::string, double>
      dist;  // Holds the shortest distance to each Node from targetNode
  std::unordered_map<std::string, std::string>
      prev;  // Holds the previous node of current node from the source
  std::unordered_map<std::string, double>
      returnMap;  // Holds the distance to all nodes reachable from sourceNode

  if (nodeMap_.find(sourceNode) == nodeMap_.end()) {
    return returnMap;
  }

  // For all Nodes N, set their distance from source to infinity, all prevs are
  // null
  for (const auto iter : nodeMap_) {
    dist[iter.first] = infinity;
    prev[iter.first] = "";  // Empty string serves as null
  }
  dist[sourceNode] = 0;

  // Min-Heap of Pairs, where .first is the shortest distance from source and
  // .second is the name C++ will use the first value of pair as the comparison
  std::priority_queue<std::pair<double, std::string>,
                      std::vector<std::pair<double, std::string>>,
                      std::greater<std::pair<double, std::string>>>
      minHeap;

  for (auto iter : nodeMap_) {
    minHeap.push(std::make_pair(dist[iter.first], iter.first));
  }

  // while pQ not empty
  while (!minHeap.empty()) {
    std::string currNode = minHeap.top().second;
    minHeap.pop();

    // for all neighbors N of currNode
    std::vector<std::string> neighborsCurr = neighborNames(currNode);
    for (const std::string& N : neighborsCurr) {
      std::unordered_map<std::string, std::multiset<double>>* neighborMapPtr =
          nodeMap_.at(currNode)->getMapPtr();
      double distanceToN = dist[currNode] + *((*neighborMapPtr)[N]).begin();
      if (dist[N] > distanceToN) {
        dist[N] = distanceToN;
        prev[N] = currNode;
      }
    }
  }

  for (auto iter : dist) {
    if (iter.second != infinity) returnMap.emplace(iter.first, iter.second);
  }
  return returnMap;
}

/// BellmanFord: Returns a map where keys are Node names and values are the
/// shortest distance from sourceNode
std::tuple<std::unordered_map<std::string, double>,
           std::unordered_map<std::string, std::string>, bool>
Graph::BellmanFord(const std::string& sourceNode) const {
  double infinity = std::numeric_limits<double>::max();  // Simulated infinity
  std::vector<std::tuple<std::string, std::string, double>> Edges = getEdges();
  bool negativeCycle = false;

  // Initialize Dist & Prev maps
  std::unordered_map<std::string, double>
      Dist;  // Holds the shortest distance to each Node from sourceNode
  std::unordered_map<std::string, std::string> Prev;  // Holds the previous Node
  for (auto iter : nodeMap_) {
    Dist.emplace(iter.first, infinity);
    Prev.emplace(iter.first, "");
  }
  Dist[sourceNode] = 0;

  // Repeatedly "Relax" Edges
  for (int i = 1; i <= numNodes() - 1; i++) {
    for (auto edge : Edges) {
      std::string nodeA = std::get<0>(edge);
      std::string nodeB = std::get<1>(edge);
      double weight = std::get<2>(edge);
      if (Dist[nodeA] == infinity) {
        continue;
      }  // infinity + weight will overflow so this guards against that
      if (Dist[nodeA] + weight < Dist[nodeB]) {
        Dist[nodeB] = Dist[nodeA] + weight;
        Prev[nodeB] = nodeA;
      }
    }
  }

  // Check for Negative Cycles
  for (auto edge : Edges) {
    std::string nodeA = std::get<0>(edge);
    std::string nodeB = std::get<1>(edge);
    double weight = std::get<2>(edge);
    if (Dist[nodeA] == infinity) {
      continue;
    }  // infinity + weight will overflow so this guards against that
    if (Dist[nodeA] + weight < Dist[nodeB]) {
      // Negative Cycle Detected:
      Prev[nodeA] = nodeB;
      negativeCycle = true;
    }
  }

  // Return
  return std::make_tuple(Dist, Prev, negativeCycle);
}

std::unordered_map<std::string, double> Graph::BellmanFordDist(
    const std::string& sourceNode) const {
  return std::get<0>(BellmanFord(sourceNode));
}
std::unordered_map<std::string, std::string> Graph::BellmanFordPrev(
    const std::string& sourceNode) const {
  return std::get<1>(BellmanFord(sourceNode));
}
bool Graph::NegativeCycle() const {
  // Warning! Very inefficient, runs BellmanFord using every Node as a source
  // until a negCycle is detected or none at all.
  for (auto iter : nodeMap_) {
    if (std::get<2>(BellmanFord(iter.first))) {
      return true;
    }
  }
  return false;
}

/// Prims: Returns a MST (as a Graph object)
Graph Graph::Prims() {
  // Initialize a tree with a single vertex, chosen arbitrarily from the graph.
  Graph MST;
  if (!connected()) {
    return MST;
  }  // If the Graph is not connected, return an empty tree.
  std::string arbitraryNode = nodeMap_.begin()->first;
  MST.addNode(arbitraryNode);

  // Repeatedly add the lightest edge until all Nodes are in the tree.
  std::vector<std::tuple<std::string, std::string, double>> edges =
      getEdgesAscending();

  while (MST.numEdges() != (numNodes() - 1)) {  // There are |N-1| Edges in a
                                                // MST
    for (auto edge : edges) {
      // If one Node is in the tree and the other is not
      if ((MST.nodeExists(std::get<0>(edge)) &&
           !MST.nodeExists(std::get<1>(edge))) ||
          (!MST.nodeExists(std::get<0>(edge)) &&
           MST.nodeExists(std::get<1>(edge)))) {
        // add Nodes and Edge to MST
        MST.addNode(std::get<0>(edge));
        MST.addNode(std::get<1>(edge));
        MST.addEdge(std::get<0>(edge), std::get<1>(edge), std::get<2>(edge));
        break;
      }
    }
  }
  return MST;
}

Graph Graph::Kruskals() {
  // create a graph F (a set of trees), where each vertex in the graph is a
  // separate tree
  Graph MST;
  if (!connected()) {
    return MST;
  }  // If the Graph is not connected, return an empty tree.

  // Add all nodes in original to new Graph
  for (auto iter : nodeMap_) {
    double data = iter.second->getData();
    std::string name = iter.first;
    MST.addNode(data, name);
  }

  // create a set S containing all the edges in the graph
  std::vector<std::tuple<std::string, std::string, double>> edges =
      getEdgesDescending();

  // while S is nonempty and F is not yet spanning
  while (!edges.empty() && MST.numEdges() != (numNodes() - 1)) {
    // remove an edge with minimum weight from S
    auto edge = edges.back();
    edges.pop_back();
    std::string nodeA = std::get<0>(edge);
    std::string nodeB = std::get<1>(edge);
    // if the removed edge connects two different trees then add it to the
    // forest F, combining two trees into a single tree
    if (!MST.pathCheck(nodeA, nodeB)) {
      MST.addNode(nodeA);
      MST.addNode(nodeB);
      MST.addEdge(nodeA, nodeB, std::get<2>(edge));
    }
  }
  return MST;
}

/// getInfo: Returns a string of all Nodes along with their Edges.
std::string Graph::getInfo() {
  std::stringstream ss;
  ss << std::fixed;  // Prevents scientific-notation
  ss << "\n\nGraph Info: " << std::endl;
  // For Every Node
  for (auto iterA : nodeMap_) {
    ss << "[" << iterA.first << ": " << iterA.second->getData() << "] ";
    // For Every Neighbor of Node
    for (auto iterB : *(iterA.second->getMapPtr())) {
      ss << "(" << iterB.first << "): ";
      // Print Each Edge of Neighbor
      for (auto weight : iterB.second) {
        ss << weight << ", ";
      }
    }
    ss << "\n\n";
  }
  return ss.str();
}

/// getEdges: Returns an unsorted vector of edges, where edges are represented
/// with 3-tuples (nodeA, nodeB, weight)
std::vector<std::tuple<std::string, std::string, double>> Graph::getEdges()
    const {
  std::vector<std::tuple<std::string, std::string, double>> edgeVec;

  // For all Nodes K in nodeMap_
  for (auto iter : nodeMap_) {
    auto K = iter.second;  // K is a Node*
    // For all neighbors N of K
    for (auto iter1 : *(K->getMapPtr())) {
      auto tempSet = iter1.second;  // tempSet is an multiset
      // For all weights from K to N, add it to the edgeVec
      for (double i : tempSet) {
        std::string nodeA = iter.first;
        std::string nodeB = iter1.first;
        std::tuple<std::string, std::string, double> tempTuple(nodeA, nodeB, i);
        edgeVec.push_back(tempTuple);
      }
    }
  }

  // If the Graph is Undirected, post-process to delete duplicates ie
  // (nodeA,nodeB,w) and (nodeB, nodeA,w)
  if (!directed) {
    // For every (A,B,w) in edgeVec, delete one (B,A,w)
    std::vector<std::tuple<std::string, std::string, double>> deleteTheseEdges;
    for (auto edge : edgeVec) {
      std::string nodeA = std::get<0>(edge);
      std::string nodeB = std::get<1>(edge);
      double weight = std::get<2>(edge);
      std::tuple<std::string, std::string, double> deleteMe(nodeB, nodeA,
                                                            weight);
      if (nodeA > nodeB)  // Prevents deleting both duplicates, we just want to
                          // delete one to leave a unique edge.
        deleteTheseEdges.push_back(deleteMe);
    }

    for (auto edge : deleteTheseEdges) {
      edgeVec.erase(std::remove(edgeVec.begin(), edgeVec.end(), edge),
                    edgeVec.end());
    }
  }

  return edgeVec;
}

/// getEdgesAscending: Returns a sorted list of edges from low to high weights
std::vector<std::tuple<std::string, std::string, double>>
Graph::getEdgesAscending() const {
  std::vector<std::tuple<std::string, std::string, double>> edges = getEdges();

  std::sort(edges.begin(), edges.end(),
            [](const std::tuple<std::string, std::string, double>& a,
               const std::tuple<std::string, std::string, double>& b) -> bool {
              return std::get<2>(a) < std::get<2>(b);
            });

  return edges;
}

/// getEdgesDescending: Returns a sorted list of edges from high to low weights
std::vector<std::tuple<std::string, std::string, double>>
Graph::getEdgesDescending() const {
  std::vector<std::tuple<std::string, std::string, double>> edges = getEdges();

  std::sort(edges.begin(), edges.end(),
            [](const std::tuple<std::string, std::string, double>& a,
               const std::tuple<std::string, std::string, double>& b) -> bool {
              return std::get<2>(a) > std::get<2>(b);
            });

  return edges;
}

int Graph::numNodes() const { return nodeMap_.size(); }

int Graph::numEdges() const { return getEdges().size(); }
bool Graph::nodeExists(const std::string& name) const {
  return (nodeMap_.find(name) != nodeMap_.end());
}

/// saveGraph: Saves a Graph object as a .txt file for later retrieval
bool Graph::saveGraph(const std::string& outputFileName) const {
  // Prep .txt file
  std::ofstream output;
  char specialChar = '%';
  char separator = '^';
  output.open(outputFileName + ".txt");
  output << std::fixed;  // Prevents scientific-notation

  // Write Header, includes directed bool
  if (directed) {
    output << specialChar
           << "PERSISTANT GRAPH: DIRECTED (Do not edit this line)" << std::endl;
  } else {
    output << specialChar
           << "PERSISTANT GRAPH: UNDIRECTED (Do not edit this line)"
           << std::endl;
  }
  output << "---------------------------------------------------------"
         << std::endl;

  // Write Nodes
  output << specialChar << "NODES (Do not edit this line):" << std::endl;
  for (const auto iter : nodeMap_) {
    output << iter.first << separator << nodeMap_.at(iter.first)->getData()
           << std::endl;
  }

  // Write Edges
  output << specialChar << "EDGES (Do not edit this line):" << std::endl;
  for (auto tuple : getEdges()) {
    output << std::get<0>(tuple) << separator << std::get<1>(tuple) << separator
           << std::get<2>(tuple) << std::endl;
  }

  // Close .txt  file
  output.close();
  return true;
}
