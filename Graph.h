//Min Chang
//Github: Minyc510

#ifndef GRAPH_H
#define GRAPH_H

#include "Node.h"
#include <vector>
#include <utility>

class Graph {

private:
  std::unordered_map<std::string, Node*> nodeMap;

public:
  Graph();
  ~Graph();

  //Trivial Functions
  bool addNode(int data, std::string name);
  bool addEdge(std::string fromNode, std::string toNode, int weight);
  bool addEdge(std::string fromNode, std::string toNode); //Default weight '1'
  bool deleteNode(std::string targetNode);
  bool deleteEdge(std::string fromNode, std::string toNode, int weight);

  //Basic Functions
  std::vector<std::string> getNeighbors(std::string name);
  bool deleteNeighbors(std::string name);

  std::set<std::string> explore(std::string name); //Returns set of Nodes reachable from passed node
  void exploreHelper(std::set<std::string> &visited, std::string name);

  //std::unordered_map<std::string, int> BFS(std::string name); //Returns list of nodes reachable from name and the distance


  //void printInfo(); Temporary Function, useful for debugging.
};

#endif // GRAPH_H
