//Min Chang
//Github: Minyc510

#ifndef NODE_H
#define NODE_H

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <set>

class Node {

private:
  double data;
  std::string name;

  //neighborMap: List of Nodes that this node has an edge to
  std::unordered_map<std::string, std::multiset<double>>* neighborMap;
  //neighborSet: List of Nodes that have an edge to this Node
  std::unordered_set<std::string> neighborOfSet;

public:
  Node(double data, std::string name);
  ~Node();

  void addNeighbor(std::string neighborName, double weight);

  //Access
  double getData();
  std::unordered_map<std::string, std::multiset<double>>* getMapPtr();
  std::unordered_set<std::string>& getSetRef();
};
#endif // NODE_H
