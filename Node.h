// Min Chang
// Github: Minyc510

#ifndef NODE_H
#define NODE_H

#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>

class Node {
 private:
  double data_;
  std::string name_;

  // neighborMap: List of Nodes that this node has an edge to
  std::unordered_map<std::string, std::multiset<double>>* neighborMap_;
  // neighborSet: List of Nodes that have an edge to this Node
  std::unordered_set<std::string> neighborOfSet_;

 public:
  Node(double data, const std::string& name);
  ~Node();

  void addNeighbor(const std::string& neighborName, double weight);

  // Access
  double getData() const;
  std::unordered_map<std::string, std::multiset<double>>* getMapPtr() const;
  std::unordered_set<std::string>& getSetRef();
};
#endif  // NODE_H
