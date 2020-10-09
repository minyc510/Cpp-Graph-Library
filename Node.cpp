// Min Chang
// Github: Minyc510

#include "Node.h"

#include <utility>

// TODO: switch to smart pointers

Node::Node(double data, const std::string& name) : data_(data), name_(name) {
  // Dynamically allocate neighborMap
  std::unordered_map<std::string, std::multiset<double>>* mapPointer =
      new std::unordered_map<std::string, std::multiset<double>>();
  neighborMap_ = mapPointer;
}

Node::~Node() { delete neighborMap_; }

void Node::addNeighbor(const std::string& neighborName, double weight) {
  // If the new neighbor is not already a neighbor add it to the list
  if (neighborMap_->find(neighborName) == neighborMap_->end()) {
    std::multiset<double> tempSet;
    std::pair<std::string, std::multiset<double>> tempPair(neighborName,
                                                           tempSet);
    neighborMap_->insert(tempPair);
  }

  // Add edge of this 'weight'
  (*neighborMap_)[neighborName].insert(weight);
}

double Node::getData() const { return data_; }

std::unordered_map<std::string, std::multiset<double>>* Node::getMapPtr() const {
  return neighborMap_;
}

std::unordered_set<std::string>& Node::getSetRef() { return neighborOfSet_; }
