//Min Chang
//Github: Minyc510

#include "Node.h"
#include <utility>

Node::Node(int data, std::string name) {
  this->data = data;
  this->name = name;
  //Dynamically allocate neighborMap
  std::unordered_map<std::string, std::multiset<int>>* mapPointer = new std::unordered_map<std::string, std::multiset<int>>();
  neighborMap = mapPointer;
}

Node::~Node() {
  delete neighborMap;
}

void Node::addNeighbor(std::string neighborName, int weight) {
  //If the new neighbor is not already a neighbor add it to the list
  if (neighborMap->find(neighborName) == neighborMap->end()) {
    std::multiset<int> tempMultiSet;
    std::pair<std::string, std::multiset<int>> tempPair(neighborName,tempMultiSet);
    neighborMap->insert(tempPair);
  }

  //Add edge of this 'weight'
  (*neighborMap)[neighborName].insert(weight);
}

std::unordered_map<std::string, std::multiset<int>>* Node::getMapPtr() {
  return neighborMap;
}

std::unordered_set<std::string>& Node::getSetRef() { return neighborOfSet; }
