//Min Chang
//Github: Minyc510

#include "Node.h"
#include <utility>

Node::Node(int data, std::string name) {
  this->data = data;
  this->name = name;
  //Dynamically allocate neighborMap
  std::unordered_map<std::string, std::multiset<double>>* mapPointer = new std::unordered_map<std::string, std::multiset<double>>();
  neighborMap = mapPointer;
}

Node::~Node() {
  delete neighborMap;
}

void Node::addNeighbor(std::string neighborName, double weight) {
  //If the new neighbor is not already a neighbor add it to the list
  if (neighborMap->find(neighborName) == neighborMap->end()) {
    std::multiset<double> tempSet;
    std::pair<std::string, std::multiset<double>> tempPair(neighborName,tempSet);
    neighborMap->insert(tempPair);
  }

  //Add edge of this 'weight'
  (*neighborMap)[neighborName].insert(weight);
}

int Node::getData() {
  return data;
}

std::unordered_map<std::string, std::multiset<double>>* Node::getMapPtr() {
  return neighborMap;
}

std::unordered_set<std::string>& Node::getSetRef() { return neighborOfSet; }
