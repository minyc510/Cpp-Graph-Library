//Min Chang
//Github: Minyc510

#include "Graph.h"
#include <iostream>
#include <string>

using namespace std;

int main() {
  //Undirected Graph Object
  Graph G(true);

  vector<string> nodes = {"A", "B", "C", "D"};
  G.addNodes(nodes);

  G.addEdge("A", "B");
  G.addEdge("A", "C");
  G.addEdge("A", "D");

  for (auto neighborName : G.neighborNames("A")) {
   cout << neighborName << " ";
  }

  cout << endl;

  Graph F = G.transpose();
  for (auto neighborName : F.neighborNames("A")) {
    cout << neighborName << " ";
  }






}
