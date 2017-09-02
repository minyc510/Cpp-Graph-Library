//Min Chang
//Github: Minyc510

#include "Graph.h"
#include <iostream>
#include <string>

using namespace std;

int main() {
  //Undirected Graph Object
  Graph G(false);

  vector<string> nodes = {"A", "B", "C", "D", "E", "F", "G", "H", "I", "J"};
  G.addNodes(nodes);

  G.addEdge("A", "B");
  G.addEdge("A", "C");
  G.addEdge("A", "D");
  G.addEdge("E", "B");
  G.addEdge("E", "F");
  G.addEdge("F", "G");
  G.addEdge("H", "G");
  G.addEdge("I", "G");
  G.addEdge("I", "J");

  for (auto x : G.getEdges()) {
    cout << get<0>(x) << get<1>(x) << get<2>(x) << endl;
  }
}
