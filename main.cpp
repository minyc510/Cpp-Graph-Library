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
  G.addEdge("A", "B");
  G.addEdge("A", "B");
  G.addEdge("A", "B");
  G.addEdge("B", "C");
  G.addEdge("A", "D");

  for (auto e : G.getEdges()) {
    cout << get<0>(e) << " " << get<1>(e) << " " << get<2>(e) << endl;
  }

  cout << "done" << endl;

}
