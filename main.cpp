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

  Graph F(G);

  for (auto e : F.getEdges()) {
    cout << get<0>(e) << " " << get<1>(e) << " " << get<2>(e) << endl;
  }

  cout << "Connectedness testing:" << endl;
  Graph H(true); //Directed Graph


  H.addNode("A");
  H.addNode("B");
  H.addNode("C");
  H.addNode("D");

  H.addEdge("A","B");
  H.addEdge("B","C");
  H.addEdge("C","D");

  cout << H.weaklyConnected() << endl;



}
