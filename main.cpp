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

  cout << "Adding Nodes: ";
  for (string node : nodes) { cout << node << " "; }
  cout << endl;
  G.addNodes(nodes);

  cout << "Adding Edges." << endl;
  G.addEdge("A", "B");
  G.addEdge("A", "C");
  G.addEdge("A", "D");

  cout << "Neighbors of 'A' in G: " << endl;
  for (auto neighborName : G.neighborNames("A")) {
   cout << neighborName << " ";
  }
  cout << endl;

  cout << "Transposing Graph G into Graph F..." << endl;
  Graph F = G.transpose();
  cout << "Neighbors of 'A' in F:" << endl;
  for (auto neighborName : F.neighborNames("A")) {
    cout << neighborName << " ";
  }
  cout << endl;



  cout << "------------------------------------" << endl;
  cout << "Done." << endl;
  cout << "------------------------------------" << endl;
}
