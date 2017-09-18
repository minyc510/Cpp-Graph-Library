//Min Chang
//Github: Minyc510

#include "Graph.h"
#include <iostream>
#include <string>

using namespace std;

int main() {

  vector<string> nodes = {"A", "B", "C", "D"};

  //Directed Graph Object
  Graph G(true);
  cout << "Directed Graph 'G' created." << endl;
  G.addNodes(nodes);
  cout << "Nodes added: ";
  for (string node : nodes) { cout << node << " "; }
  G.addEdge("A", "B");
  G.addEdge("A", "C");
  G.addEdge("A", "D");
  cout << "\nEdges added. ";

  cout << "\nNeighbors of 'A' in G: " << endl;
  for (auto neighborName : G.neighborNames("A")) {
   cout << neighborName << " ";
  }
  cout << endl;

  Graph F = G.transpose();
  cout << "Neighbors of 'A' in transpose of G:" << endl;
  for (auto neighborName : F.neighborNames("A")) {
    cout << neighborName << " ";
  }
  cout << endl;

  cout << "Creating Directed Graph H." << endl;
  Graph H(true);
  H.addNodes(nodes);
  H.addEdge("A","B");
  H.addEdge("B","C");
  H.addEdge("B","D");

  cout << "H:";
  cout << H.getInfo();

  cout << "H weakly connected?: " << H.weaklyConnected() << endl;
  cout << "H strongly connected?: " << H.stronglyConnected() << endl;


  cout << "------------------------------------" << endl;
  cout << "Done." << endl;
  cout << "------------------------------------" << endl;
}
