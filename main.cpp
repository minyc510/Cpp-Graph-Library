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
  H.addEdge("A","B",4);
  H.addEdge("B","C",2);
  H.addEdge("B","D",3);

  cout << "H:";
  cout << H.getInfo();

  cout << "H weakly connected?: " << H.weaklyConnected() << endl;
  cout << "H strongly connected?: " << H.stronglyConnected() << endl;

  for (auto x : H.getEdges()) {
    cout << get<2>(x) << " ";
  }
  cout << endl;
  for (auto x : H.getEdgesSorted()) {
    cout << get<2>(x) << " ";
  }
  cout << endl;

  vector<string> nodes1 = {"A", "B", "C", "D", "E", "F", "G", "H", "I"};
  Graph I(false);
  I.addNodes(nodes1);
  I.addEdge("A","B",1);
  I.addEdge("B","D",2);
  I.addEdge("D","C",8);
  I.addEdge("D","E",50);
  I.addEdge("D","F",5);
  I.addEdge("D","I",1);
  I.addEdge("G","I",2);
  I.addEdge("G","E",3);
  I.addEdge("C","H",7);
  I.addEdge("H","F",100);
  cout << "Prims.." << endl;
  Graph MST = I.Prims();

  cout << I.getInfo();
  cout << I.connected() << "??";
  cout << MST.getInfo();


  cout << "------------------------------------" << endl;
  cout << "Done." << endl;
  cout << "------------------------------------" << endl;
}
