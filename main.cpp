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
  G.addEdge("B", "C");
  cout << G.pathCheck("A", "B") << endl;
  cout << G.pathCheck("C", "A") << endl;
  cout << G.pathCheck("B", "J") << endl;
}
