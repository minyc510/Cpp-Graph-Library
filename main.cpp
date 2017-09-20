//Min Chang
//Github: Minyc510

#include "Graph.h"
#include <iostream>
#include <string>

using namespace std;

string edgeString(tuple<string, string, int>); //Useful for print edges

int main() {

  vector<string> nodes = {"A", "B", "C", "D"};

  //Directed Graph Object
  Graph G(true);
  G.addNodes(nodes);
  for (string node : nodes) { cout << node << " "; }
  G.addEdge("A", "B", 7);
  G.addEdge("A", "C", 3);
  G.addEdge("A", "D", 12);
  G.addEdge("D", "C", 1);
  G.addEdge("B", "C", 35);
  tuple<string, string, int> edge ("A","B",69);
  G.addEdge(edge);


  cout << "\nStandard: " << endl;
  for (auto x : G.getEdges()) { cout << edgeString(x) << " "; }

  cout << "\nAscending: " << endl;
  for (auto x : G.getEdgesAscending()) { cout << edgeString(x) << " "; }

  cout << "\nDescending: " << endl;
  for (auto x : G.getEdgesDescending()) { cout << edgeString(x) << " "; }

  cout << "\n------------------------------------" << endl;
  cout << "Done." << endl;
  cout << "------------------------------------" << endl;
}

string edgeString(tuple<string, string, int> edge) {
  string str = get<0>(edge) + get<1>(edge) + to_string(get<2>(edge));
  return str;
}
