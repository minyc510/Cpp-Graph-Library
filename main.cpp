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
  Graph G(false);
  G.addNodes(nodes);
  G.addEdge("A", "B", 1);
  G.addEdge("A", "C", 4);
  G.addEdge("A", "D", 3);
  G.addEdge("B", "D", 2);
  G.addEdge("D", "C", 5);
  tuple<string, string, int> edge ("A","B",69);
  G.addEdge(edge);

  Graph MST1 = G.Kruskals();
  Graph MST2 = G.Prims();
  cout << G.getInfo() << endl;
  cout << MST1.getInfo() << endl;
  cout << MST2.getInfo() << endl;

  cout << "\n***BELLMAN FORD***\n";
  unordered_map<string, int> dist = G.BellmanFord("A");
  for (auto x : dist) {
    cout << x.first << " " << x.second << endl;
  }





  cout << "\n------------------------------------" << endl;
  cout << "Done." << endl;
  cout << "------------------------------------" << endl;
}

string edgeString(tuple<string, string, int> edge) {
  string str = get<0>(edge) + get<1>(edge) + to_string(get<2>(edge));
  return str;
}
