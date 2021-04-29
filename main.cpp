// Min Chang
// Github: Minyc510

#include <fstream>
#include <iostream>
#include <string>

#include "Graph.h"

using namespace std;

string edgeString(const tuple<string, string, int>&);  // Useful for print edges

int main() {
  vector<string> nodes = {"A", "B", "C", "D"};
  Graph G;
  G.addNode(1.12314, "A");
  G.addNode(-7.3412, "B");
  G.addNode(420, "C");
  G.addNode(-12423, "D");
  G.addNode("Isolated");
  G.addNode("X");
  G.addNode("X");
  G.addNode("Y");

  G.addEdge("A", "B", 1);
  G.addEdge("A", "C", 3);
  G.addEdge("B", "C", -714234.322323);
  G.addEdge("C", "D", 313412341234123);
  G.addEdge("D", "A", -3);
  G.addEdge("X", "Y");


  G.saveGraph("Yee");
  Graph F(string("Yee.txt"));

  Graph H = G.transpose();
  H.saveGraph("Foo");

  Graph Foo(string("Foo.txt"));

  cout << G.getInfo();
  cout << F.getInfo();
  cout << Foo.getInfo();

  cout << "\n------------------------------------" << endl;
  cout << "Done." << endl;
  cout << "------------------------------------" << endl;
}

string edgeString(const tuple<string, string, int>& edge) {
  string str = get<0>(edge) + get<1>(edge) + to_string(get<2>(edge));
  return str;
}
