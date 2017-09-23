//Min Chang
//Github: Minyc510

#include "Graph.h"
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

string edgeString(tuple<string, string, int>); //Useful for print edges

int main() {

  vector<string> nodes = {"A", "B", "C", "D"};
  Graph G;
  G.addNode(1.12314, "A");
  G.addNode(-7.3412, "B");
  G.addNode(420, "C");
  G.addNode(-12423, "D");

  G.addEdge("A","B", 1);
  G.addEdge("B","C", -714.32);
  G.addEdge("C","D", 420);
  G.addEdge("D","A", -3);

  G.saveGraph("Yee");
  Graph F(string("Yee.txt"));

  cout << G.getInfo();
  cout << F.getInfo();


  cout << "\n------------------------------------" << endl;
  cout << "Done." << endl;
  cout << "------------------------------------" << endl;
}

string edgeString(tuple<string, string, int> edge) {
  string str = get<0>(edge) + get<1>(edge) + to_string(get<2>(edge));
  return str;
}
