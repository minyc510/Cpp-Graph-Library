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
  Graph Q;
  Q.addNodes(nodes);
  Q.addEdge("A","B", 1);
  Q.addEdge("B","C", 7);
  Q.addEdge("C","D", 2);
  Q.addEdge("D","A", -3);

  Q.saveToFile("G");





}

string edgeString(tuple<string, string, int> edge) {
  string str = get<0>(edge) + get<1>(edge) + to_string(get<2>(edge));
  return str;
}
