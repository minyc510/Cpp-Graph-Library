//Min Chang
//Github: Minyc510

#include "Graph.h"
#include <iostream>
#include <sstream>
#include <queue>

using namespace std;

//Substitute "to_string"
string int2String(int i)
{
  stringstream ss;
    ss << i;
    return ss.str();
}

int main() {
  //Undirected Graph Object
  Graph G(false);

  vector<string> nodes = {"A", "B", "C", "D", "E"};
  G.addNodes(nodes);

  G.addEdge("A", "B", 18);
  G.addEdge("A", "C", 2);
  G.addEdge("C", "D", 3);
  G.addEdge("D", "E", 4);
  G.addEdge("B", "E", 1);

  cout << "G connected?" << G.connected() << endl;

}
