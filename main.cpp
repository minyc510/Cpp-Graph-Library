//Min Chang
//Github: Minyc510

#include "Graph.h"
#include <iostream>
#include <sstream>

using namespace std;

//Substitute "to_string"
string int2String(int i)
{
    stringstream ss;
    ss << i;
    return ss.str();
}

int main() {
  Graph G;

  G.addNode("A");
  G.addNode("B");
  G.addNode("C");


  G.addEdge("A", "B", 3);
  G.addEdge("B", "A", 3);

  G.addEdge("B", "C", 1);
  G.addEdge("C", "B", 1);

  G.addEdge("A", "C", 1);
  G.addEdge("C", "A", 1);


for (auto tPair : G.BFS("A")) {
    cout << tPair.first << ": " << tPair.second << endl;
  }


}
