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
  Graph G(false);

  G.addNode("A");
  G.addNode("B");
  G.addNode("C");
  G.addNode("D");

  G.addEdge("A","B");
  G.addEdge("A","B");
  G.addEdge("C","D");

  cout << G.getInfo();

}
