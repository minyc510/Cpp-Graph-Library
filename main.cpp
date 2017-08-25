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
  cout << "C++ Graph Library - Basic Testing." << endl;
  const int N = 100000;
  Graph G;

  //Add Nodes
  cout << "Adding Nodes..." << endl;
  for (int i=0; i < N; i++) {
      G.addNode(0, int2String(i));
      if (i % (N/25) == 0) cout << "|";
  }

  //Add Edges
  cout << "\n\nAdding Edges..." << endl;
  for (int i=0; i < N-1; i++) {
      G.addEdge(int2String(i), int2String(i+1),0);
      if (i % (N/25) == 0) cout << "|";
  }

  //Delete Edges
  cout << "\n\nRemoving Edges..." << endl;
  for (int i=0; i < N-1; i++) {
      G.deleteEdge(int2String(i), int2String(i+1),0);
      if (i % (N/25) == 0) cout << "|";
  }

  //Add Edges II
  cout << "\n\nAdding Edges II..." << endl;
  for (int i=0; i < N-1; i++) {
      G.addEdge(int2String(i), int2String(i+1),0);
      if (i % (N/25) == 0) cout << "|";
  }

  //Delete Nodes
  cout << "\n\nRemoving Nodes..." << endl;
  for (int i=0; i < N; i++) {
      G.deleteNode(int2String(i));
      if (i % (N/25) == 0) cout << "|";
  }
}
