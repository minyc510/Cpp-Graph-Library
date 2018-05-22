## Synopsis

This C++ Graph Library consists of a *.h and *.cpp files, most-importantly: Graph.h and Graph.cpp. Including these files will allow the user to instantiate 'Graph' objects to model graph-related problems. Graphs are persistent (ie they can be written to disk as *.txt files for later retrieval through a read-from-file Graph constructor). 

No external libraries (eg Boost) were used in this project, except for the C++ Standard Template Library.
## Code Example

	#include Graph.h
	using namespace std;
 	int main()
 	{
    	  //Declare a Graph object, populate it.	
    	  Graph G;
    	  G.addNode("A"); 
    	  G.addNode("B");
    	  G.addEdge("A","B");
    	  ...
    	  vector<string> shortestPath = G.BFS("A","Z");
    	  Graph MST = G.Prims(); //Kruskals() also available
    	  MST.saveGraph("MinimumSpanningTree");

    	  //Retrieve Graph object from .txt file
    	  Graph F(string("NetworkConnections.txt"));
  	}

## Installation

Simply download the .h and .cpp files, place them in your working directory, and "#include" them in your appropiate files.

