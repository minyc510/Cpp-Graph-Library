## Synopsis

This C++ Graph Library consists of a .h and .cpp files, most-importantly: Graph.g and Graph.cpp. Including these files will allow the user to create 'Graph' objects which can be used for whatever Graph-related problem a user may need to solve.
 
## Code Example

	#include Graph.h

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
  	}

## Motivation

Rather than writing this Graph library for practical use, it was really a tool for learning more about Graphs and further cementing my experience with C++. The idea came to me when working on a homework assignment for my 'Algorithm Design and Analysis' class (ie CSE 101 at UCSD) which involved coding a Graph algorithm, where having a Graph library would have been useful. 

## Installation

Simply download the .h and .cpp files, place them in your working directory, and "#include" them in your appropiate files.

