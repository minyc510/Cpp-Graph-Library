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
		G.BFS("A");
	}

## Motivation

This Graph Library is not intended to be the best out there, but creating it was simply a tool for learning and interest. 

## Installation

Simply download the .h and .cpp files, place them in your working directory, and "#include" them in your appropiate files.

