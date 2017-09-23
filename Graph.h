//Min Chang
//Github: Minyc510

#ifndef GRAPH_H
#define GRAPH_H

#include "Node.h"
#include <vector>
#include <utility>
#include <tuple>

class Graph {

private:
  std::unordered_map<std::string, Node*> nodeMap;
  bool directed = true;

public:
  //Constructors & Destructor
  Graph(); //Graphs are directed by default
  Graph(bool directed);
  Graph(const Graph& other); //Copy-Constructor, uses getEdges function
  ~Graph();

  //Trivial Functions
  bool addNode(int data, std::string name);
  bool addNode(std::string name); //Default data-value '1'
  void addNodes(std::vector<std::string> nodes);
  void addNodes(std::vector<std::pair<int, std::string>> nodes);
  bool addEdge(std::string fromNode, std::string toNode, double weight);
  bool addEdge(std::string fromNode, std::string toNode); //Default weight '1'
  bool addEdge(std::tuple<std::string, std::string, double> edge);
  bool deleteNode(std::string targetNode);
  bool deleteEdge(std::string fromNode, std::string toNode, double weight);
  bool deleteEdge(std::string fromNode, std::string toNode); //Default weight '1'

  //Undirected Graph Specific Functions
  bool connected(); //Is the Graph connected?

  //Directed Graph Specific Functions
  bool weaklyConnected() const; //Is the Graph weakly connected?
  bool stronglyConnected();

  //Modification Functions
  Graph transpose() const; //Creates a copy, reverses edges of that copy and returns it.

  //Neighbor Functions
  std::vector<std::string> neighborNames(std::string name);
  std::vector<std::pair<std::string, double>> neighborDistMin(std::string name);
  std::vector<std::pair<std::string, double>> neighborDistMax(std::string name);
  bool deleteNeighbors(std::string name);

  //Explore Functions
  std::unordered_set<std::string> explore(std::string sourceNode); //Returns a set of Nodes reachable from the source Node
  void exploreHelper(std::unordered_set<std::string> &visited, std::string name);
  std::vector<std::string> reachableNames(std::string sourceNode); //Returns a list of Nodes that are reachable from the target
  std::vector<std::pair<std::string, double>> reachableDists(std::string sourceNode); //Includes distances
  bool pathCheck(std::string fromNode, std::string toNode);

  //Core Graph Functions
  std::vector<std::string> BFS(std::string sourceNode, std::string targetNode); //Returns the shortest path from source to target
  std::vector<std::string> DFS(std::string sourceNode, std::string targetNode); //Returns the shortest path from source to target
  void DFShelper(std::string sourceNode, std::string targetNode, std::unordered_map<std::string, std::string> &prevMap);
  std::vector<std::string> Dijktras(std::string sourceNode, std::string targetNode); //Returns the shortest path from source to target
  std::unordered_map<std::string, double> Dijktras(std::string sourceNode); //Returns a map where keys are nodes reachable from source and values are the shortest distance from source

  //BellmanFord: Returns a 3-tuple containing the Dist and Prev maps, as well as a boolean for the existence of a negative cycle
  std::tuple<std::unordered_map<std::string, double>, std::unordered_map<std::string, std::string>, bool> BellmanFord(std::string sourceNode);
  std::unordered_map<std::string, double> BellmanFordDist(std::string sourceNode); //Returns just the Dist map
  std::unordered_map<std::string, std::string> BellmanFordPrev(std::string sourceNode); //Returns just the Prev map
  bool NegativeCycle(); //Does the graph contain a negCycle? Warning!: Exponential Run-Time

  //MST Functions
  Graph Prims();
  Graph Kruskals();

  //About Graph Functions
  std::string getInfo(); //Returns a list of all Nodes along with their Edges.
  std::vector< std::tuple<std::string, std::string, double> > getEdges() const; //Returns a vector of Edges, where Edges are represented with a 3-tuple (nodeA,nodeB,weight)
  std::vector< std::tuple<std::string, std::string, double> > getEdgesAscending() const;
  std::vector< std::tuple<std::string, std::string, double> > getEdgesDescending() const;
  int numNodes(); //Returns the number of Nodes
  int numEdges();
  bool nodeExists(std::string node); //Is the Node in the Graph?
};

#endif // GRAPH_H
