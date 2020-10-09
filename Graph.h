// Min Chang
// Github: Minyc510

#ifndef GRAPH_H
#define GRAPH_H

#include <tuple>
#include <utility>
#include <vector>

#include "Node.h"

// TODO: Make a template class.
class Graph {
 private:
  std::unordered_map<std::string, Node*> nodeMap_;
  bool directed = true;

 public:
  // Constructors & Destructor
  Graph();  // Graphs are directed by default
  Graph(bool directed);
  Graph(const Graph& other);  // Copy-Constructor, uses getEdges function
  Graph(const std::string& inputFileName);  // Constructs Graph from .txt file
  ~Graph();

  // Trivial Functions
  bool addNode(double data, const std::string& name);
  bool addNode(const std::string& name);  // Default data-value '1'
  void addNodes(const std::vector<std::string>& nodes);
  void addNodes(const std::vector<std::pair<double, std::string>>& nodes);
  bool addEdge(const std::string& fromNode, const std::string& toNode,
               double weight);
  bool addEdge(const std::string& fromNode,
               const std::string& toNode);  // Default weight '1'
  bool addEdge(const std::tuple<std::string, std::string, double>& edge);
  bool deleteNode(const std::string& targetNode);
  bool deleteEdge(const std::string& fromNode, const std::string& toNode,
                  double weight);
  bool deleteEdge(const std::string& fromNode,
                  const std::string& toNode);  // Default weight '1'

  // Undirected Graph Specific Functions
  bool connected() const;  // Is the Graph connected?

  // Directed Graph Specific Functions
  bool weaklyConnected() const;  // Is the Graph weakly connected?
  bool stronglyConnected() const;

  // Creates a copy, reverses edges of that copy and returns it.
  Graph transpose() const;

  // Neighbor Functions
  std::vector<std::string> neighborNames(const std::string& name) const;
  std::vector<std::pair<std::string, double>> neighborDistMin(
      const std::string& name) const;
  std::vector<std::pair<std::string, double>> neighborDistMax(
      const std::string& name) const;
  bool deleteNeighbors(const std::string& name);

  // Explore Functions
  // Returns a set of Nodes reachable from the source Node
  std::unordered_set<std::string> explore(const std::string& sourceNode) const;
  void exploreHelper(std::unordered_set<std::string>& visited,
                     const std::string& name) const;

  // Returns a list of Nodes that are reachable from the target
  std::vector<std::string> reachableNames(const std::string& sourceNode) const;
  std::vector<std::pair<std::string, double>> reachableDists(
      const std::string& sourceNode) const;  // Includes distances
  bool pathCheck(const std::string& fromNode, const std::string& toNode) const;

  // Core Graph Functions
  // Returns the shortest path from source to target
  std::vector<std::string> BFS(const std::string& sourceNode,
                               const std::string& targetNode) const;
  // Returns the shortest path from source to target
  std::vector<std::string> DFS(const std::string& sourceNode,
                               const std::string& targetNode) const;
  void DFShelper(const std::string& sourceNode, const std::string& targetNode,
                 std::unordered_map<std::string, std::string>& prevMap) const;
  // Returns the shortest path from source to target
  std::vector<std::string> Dijktras(const std::string& sourceNode,
                                    const std::string& targetNode) const;
  // Returns a map where keys are nodes reachable
  // from source and values are the shortest
  // distance from source
  std::unordered_map<std::string, double> Dijktras(
      const std::string& sourceNode) const;

  // BellmanFord: Returns a 3-tuple containing the Dist and Prev maps, as well
  // as a boolean for the existence of a negative cycle
  std::tuple<std::unordered_map<std::string, double>,
             std::unordered_map<std::string, std::string>, bool>
  BellmanFord(const std::string& sourceNode) const;
  std::unordered_map<std::string, double> BellmanFordDist(
      const std::string& sourceNode) const;  // Returns just the Dist map
  std::unordered_map<std::string, std::string> BellmanFordPrev(
      const std::string& sourceNode) const;  // Returns just the Prev map
  bool NegativeCycle() const;  // Does the graph contain a negCycle? Warning!:
                               // Exponential Run-Time

  // MST Functions
  Graph Prims();
  Graph Kruskals();

  // About Graph Functions
  std::string getInfo();  // Returns a list of all Nodes along with their Edges.
  std::vector<std::tuple<std::string, std::string, double>> getEdges()
      const;  // Returns a vector of Edges, where Edges are represented with a
              // 3-tuple (nodeA,nodeB,weight)
  std::vector<std::tuple<std::string, std::string, double>> getEdgesAscending()
      const;
  std::vector<std::tuple<std::string, std::string, double>> getEdgesDescending()
      const;
  int numNodes() const;  // Returns the number of Nodes
  int numEdges() const;
  bool nodeExists(const std::string& node) const;  // Is the Node in the Graph?

  // Persistent Graph Functions
  bool saveGraph(const std::string& outputFileName) const;
};

#endif  // GRAPH_H
