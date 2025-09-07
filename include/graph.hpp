#pragma once

#include <algorithm>
#include <iostream>
#include <vector>
#include <queue>
#include <limits>

// Type Alias
using NodeID = int;
using Weight = int;

struct Edge{
    NodeID target;
    Weight weight;

    Edge() : target(0), weight(0){}
    Edge(NodeID t, Weight w) : target(t), weight(w) {}

    friend std::ostream& operator<<(std::ostream& os, const Edge& edge) {
        os << "(" << edge.target << ", " << edge.weight << ")";
        return os;
    }
};

class Graph{
private:
    NodeID numNodes; // Total number of nodes in the graph
    std::vector<std::vector<Edge>> adj; // Adjacency list to store the graph
    bool directed; // True if the graph is directed, false otherwise

    // Helper method to validate if a NodeID is within the graph's bounds
    void validateNodeID(NodeID i) const{
         if (i < 0 || i >= numNodes) {
            throw std::out_of_range("Node ID " + std::to_string(i) + " is out of graph bounds (0 to " + std::to_string(numNodes - 1) + ")");
         }
    }

public:
    // Constructor: Initializes a graph with a specified number of nodes and directionality.
    Graph(NodeID n, bool isDirected = false) : numNodes(n), directed(isDirected){
        if (n <= 0) {
            std::cerr << "Number of nodes must be positive." << std::endl;
        }
        adj.resize(numNodes); // Resize the adjacency list to hold 'n' lists for each node
    }

    /*
        Set the number of nodes in the graph
    */
    void resize(NodeID n){
        numNodes = n;
        adj.resize(n);
    }

    /*
        Add an edge between two nodes with a given weight.
        For undirected graphs, an edge is added in both directions
    */
    void addEdge(NodeID u, NodeID v, Weight w) {
        validateNodeID(u);
        validateNodeID(v);

        // Add edge from u to v
        adj[u].emplace_back(v, w);

        // If the graph is undirected, add an edge from v to u as well.
        if (!directed) {
            adj[v].emplace_back(u, w);
        }
    }

    /*
        Get all neighbors (and their edge weights) of a given node.
    */
    const std::vector<Edge>& getNeighbors(NodeID u) const {
        validateNodeID(u);
        return adj[u];
    }

    /*
        Get the total number of nodes in the graph.
    */
    int getNumNodes() const {
        return numNodes;
    }

    /*
        Check if the graph is directed.
    */
    bool isDirected() const {
        return directed;
    }
    /* 
        Print the graph's adjacency list for debugging purposes.
    */
    void printGraph() const {
        std::cout << "Graph (Nodes: " << numNodes << ", Directed: " << (directed ? "Yes" : "No") << ")\n";
        for (NodeID i = 0; i < numNodes; ++i) {
            std::cout << "Node " << i << ": ";
            if (adj[i].empty()) {
                std::cout << "(no outgoing edges)";
            } else {
                for (const Edge& edge : adj[i]) {
                    std::cout << edge << " ";
                }
            }
            std::cout << "\n";
        }
    }
};

struct PathResult{
    // Shortest distance from the source node to every other node.
    std::vector<Weight> distances;
    // Predecessor of each node on the shortest path from the source.
    std::vector<NodeID> predecessors;

    /*
        Construct a path from source to target
        Returns an empty vector if target is unreachable or invalid.
    */
    std::vector<NodeID> constructPath(NodeID target) const {
        std::vector<NodeID> path;
        if (target >= distances.size() || distances[target] == std::numeric_limits<Weight>::max()) {
            return path; // Return empty path when target is max weight
        }

        NodeID current = target;

        while (current != -1) {
            path.push_back(current);
            if (predecessors[current] == -1) { // Reached the source node
                 break;
            }
            current = predecessors[current];
        }
        std::reverse(path.begin(), path.end());
        return path;
    }
};

/*
    Breadth-First Search algorithm
    Shortest Path
    Unweighted edges
    Ignores weights
*/
struct Bfs{
    PathResult mesh;

    void solve(NodeID startNode, Graph& graph){
        if(startNode < 0 || startNode >= graph.getNumNodes()){
            std::cerr << "Bfs tried to solve from a out of bounds starting nodes" << std::endl;
        }

        // Reset results to the starting values
        mesh.distances.assign(graph.getNumNodes(), std::numeric_limits<Weight>::max()); // Represents infinity
        mesh.predecessors.assign(graph.getNumNodes(), -1);

        std::queue<NodeID> q;

        mesh.distances[startNode] = 0;
        q.push(startNode);

        while(!q.empty()){
            NodeID u = q.front();
            q.pop();

            for(const Edge& edge : graph.getNeighbors(u)){
                NodeID v = edge.target;
                if(mesh.distances[v] == std::numeric_limits<Weight>::max()){
                    mesh.distances[v] = mesh.distances[u] + 1;
                    mesh.predecessors[v] = u;
                    q.push(v);
                }
            }
        }
    }
};

/*
    Dijkstra Search algorithm
    Shortest Path
    Weighted edges
    No negativ weights
*/
struct Dijkstra{
    PathResult mesh;

    void solve(NodeID startNode, Graph& graph){
        if(startNode < 0 || startNode >= graph.getNumNodes()){
            std::cerr << "Bfs tried to solve from a out of bounds starting nodes" << std::endl;
        }

        // Reset results to the starting values
        mesh.distances.assign(graph.getNumNodes(), std::numeric_limits<Weight>::max()); // Represents infinity
        mesh.predecessors.assign(graph.getNumNodes(), -1);

        std::priority_queue<std::pair<Weight, NodeID>, std::vector<std::pair<Weight, NodeID>>, std::greater<std::pair<Weight, NodeID>>> pq;

        mesh.distances[startNode] = 0;
        pq.push({0, startNode});

        while (!pq.empty()) {
            Weight d = pq.top().first; // Distance
            NodeID u = pq.top().second; // NodeID
            pq.pop();

            if (d > mesh.distances[u]) {
                continue; // Continue if current distance is greater than mesh value
            }

            for (const Edge& edge : graph.getNeighbors(u)){
                NodeID v = edge.target;
                Weight new_dist = mesh.distances[u] + edge.weight;

                if (new_dist < mesh.distances[v]) {
                    mesh.distances[v] = new_dist;
                    mesh.predecessors[v] = u;
                    pq.push({new_dist, v});
                }
            }
        }
    }
};