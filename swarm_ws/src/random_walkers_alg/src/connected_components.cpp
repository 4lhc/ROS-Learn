/*
 *  connected_components.cpp
 *
 *  author : Sreejith S
 *  email  : echo $(base64 -d <<< NDQ0bGhjCg==)@gmail.com
 *  date   : Wed 23 Jun 2020 04:16:51 PM IST
 *  ver    :
 *
 * Find the connected components in the given graph
 *
 */

#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <sstream>
#include <cassert>
#include "Graph.h"


using std::vector;
using namespace ALG;


template <class Type>
void recursiveDFS(Graph<Type>& G, vector<char>& visited, int source)
{
    //Args: Graph, visited, index of Source Node
    //Here source node index is used for the ease of programming
    visited[source] = 1;
    std::cout << G[source] << ", ";
    vector<int> outAdj = G.outAdjacentNodes(G[source]);
    for(auto& w : outAdj)
    {
        if (!visited[w])
            recursiveDFS(G, visited, w);
    }
}


int main()
{

    Graph<char> G;
    int n;
    char c, node_name;
    //std::cout << "Enter No. of Nodes: " << std::endl;
    std::cin >> n;
    assert(n < G.sizeMax());
    //std::cout << "Enter the Graph (Adjacency List): " << std::endl;
    for (int i = 0; i < n; ++i) {
        std::cin >> node_name;
        G.addNode(node_name);
        std::string line;
        std::getline(std::cin, line);
        std::istringstream iss(line);
        while ( iss >> c )
        {
            G.addEdge(node_name, c);
        }

    }
    //G.printAdjMat();

    std::vector<char> visited(G.size(), 0); //to keep track of visited nodes
    for (int i = 0; i < G.size(); i++)
    {
        if (!visited[i])
        {
            //Node index is used instead of name
             recursiveDFS(G, visited, i); //starting at node of index 0
            std::cout << "\n";

        }
    }

    return 0;
}


