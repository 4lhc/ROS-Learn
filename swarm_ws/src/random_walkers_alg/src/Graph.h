/*
 *  Graph.h
 *
 *  author : Sreejith S
 *  email  : echo $(base64 -d <<< NDQ0bGhjCg==)@gmail.com
 *  date   : Sun 17 May 2020 17:59:56 IST
 *  ver    :
 *
 *  A graph implementation based on https://www.youtube.com/watch?v=BcaZIxaS-F4
 *
 */

#ifndef GRAPH_H
#define GRAPH_H
#include <assert.h>
#include <string>
#include <vector>
#include <iostream>
#include <iomanip>

namespace ALG
{

    class Edge
    {
        int source;
        int target;

    };

    template <class Type>
    class Graph
    {
        private:
            static const int MAX{20};
            int adjacencyMatrix[MAX][MAX];
            Type labels[MAX];
            std::vector<Edge*> edgeList;
            int numNodes{0}, numEdges{0};

        public:
            Graph();
            ~Graph();
            int size() const; //Return the number of nodes
            int getNumEdges() const;
            int sizeMax() const; //Return MAX
            void addNode(const Type& value);
            void addEdge(const Type& source, const Type& target);
            void addEdge(const Type& source, const Type& target, int weight);
            int getIndex(const Type& value) const;
            Type& operator [] (int node);
            void printAdjMat() const;
            std::vector<int> outAdjacentNodes(const Type& v) const;

    };
}

#endif
