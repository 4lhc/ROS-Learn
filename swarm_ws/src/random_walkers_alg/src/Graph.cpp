/*
 *  Graph.cpp
 *
 *  author : Sreejith S
 *  email  : echo $(base64 -d <<< NDQ0bGhjCg==)@gmail.com
 *  date   : Sun 17 May 2020 17:59:56 IST
 *  ver    :
 *
 */


#include "Graph.h"

using namespace ALG;


template <class Type>
const int Graph<Type>::MAX;

template <class Type>
Graph<Type>::Graph() {}

template <class Type>
Graph<Type>::~Graph() {}

template <class Type>
void Graph<Type>::addNode(const Type& value)
{
    assert(size() < MAX);
    if (getIndex(value) == -1)
    {
        //Avoid duplicate nodes
        int newNodeNum = numNodes;
        numNodes++;
        for(int i = 0; i < numNodes; i++)
        {
            adjacencyMatrix[i][newNodeNum] = 0;
            adjacencyMatrix[newNodeNum][i] = 0; //since no connections exists
        }
        labels[newNodeNum] = value;
    }
}

template <class Type>
void  Graph<Type>::addEdge(const Type& source, const Type& target, int weight)
{
    //in case target node doesn't exist, create it.
    int src{0}, dst{0};
    if (getIndex(target) == -1)
        addNode(target);
    src = getIndex(source);
    dst = getIndex(target);
    assert(src < size() && dst < size());
    adjacencyMatrix[src][dst] = weight;
    numEdges++;
}

template <class Type>
void  Graph<Type>::addEdge(const Type& source, const Type& target)
{
    addEdge(source, target, 1);
}

template <class Type>
int Graph<Type>::getIndex(const Type& value) const
{
    for(int i = 0; i < numNodes; i++)
    {
        if (labels[i] == value)
            return i;
    }
    return -1;
}

template <class Type>
Type& Graph<Type>::operator [](int node)
{
    assert(node < size());
    return labels[node];
}

template <class Type>
int Graph<Type>::size() const
{
    return numNodes;
}

template <class Type>
int Graph<Type>::sizeMax() const
{
    return MAX;
}

template <class Type>
int Graph<Type>::getNumEdges() const
{
    //undirected edges counted as two
    return numEdges;
}

template <class Type>
void Graph<Type>::printAdjMat() const
{
    int col_margin{3};
    int n = size();
    std::string hl(col_margin*n*2-1, '-');
    std::cout << " ";
    for (int i = 0; i < n; i++)
    {
        std::cout << std::setw(col_margin) << ' '
                  << std::setw(col_margin) << labels[i];
    }
    std::cout << std::endl;
    for(int i = 0; i < n; i++)
    {
        std::cout << std::setw(col_margin*(n*2 + 1)) << hl
                  << std::endl << labels[i];
        for(int j = 0; j < n; j++)
        {
            std::cout << std::setw(col_margin) << '|'
                      << std::setw(col_margin) << adjacencyMatrix[i][j];
        }
        std::cout << std::setw(col_margin) << '|' << std::endl;
    }
    std::cout << std::setw(col_margin*(n*2 + 1)) << hl << std::endl;
}

template <class Type>
std::vector<int> Graph<Type>::outAdjacentNodes(const Type& v) const
{
    std::vector<int> adj;
    int vi = getIndex(v);
    for(int i = 0; i < size(); i++)
    {
        if (adjacencyMatrix[vi][i] == 1)
        {
            adj.push_back(i);
        }
    }
    return adj;
}

//Explicitly instantiate the template
template class ALG::Graph<char>;
template class ALG::Graph<int>;
template class ALG::Graph<std::string>;
