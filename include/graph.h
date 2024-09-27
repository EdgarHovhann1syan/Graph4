#ifndef GRAPH_H
#define GRAPH_H
#include <vector>
#include <list>
#include <iostream>
#include <queue>
#include <algorithm>
#include <stdexcept>
#include <stack>
class Graph
{
public:
    Graph(std::size_t countOfVertices, bool isDirected = true);
    void addEdge(std::size_t u, std::size_t v, int weight);
    void removeEdge(std::size_t u, std::size_t v);
    void printGraph();
    void dfs(std::size_t u);
    void bfs(std::size_t u);
    void transpose();
    std::size_t getCountNodesAtAGivenLevelBFS(std::size_t u, std::size_t level);
    std::size_t getCountNodesAtAGivenLevelDFS(std::size_t u, std::size_t level);
    std::size_t getCountOfAllPossiblePaths(std::size_t u, std::size_t v);
    bool isCycled();
    std::vector<std::size_t> topSort_dfs();
    std::vector<std::size_t> topSort_Kahn();
    void findSCC_Kosarajou();
    void findSCC_Tarjan();
    std::vector<int> findSSSP_dfs(std::size_t u);
    std::vector<std::size_t> findShortestPath_dfs(std::size_t u, std::size_t v);
    std::vector<int> findSSSP_dijkstra(std::size_t u);
private:
    std::vector<std::list<std::pair<std::size_t, int>>> m_adjList;
    std::size_t m_countOfVertices;
    bool m_isDirected;
    void dfs_helper(std::size_t u, std::vector<bool>& visited);
    void dfs_helper_countNodes(std::size_t u, std::size_t currentLevel, std::size_t targetLevel, std::vector<bool>& visited, std::size_t& count);
    std::size_t dfs_helper_countPaths(std::size_t u, std::size_t v, std::vector<bool>& visited);
    bool dfs_helper_isCycledDirected(std::size_t u, std::vector<bool>& visited, std::vector<bool>& onStack);
    bool dfs_helper_isCycledUndirected(std::size_t u, std::vector<bool>& visited, std::size_t parent);
    bool isCycledUndirected_helper();
    bool isCycledDirected_helper();
    void dfs_helper_topSort(std::size_t u, std::vector<bool>& visited, std::vector<std::size_t>& topOrder);
    void fillInOrder(std::size_t u, std::vector<bool>& visited, std::stack<std::size_t>& s);
    void dfs_helper_onTransposedGraph(std::size_t u, std::vector<bool>& visited);
    void dfs_helper_Tarjan(std::size_t u, std::vector<int>& ids, std::vector<int>& lowLink, std::stack<std::size_t>& s, std::vector<bool>& onStack);
    void dfs_helper_sssp(std::size_t u, std::vector<int>& dist);
    void dfs_helper_sssp_with_path(std::size_t u, std::vector<int>& dist, std::vector<int>& parent);
};
#endif //GRAPH_H