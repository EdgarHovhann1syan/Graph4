#include "../include/graph.h"

Graph::Graph(std::size_t countOfVertices, bool isDirected) : m_countOfVertices(countOfVertices), m_isDirected(isDirected)
{
    this->m_adjList.resize(this->m_countOfVertices);
}

void Graph::addEdge(std::size_t u, std::size_t v, int weight)
{
    if(u < this->m_countOfVertices && v < this->m_countOfVertices)
    {
        this->m_adjList[u].push_back(std::make_pair(v, weight));
        if(!this->m_isDirected)
        {
            this->m_adjList[v].push_back(std::make_pair(u, weight));
        }
    }
}

void Graph::removeEdge(std::size_t u, std::size_t v)
{
    if(u < this->m_countOfVertices && v < this->m_countOfVertices)
    {
        this->m_adjList[u].remove_if([v](const std::pair<std::size_t, int>& edge) {return edge.first == v;});

        if(!this->m_isDirected)
        {
            this->m_adjList[v].remove_if([u](const std::pair<std::size_t, int>& edge) {return edge.first == u;});
        }
    }
}

void Graph::printGraph()
{
    for(std::size_t u = 0; u < this->m_countOfVertices; ++u)
    {
        std::cout << "Vertex " << u << ": ";
        for(const auto& v : this->m_adjList[u])
        {
            std::cout << "( vertex: " << v.first << ", " << "weight: " << v.second << " )";
        }
        std::cout << std::endl;
    }
}

void Graph::dfs_helper(std::size_t u, std::vector<bool>& visited)
{
    visited[u] = true;
    std::cout << u << " ";
    for(const auto& v : this->m_adjList[u])
    {
        if(!visited[v.first])
        {
            dfs_helper(v.first, visited);
        }
    }
}

void Graph::dfs(std::size_t u)
{
    std::vector<bool> visited(this->m_countOfVertices, false);
    dfs_helper(u, visited);
    std::cout << std::endl;
}

void Graph::bfs(std::size_t u)
{
    std::vector<bool> visited(this->m_countOfVertices, false);
    std::queue<std::size_t> q;
    q.push(u);
    visited[u] = true;
    while(!q.empty())
    {
        std::size_t current = q.front();
        std::cout << current << " ";
        q.pop();
        for(const auto& v : this->m_adjList[current])
        {
            if(!visited[v.first])
            {
                q.push(v.first);
                visited[v.first] = true;
            }
        }
    }
    std::cout << std::endl;
}

void Graph::transpose()
{
    if(this->m_isDirected)
    {
        std::vector<std::list<std::pair<std::size_t, int>>> transposedGraph(this->m_countOfVertices);

        for(std::size_t u = 0; u < this->m_countOfVertices; ++u)
        {
            for(const auto& v : this->m_adjList[u])
            {
                transposedGraph[v.first].push_back(std::make_pair(u, v.second));
            }
        }
        this->m_adjList = transposedGraph;
    }
}

std::size_t Graph::getCountNodesAtAGivenLevelBFS(std::size_t u, std::size_t level)
{
    std::vector<bool> visited(this->m_countOfVertices, false);
    std::vector<std::size_t> levels(this->m_countOfVertices, 0);
    std::queue<std::size_t> q;
    visited[u] = true;
    q.push(u);
    while(!q.empty())
    {
        std::size_t current = q.front();
        for(const auto& v : this->m_adjList[current])
        {
            if(!visited[v.first])
            {
                visited[v.first] = true;
                q.push(v.first);
                levels[v.first] = levels[current] + 1;
            }
        }
    }
    std::size_t count = 0;
    for(int i = 0; i < this->m_countOfVertices; ++i)
    {
        if(levels[i] == level) ++count;
    }

    return count;
}

void Graph::dfs_helper_countNodes(std::size_t u, std::size_t currentLevel, std::size_t targetLevle, std::vector<bool>& visited, std::size_t& count)
{
    visited[u] = true;
    if(currentLevel == targetLevle) ++count;
    for(const auto& v : this->m_adjList[u])
    {
        if(!visited[v.first])
        {
            dfs_helper_countNodes(v.first, currentLevel + 1, targetLevle, visited, count);
        }
    }
}

std::size_t Graph::getCountNodesAtAGivenLevelDFS(std::size_t u, std::size_t level)
{
    std::vector<bool> visited(this->m_countOfVertices, false);
    std::size_t count = 0;
    dfs_helper_countNodes(u, 0, level, visited, count);
    return count;
}

std::size_t Graph::dfs_helper_countPaths(std::size_t u, std::size_t v, std::vector<bool>& visited)
{
    if(u == v) return 1;
    visited[u] = true;
    std::size_t count = 0;
    for(const auto& i : this->m_adjList[u])
    {
        if(!visited[i.first])
        {
            count += dfs_helper_countPaths(i.first, v, visited);
        }
    }
    visited[u] = false;
    return count;
}

std::size_t Graph::getCountOfAllPossiblePaths(std::size_t u, std::size_t v)
{
    std::vector<bool> visited(this->m_countOfVertices, false);
    return dfs_helper_countPaths(u, v, visited);
}

bool Graph::dfs_helper_isCycledDirected(std::size_t u, std::vector<bool>& visited, std::vector<bool>& onStack)
{
    visited[u] = true;
    onStack[u] = true;
    for(const auto& v : this->m_adjList[u])
    {
        if(!visited[v.first])
        {
            if(dfs_helper_isCycledDirected(v.first, visited, onStack)) return true;
        } else if(onStack[v.first])
        {
            return true;
        }
    }
    onStack[u] = false;
    return false;
}

bool Graph::dfs_helper_isCycledUndirected(std::size_t u, std::vector<bool>& visited, std::size_t parent)
{
    visited[u] = true;

    for(const auto& v : this->m_adjList[u])
    {
        if(!visited[v.first])
        {
            if(dfs_helper_isCycledUndirected(v.first, visited, u)) return true;
        } else 
        {
            if(v.first != parent)
            {
                return true;
            }
        }
    }
    return false;
}

bool Graph::isCycledDirected_helper()
{
    std::vector<bool> visited(this->m_countOfVertices, false);
    std::vector<bool> onStack(this->m_countOfVertices, false);
    for(std::size_t u = 0; u < this->m_countOfVertices; ++u)
    {
        if(!visited[u])
        {
            if(dfs_helper_isCycledDirected(u, visited, onStack))
            {
                return true;
            }
        }
    }
    return false;
}

bool Graph::isCycledUndirected_helper()
{
    std::vector<bool> visited(this->m_countOfVertices, false);
    for(std::size_t u = 0; u < this->m_countOfVertices; ++u)
    {
        if(!visited[u])
        {
            if(dfs_helper_isCycledUndirected(u, visited, -1))
            {
                return true;
            }
        }
    }
    return false;
}

bool Graph::isCycled()
{
    return this->m_isDirected ? isCycledDirected_helper() : isCycledUndirected_helper();
}

void Graph::dfs_helper_topSort(std::size_t u, std::vector<bool>& visited, std::vector<std::size_t>& topOrder)
{
    visited[u] = true;
    for(const auto& v : this->m_adjList[u])
    {
        if(!visited[v.first])
        {
            dfs_helper_topSort(v.first, visited, topOrder);
        }
    }
    topOrder.push_back(u);
}

std::vector<std::size_t> Graph::topSort_dfs()
{
    std::vector<bool> visited(this->m_countOfVertices, false);
    std::vector<std::size_t> topOrder;
    for(std::size_t u = 0; u < this->m_countOfVertices; ++u)
    {
        if(!visited[u])
        {
            dfs_helper_topSort(u, visited, topOrder);
        }
    }
    if(this->m_countOfVertices != topOrder.size()) throw std::logic_error("Graph is Cycled. TopSort is impossible");
    std::reverse(topOrder.begin(), topOrder.end());
    return topOrder;
}


std::vector<std::size_t> Graph::topSort_Kahn()
{
    std::vector<std::size_t> inDegree(this->m_countOfVertices, 0);
    for(std::size_t u = 0; u < this->m_countOfVertices; ++u)
    {
        for(const auto& v : this->m_adjList[u])
        {
            ++inDegree[v.first];
        }
    }

    std::queue<std::size_t> q;
    for(int i = 0; i < this->m_countOfVertices; ++i)
    {
        if(inDegree[i] == 0) q.push(i);
    }

    std::vector<std::size_t> topOrder;
    while(!q.empty())
    {
        std::size_t current = q.front();
        q.pop();
        topOrder.push_back(current);
        for(const auto& v : this->m_adjList[current])
        {
            --inDegree[v.first];
            if(inDegree[v.first] == 0)
            {
                q.push(v.first);
            }
        }
    }
    if(topOrder.size() != this->m_countOfVertices) throw std::logic_error("Graph is cycled. TopSort is impossible");
    return topOrder;
}

void Graph::fillInOrder(std::size_t u, std::vector<bool>& visited, std::stack<std::size_t>& s)
{
    visited[u] = true;
    for(const auto& v : this->m_adjList[u])
    {
        if(!visited[v.first])
        {
            fillInOrder(v.first, visited, s);
        }
    }
    s.push(u);
}

void Graph::dfs_helper_onTransposedGraph(std::size_t u, std::vector<bool>& visited)
{
    visited[u] = true;
    std::cout << u << " ";
    for(const auto& v : this->m_adjList[u])
    {
        if(!visited[v.first])
        {
            dfs_helper_onTransposedGraph(v.first, visited);
        }
    }
}

void Graph::findSCC_Kosarajou()
{
    std::vector<bool> visited(this->m_countOfVertices, false);
    std::stack<std::size_t> s;
    for(std::size_t u = 0; u < this->m_countOfVertices; ++u)
    {
        if(!visited[u])
        {
            fillInOrder(u, visited, s);
        }
    }

    std::fill(visited.begin(), visited.end(), false);
    transpose();
    while(!s.empty())
    {
        std::size_t current = s.top();
        s.pop();
        if(!visited[current])
        {
            std::cout << "SCC: ";
            dfs_helper_onTransposedGraph(current, visited);
            std::cout << std::endl;
        }
    }
}

void Graph::dfs_helper_Tarjan(std::size_t u, std::vector<int>& ids, std::vector<int>& lowLink, std::stack<std::size_t>& s, std::vector<bool>& onStack)
{
    static int id = 0;
    ids[u] = lowLink[u] = id++;
    onStack[u] = true;
    s.push(u);
    for(const auto& v : this->m_adjList[u])
    {
        if(ids[v.first] == -1) dfs_helper_Tarjan(v.first, ids, lowLink, s, onStack);
        if(onStack[v.first])
        {
            lowLink[u] = std::min(lowLink[v.first], lowLink[u]);
        }
    }

    if(ids[u] == lowLink[u])
    {
        std::cout << "SCC: ";
        while(true)
        {
            std::size_t top = s.top();
            onStack[top] = false;
            s.pop();
            std::cout << top << " ";
            if(u == top) break;
        }
        std::cout << std::endl;
    }
}

void Graph::findSCC_Tarjan()
{
    std::vector<int> ids(this->m_countOfVertices, -1);
    std::vector<int> lowLink(this->m_countOfVertices, -1);
    std::stack<std::size_t> s;
    std::vector<bool> onStack(this->m_countOfVertices, false);
    for(std::size_t u = 0; u < this->m_countOfVertices; ++u)
    {
        if(ids[u] == -1) dfs_helper_Tarjan(u, ids, lowLink, s, onStack);
    }
    
}

void Graph::dfs_helper_sssp(std::size_t u, std::vector<int>& dist)
{
    for(const auto& v : this->m_adjList[u])
    {
        int weight = v.second;
        int newDist = dist[u] + weight;
        if(newDist < dist[v.first])
        {
            dist[v.first] = newDist;
            dfs_helper_sssp(v.first, dist);
        }
    }
}

std::vector<int> Graph::findSSSP_dfs(std::size_t u)
{
    std::vector<int> dist(this->m_countOfVertices, std::numeric_limits<int>::max());
    dist[u] = 0;
    dfs_helper_sssp(u, dist);
    return dist;
}

void Graph::dfs_helper_sssp_with_path(std::size_t u, std::vector<int>& dist, std::vector<int>& parent)
{

    for(const auto& neighbor : this->m_adjList[u])
    {
        int weight = neighbor.second;
        std::size_t v = neighbor.first;

        int newDist = dist[u] + weight;

        if(newDist < dist[v])
        {
            dist[v] = newDist;
            parent[v] = u;
            dfs_helper_sssp_with_path(v, dist, parent);
        }
    }
}

std::vector<std::size_t> Graph::findShortestPath_dfs(std::size_t u, std::size_t v)
{
    std::vector<int> dist(this->m_countOfVertices, std::numeric_limits<int>::max());
    std::vector<int> parent(this->m_countOfVertices, -1);
    dist[u] = 0;
    dfs_helper_sssp_with_path(u, dist, parent);
    if(dist[v] == std::numeric_limits<int>::max())
    {
        return {};
    }

    std::vector<std::size_t> path;
    for(std::size_t at = v; at != -1; at = parent[at])
    {
        path.push_back(at);
    }

    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<int> Graph::findSSSP_dijkstra(std::size_t u)
{
    std::vector<int> dist(this->m_countOfVertices, std::numeric_limits<int>::max());
    dist[u] = 0;
    std::priority_queue<std::pair<int, std::size_t>, std::vector<std::pair<int, std::size_t>>, std::greater<std::pair<int, std::size_t>>> pq;
    pq.push({0, u}); // pair<int, std::size_t> vertex , weight
    while(!pq.empty())
    {
        auto topElement = pq.top();
        int currentDist = topElement.first;
        std::size_t u = topElement.second;
        pq.pop();
        if(currentDist > dist[u]) continue;

        for(const auto& v : this->m_adjList[u])
        {
            int newDist = dist[u] + v.second;
            if(newDist < dist[v.first])
            {
                dist[v.first] = newDist;
                pq.push({newDist, v.first});
            }
        }
    }
    return dist;
}

