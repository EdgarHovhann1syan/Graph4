#include "../include/graph.h"
int main()
{
    Graph g(4, true);
    g.addEdge(0, 1, 5);
    g.addEdge(1, 2, 7);
    g.addEdge(0, 3, 1);
    g.addEdge(3, 2, 5);
    const auto dist = g.findSSSP_dijkstra(0);
    for(int i = 0; i < dist.size(); ++i)
    {
        std::cout << "from 0 to " << i << " = " << dist[i] << std::endl;
    }
    return 0;
}
