#pragma once
#include <atomic>
#include <queue>
#include <vector>
#include <mutex>

class Graph {
public:
    explicit Graph(int vertices);
    Graph(Graph&& other) noexcept 
        : V(other.V)
        , adjList(std::move(other.adjList)) 
    {
        other.V = 0;
    }
    Graph(const Graph&) = delete;
    Graph& operator=(const Graph&) = delete;


    void addEdge(int src, int dest);
    void parallelBFS(int startVertex); // заглушка, как в Java
    void bfs(int startVertex);         // обычный BFS
    int vertices() const;

private:
    void bfs_task(std::queue<int>& q, std::vector<std::atomic<bool>> &visited, std::atomic<int> &init_lock);
    int V;
    std::mutex queue_mutex;
    std::vector<std::vector<int>> adjList;
};
