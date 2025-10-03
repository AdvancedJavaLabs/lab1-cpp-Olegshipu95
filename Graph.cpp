#include "Graph.h"
#include <algorithm>
#include <atomic>
#include <functional>
#include <queue>
#include <thread>

Graph::Graph(int vertices) : V(vertices), adjList(vertices) {}

void Graph::addEdge(int src, int dest) {
    if (src < 0 || dest < 0 || src >= V || dest >= V) return;
    auto& vec = adjList[src];
    if (std::find(vec.begin(), vec.end(), dest) == vec.end()) {
        vec.push_back(dest);
    }
}


void Graph::bfs_task(std::queue<int>& q, std::vector<std::atomic<bool>> &visited, std::atomic<int>& init_lock) {

    while (true) {
        int current;
        {
            std::lock_guard<std::mutex> lock(queue_mutex);

            // Вот тут могут быть проблемы, но их чет тяжко исправить без костыля с нормальной производительностью.

            if (q.empty()) {

                if (init_lock.load()) {
                    continue;
                }

                    return;
            }
            current = q.front();
            q.pop();
        }

        for (int neighbor : adjList[current]) {
            bool expected = false;
            if (visited[neighbor].compare_exchange_strong(expected, true)) {
                std::lock_guard<std::mutex> lock(queue_mutex);
                q.push(neighbor);
            }
        }
    }
}


void Graph::parallelBFS(int startVertex) {

    if (startVertex < 0 || startVertex >= V) return;

    std::vector<std::atomic<bool>> visited(V);
    for (int i = 0; i < V; ++i) {
        visited[i].store(false);
    }

    visited[startVertex] = true;
    std::queue<int> q;
    q.push(startVertex);

    unsigned num_threads = std::thread::hardware_concurrency();
    if (num_threads == 0) num_threads = 2;
    std::vector<std::thread> threads;


    std::atomic<int> init_lock;
    init_lock.store(1);

    for (unsigned i = 0; i < num_threads; ++i) {

        threads.emplace_back(&Graph::bfs_task, this, std::ref(q),
                             std::ref(visited), std::ref(init_lock));

    }

    init_lock.store(0);

    for (auto& t : threads) {
        t.join();
    }
}




void Graph::bfs(int startVertex) {
    if (startVertex < 0 || startVertex >= V) return;
    std::vector<char> visited(V, 0);
    std::queue<int> q;

    visited[startVertex] = 1;
    q.push(startVertex);

    while (!q.empty()) {
        int u = q.front();
        q.pop();
        for (int n : adjList[u]) {
            if (!visited[n]) {
                visited[n] = 1;
                q.push(n);
            }
        }
    }
}

int Graph::vertices() const { return V; }
