// GraphMST.h
#ifndef _GRAPH_H_
#define _GRAPH_H_

#include "tf/tf.h"
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <queue>
#include <utility>
#include <vector>

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <queue>
#include <utility>
#include <vector>

class GraphMST {
public:
    struct Edge {
        int u;
        int v;
        double w;
    };

    explicit GraphMST(int num_vertices)
        : num_vertices_(num_vertices) {}

    void clearEdges() { edges_.clear(); }

    void addEdge(int u, int v, double w) {
        edges_.push_back({u, v, w});
    }

    // 100% mimic python version:
    // - nodes are ONLY those incident to at least one edge
    // - if number_of_nodes < num_vertices => false
    // - if number_of_nodes == 0 => print and false
    // - else check connectivity
    bool is_graph_connected() const {
        if (num_vertices_ <= 0) return false;

        if (edges_.empty()) {
            std::cout << "\033[91m Communication graph is empty! \033[0m\n";
            return false;
        }

        std::vector<std::vector<int>> adj(num_vertices_);
        std::vector<bool> present(num_vertices_, false);

        int present_count = 0;
        for (const auto& e : edges_) {
            if (e.u < 0 || e.u >= num_vertices_ || e.v < 0 || e.v >= num_vertices_) {
                continue;
            }
            adj[e.u].push_back(e.v);
            adj[e.v].push_back(e.u);

            if (!present[e.u]) { present[e.u] = true; ++present_count; }
            if (!present[e.v]) { present[e.v] = true; ++present_count; }
        }

        if (present_count < num_vertices_) return false;

        if (present_count == 0) {
            std::cout << "\033[91m Communication graph is empty! \033[0m\n";
            return false;
        }

        std::vector<bool> visited(num_vertices_, false);
        std::queue<int> q;
        q.push(0);
        visited[0] = true;

        while (!q.empty()) {
            int cur = q.front();
            q.pop();
            for (int nxt : adj[cur]) {
                if (!visited[nxt]) {
                    visited[nxt] = true;
                    q.push(nxt);
                }
            }
        }

        for (int i = 0; i < num_vertices_; ++i) {
            if (!visited[i]) return false;
        }
        return true;
    }

    // Mimic python KruskalMST behavior:
    // - if not connected: print error and return ALL edges (u,v) in insertion order
    // - else: stable sort edges_ in-place by weight, then union-find
    bool KruskalMST(std::vector<std::vector<double>>& selected_edge) {
        selected_edge.clear();
        if (!is_graph_connected()) {
            std::cout << "\u001b[91m Communcation graph is not connected. "
                         "Failed to find minimum spainning tree! \u001b[0m\n";
            selected_edge.reserve(edges_.size());
            for (const auto& e : edges_) {
                selected_edge.push_back({double(e.u+1), double(e.v+1), e.w});
            }
            return false;
        }

        std::stable_sort(edges_.begin(), edges_.end(),
                         [](const Edge& a, const Edge& b) { return a.w < b.w; });

        parent_.assign(num_vertices_, 0);
        rank_.assign(num_vertices_, 0);
        for (int i = 0; i < num_vertices_; ++i) parent_[i] = i;

        std::vector<Edge> result;
        result.reserve(static_cast<std::size_t>(num_vertices_ - 1));

        std::size_t idx_edge = 0;
        int num_edge = 0;

        while (num_edge < num_vertices_ - 1 && idx_edge < edges_.size()) {
            const auto& e = edges_[idx_edge++];
            int x = find(parent_, e.u);
            int y = find(parent_, e.v);

            if (x != y) {
                ++num_edge;
                result.push_back(e);
                union_sets(parent_, rank_, x, y);
            }
        }

        selected_edge.reserve(result.size());
        for (const auto& e : result) {
            selected_edge.push_back({double(e.u), double(e.v), e.w});
        }
        return true;
    }

private:
    int num_vertices_;
    std::vector<Edge> edges_;

    std::vector<int> parent_;
    std::vector<int> rank_;

    int find(std::vector<int>& parent, int i) {
        if (parent[i] != i) parent[i] = find(parent, parent[i]);
        return parent[i];
    }

    void union_sets(std::vector<int>& parent, std::vector<int>& rank, int x, int y) {
        if (rank[x] < rank[y]) parent[x] = y;
        else if (rank[x] > rank[y]) parent[y] = x;
        else { parent[y] = x; rank[x] += 1; }
    }
};

#endif