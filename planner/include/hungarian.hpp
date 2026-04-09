#pragma once
#include <vector>
#include <limits>
#include <cmath>
#include <algorithm>

// ============================================================
// Hungarian / Assignment (Min-Cost) for rectangular matrices
// - header-only
// - Complexity: O(n^3)
// - Based on classic potentials implementation for min cost assignment.
// - Works with nRows x nCols, returns assignment for rows:
//     row_to_col[i] = j (0-based), or -1 if unmatched (when you pad).
// ============================================================

class HungarianMinCost {
public:
    // Solve min-cost assignment for cost matrix (nRows x nCols).
    // If you want "unmatched", you should pad with big costs yourself.
    // Returns vector<int> row_to_col size nRows, -1 if no real assignment.
    static std::vector<int> Solve(const std::vector<std::vector<double>>& cost) {
        const int n = (int)cost.size();
        const int m = n ? (int)cost[0].size() : 0;
        if (n == 0 || m == 0) return {};

        // We solve for rectangular by requiring n<=m using transpose trick.
        // Here we implement a generic n x m solver where n<=m; if n>m, transpose.
        if (n <= m) {
            return solve_n_le_m(cost);
        } else {
            // transpose and solve, then invert mapping
            std::vector<std::vector<double>> ct(m, std::vector<double>(n, 0.0));
            for (int i = 0; i < n; ++i)
                for (int j = 0; j < m; ++j)
                    ct[j][i] = cost[i][j];

            std::vector<int> col_to_row = solve_n_le_m(ct); // size m: each "row" is original col
            // col_to_row[x] gives assigned column (original row index) for transposed row x
            // We want row_to_col for original rows:
            std::vector<int> row_to_col(n, -1);
            for (int col = 0; col < m; ++col) {
                int row = col_to_row[col];
                if (row >= 0 && row < n) row_to_col[row] = col;
            }
            return row_to_col;
        }
    }

private:
    // Solve when nRows <= nCols
    static std::vector<int> solve_n_le_m(const std::vector<std::vector<double>>& a) {
        const int n = (int)a.size();
        const int m = (int)a[0].size();

        const double INF = std::numeric_limits<double>::infinity();

        // 1-indexed arrays for classic implementation
        std::vector<double> u(n + 1, 0.0), v(m + 1, 0.0);
        std::vector<int> p(m + 1, 0), way(m + 1, 0);

        for (int i = 1; i <= n; ++i) {
            p[0] = i;
            int j0 = 0;
            std::vector<double> minv(m + 1, INF);
            std::vector<char> used(m + 1, false);

            do {
                used[j0] = true;
                int i0 = p[j0];
                int j1 = 0;
                double delta = INF;

                for (int j = 1; j <= m; ++j) {
                    if (used[j]) continue;
                    double cur = a[i0 - 1][j - 1] - u[i0] - v[j];
                    if (cur < minv[j]) {
                        minv[j] = cur;
                        way[j] = j0;
                    }
                    if (minv[j] < delta) {
                        delta = minv[j];
                        j1 = j;
                    }
                }

                for (int j = 0; j <= m; ++j) {
                    if (used[j]) {
                        u[p[j]] += delta;
                        v[j]    -= delta;
                    } else {
                        minv[j] -= delta;
                    }
                }
                j0 = j1;

            } while (p[j0] != 0);

            // augmenting
            do {
                int j1 = way[j0];
                p[j0] = p[j1];
                j0 = j1;
            } while (j0 != 0);
        }

        // p[j] = assigned row for column j
        std::vector<int> row_to_col(n, -1);
        for (int j = 1; j <= m; ++j) {
            if (p[j] >= 1 && p[j] <= n) {
                row_to_col[p[j] - 1] = j - 1;
            }
        }
        return row_to_col;
    }
};