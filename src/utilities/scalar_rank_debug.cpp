#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <vector>
#include <cmath>
#include <utility>
#include <algorithm>
#include <tuple>
#include <unordered_set>
#include <stdexcept>

namespace py = pybind11;

using Point = std::pair<double, double>;
using Trajectory = std::vector<Point>;

double distance(const Point& a, const Point& b) {
    return std::hypot(b.first - a.first, b.second - a.second);
}

std::pair<Trajectory, double> resample_trajectory(const Trajectory& traj, int num_points = 100) {
    if (traj.empty()) {
        // If empty, return num_points copies of origin
        return {Trajectory(num_points, {0.0, 0.0}), 0.0};
    }
    if (traj.size() == 1) {
        // Single point trajectory
        return {Trajectory(num_points, traj[0]), 0.0};
    }

    std::vector<double> cumdist(traj.size(), 0.0);
    for (size_t i = 1; i < traj.size(); ++i) {
        cumdist[i] = cumdist[i-1] + distance(traj[i-1], traj[i]);
    }

    double total_length = cumdist.back();
    if (total_length == 0.0) {
        return {Trajectory(num_points, traj[0]), 0.0};
    }

    Trajectory resampled;
    double step = total_length / (num_points - 1);
    size_t j = 0;

    for (int i = 0; i < num_points; ++i) {
        double target = i * step;

        while (j < cumdist.size() - 2 && cumdist[j+1] < target) {
            j++;
        }

        double segment_length = cumdist[j+1] - cumdist[j];
        double t = 0.0;
        if (segment_length > 1e-8) {
            t = (target - cumdist[j]) / segment_length;
        }
        const Point& p1 = traj[j];
        const Point& p2 = traj[j+1];
        Point interp = {
            p1.first + t * (p2.first - p1.first),
            p1.second + t * (p2.second - p1.second)
        };
        resampled.push_back(interp);
    }

    return {resampled, total_length};
}

void dfs(int node, const std::vector<std::vector<int>>& adj, std::vector<bool>& visited, std::vector<int>& cluster) {
    visited[node] = true;
    cluster.push_back(node);
    for (int neighbor : adj[node]) {
        if (!visited[neighbor]) {
            dfs(neighbor, adj, visited, cluster);
        }
    }
}

double compute_proximity_penalty(const std::vector<Trajectory>& trajs, double threshold = 1.0) {
    if (trajs.empty()) {
        return 0.0;
    }
    int num_trajs = trajs.size();
    int num_points = trajs[0].size();

    // Check all trajectories have the same number of points
    for (const auto& traj : trajs) {
        if ((int)traj.size() != num_points) {
            throw std::invalid_argument("All trajectories must have the same number of points.");
        }
    }

    double penalty = 0.0;

    for (int t = 0; t < num_points; ++t) {
        // Points at time t
        std::vector<Point> points_at_t(num_trajs);
        for (int i = 0; i < num_trajs; ++i) {
            points_at_t[i] = trajs[i][t];
        }

        // Build adjacency based on distance threshold
        std::vector<std::vector<int>> adj(num_trajs);
        std::vector<std::tuple<int, int, double>> edges;

        for (int i = 0; i < num_trajs; ++i) {
            for (int j = i + 1; j < num_trajs; ++j) {
                double dist = distance(points_at_t[i], points_at_t[j]);
                if (dist < threshold) {
                    adj[i].push_back(j);
                    adj[j].push_back(i);
                    edges.emplace_back(i, j, dist);
                }
            }
        }

        // Find connected components (clusters) at time t
        std::vector<bool> visited(num_trajs, false);
        for (int i = 0; i < num_trajs; ++i) {
            if (!visited[i]) {
                std::vector<int> cluster;
                dfs(i, adj, visited, cluster);

                if (cluster.size() >= 2) {
                    double cluster_penalty = 0.0;
                    std::unordered_set<int> cluster_set(cluster.begin(), cluster.end());

                    // Sum squared penalties of edges inside cluster
                    for (auto& [u, v, dist] : edges) {
                        if (cluster_set.count(u) && cluster_set.count(v)) {
                            double diff = threshold - dist;
                            cluster_penalty += diff * diff;
                        }
                    }

                    // Apply cluster size multiplier (cubic penalty)
                    double multiplier = std::pow(cluster.size() - 1, 5);
                    penalty += multiplier * cluster_penalty;
                }
            }
        }
    }
    return penalty;
}

// New: Compute smoothness penalty for a single trajectory (sum squared turning angles)
double compute_smoothness_penalty(const Trajectory& traj) {
    double penalty = 0.0;
    for (size_t i = 1; i + 1 < traj.size(); ++i) {
        const auto& p0 = traj[i-1];
        const auto& p1 = traj[i];
        const auto& p2 = traj[i+1];
        double dx1 = p1.first - p0.first;
        double dy1 = p1.second - p0.second;
        double dx2 = p2.first - p1.first;
        double dy2 = p2.second - p1.second;
        double dot = dx1 * dx2 + dy1 * dy2;
        double mag1 = std::hypot(dx1, dy1);
        double mag2 = std::hypot(dx2, dy2);
        if (mag1 > 1e-8 && mag2 > 1e-8) {
            double cos_angle = dot / (mag1 * mag2);
            cos_angle = std::clamp(cos_angle, -1.0, 1.0);
            double angle = std::acos(cos_angle);
            penalty += angle * angle;  // sum squared angle change
        }
    }
    return penalty;
}

std::tuple<double, std::vector<Trajectory>> scalar_rank_combination(
    const std::vector<Trajectory>& comb,
    int num_points,
    double lambda_length,
    double lambda_weight,
    double proximity_threshold,
    double lambda_smoothness)
{
    std::vector<Trajectory> resampled_trajs;
    double total_length = 0.0;
    double total_smoothness = 0.0;

    for (const auto& traj : comb) {
        auto [resampled, length] = resample_trajectory(traj, num_points);
        resampled_trajs.push_back(resampled);
        total_length += length;
        total_smoothness += compute_smoothness_penalty(resampled);
    }

    double proximity_penalty = compute_proximity_penalty(resampled_trajs, proximity_threshold);
    double score = lambda_length * total_length + lambda_weight * proximity_penalty + lambda_smoothness * total_smoothness;

    return std::make_tuple(score, resampled_trajs);
}

// Pybind11 module
PYBIND11_MODULE(scalar_rank, m) {
    m.doc() = "Trajectory ranking with proximity and smoothness penalties";

    m.def("scalar_rank_combination", &scalar_rank_combination,
        py::arg("comb"),
        py::arg("num_points") = 100,
        py::arg("lambda_length") = 1.0,
        py::arg("lambda_weight") = 1.0,
        py::arg("proximity_threshold") = 1.0,
        py::arg("lambda_smoothness") = 0.5,
        "Compute scalar rank and return resampled trajectories"
    );
}
