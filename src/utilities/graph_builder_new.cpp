#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <cmath>
#include <unordered_map>
#include <vector>
#include <string>
#include <set>

namespace py = pybind11;
using Vec2 = std::vector<double>;

bool point_inside_circle(const Vec2& p, const Vec2& center, double radius) {
    double dx = p[0] - center[0];
    double dy = p[1] - center[1];
    return std::sqrt(dx * dx + dy * dy) < radius;
}

bool line_intersects_circle(const Vec2& p1, const Vec2& p2, const Vec2& center, double radius) {
    double dx = p2[0] - p1[0];
    double dy = p2[1] - p1[1];
    double fx = p1[0] - center[0];
    double fy = p1[1] - center[1];

    double a = dx*dx + dy*dy;
    double b = 2 * (fx*dx + fy*dy);
    double c = fx*fx + fy*fy - radius*radius;

    double discriminant = b*b - 4*a*c;
    if (discriminant < 0) return false;
    discriminant = std::sqrt(discriminant);

    double t1 = (-b - discriminant) / (2*a);
    double t2 = (-b + discriminant) / (2*a);

    return !(t2 < 0 || t1 > 1);
}

bool line_intersects_any_circle(const Vec2& p1, const Vec2& p2, const std::vector<Vec2>& obstacles, double radius) {
    for (const auto& obs : obstacles) {
        if (line_intersects_circle(p1, p2, obs, radius)) return true;
    }
    return false;
}

bool is_duplicate(const Vec2& point, const std::vector<Vec2>& points, double tol = 1e-6) {
    for (const auto& p : points) {
        double dx = point[0] - p[0];
        double dy = point[1] - p[1];
        if (std::sqrt(dx * dx + dy * dy) < tol) return true;
    }
    return false;
}

std::pair<std::unordered_map<std::string, std::vector<std::string>>,
          std::unordered_map<std::string, Vec2>>
generate_graph(const Vec2& x_start,
               const Vec2& x_goal,
               const std::vector<Vec2>& obstacles,
               double o_radius) {

    std::unordered_map<std::string, Vec2> node_labels;
    std::unordered_map<std::string, Vec2> valid_nodes;
    std::vector<Vec2> all_points;

    node_labels["Start"] = x_start;
    node_labels["Goal"] = x_goal;
    valid_nodes["Start"] = x_start;
    valid_nodes["Goal"] = x_goal;
    all_points.push_back(x_start);
    all_points.push_back(x_goal);

    for (size_t i = 0; i < obstacles.size(); ++i) {
        const Vec2& obs = obstacles[i];
        double dx = x_start[0] - obs[0];
        double dy = x_start[1] - obs[1];
        double angle = std::atan2(dy, dx);
        double offset = o_radius * 3;

        Vec2 perp1 = {
            obs[0] + offset * std::cos(angle + M_PI_2),
            obs[1] + offset * std::sin(angle + M_PI_2)
        };
        Vec2 perp2 = {
            obs[0] + offset * std::cos(angle - M_PI_2),
            obs[1] + offset * std::sin(angle - M_PI_2)
        };

        for (int pidx = 0; pidx < 2; ++pidx) {
            Vec2 point = (pidx == 0) ? perp1 : perp2;
            std::string tag_base = "O" + std::to_string(i) + (pidx == 0 ? "_P1" : "_P2");
            std::string tag = tag_base;
            bool valid = true;

            // Check if point is inside any obstacle
            for (size_t j = 0; j < obstacles.size(); ++j) {
                if (point_inside_circle(point, obstacles[j], o_radius)) {
                    // Try midpoint alternative
                    Vec2 midpoint = {
                        (obstacles[i][0] + obstacles[j][0]) / 2.0,
                        (obstacles[i][1] + obstacles[j][1]) / 2.0
                    };
                    bool inside_any = false;
                    for (const auto& other : obstacles) {
                        if (point_inside_circle(midpoint, other, o_radius)) {
                            inside_any = true;
                            break;
                        }
                    }
                    if (!inside_any) {
                        point = midpoint;
                        tag += "_alt";
                    } else {
                        valid = false;
                    }
                    break;
                }
            }

            if (valid && !is_duplicate(point, all_points)) {
                valid_nodes[tag] = point;
                node_labels[tag] = point;
                all_points.push_back(point);
            }
        }
    }

    std::vector<std::string> nodes;
    for (const auto& kv : valid_nodes) {
        nodes.push_back(kv.first);
    }

    std::unordered_map<std::string, std::vector<std::string>> adj;
    for (const auto& node : nodes) {
        adj[node] = {};
    }

    for (size_t i = 0; i < nodes.size(); ++i) {
        for (size_t j = i + 1; j < nodes.size(); ++j) {
            const auto& p1 = valid_nodes[nodes[i]];
            const auto& p2 = valid_nodes[nodes[j]];
            if (!line_intersects_any_circle(p1, p2, obstacles, o_radius)) {
                adj[nodes[i]].push_back(nodes[j]);
                adj[nodes[j]].push_back(nodes[i]);
            }
        }
    }

    return {adj, node_labels};
}

PYBIND11_MODULE(graph_builder, m) {
    m.doc() = "Graph builder with obstacle avoidance";
    m.def("generate_graph", &generate_graph, "Generate graph nodes and edges avoiding circular obstacles");
}
