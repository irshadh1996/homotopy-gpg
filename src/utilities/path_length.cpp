#include <pybind11/pybind11.h>
#include <pybind11/stl.h>   // for std::vector and std::string conversion
#include <cmath>
#include <limits>

namespace py = pybind11;

// Compute Euclidean distance between two points (2D assumed)
double euclidean_distance(const std::vector<double>& p1, const std::vector<double>& p2) {
    double dx = p2[0] - p1[0];
    double dy = p2[1] - p1[1];
    return std::sqrt(dx*dx + dy*dy);
}

// Compute path length: path_nodes is a vector of node names (strings),
// node_labels is a dict<string, vector<double>>
double compute_path_length(
    const std::vector<std::string>& path_nodes,
    const py::dict& node_labels)
{
    double length = 0.0;
    for (size_t i = 1; i < path_nodes.size(); ++i) {
        std::string n1 = path_nodes[i-1];
        std::string n2 = path_nodes[i];

        // Lookup coordinates for n1
        py::object coord1_obj = node_labels[n1.c_str()];
        std::vector<double> coord1 = coord1_obj.cast<std::vector<double>>();

        // Lookup coordinates for n2
        py::object coord2_obj = node_labels[n2.c_str()];
        std::vector<double> coord2 = coord2_obj.cast<std::vector<double>>();

        length += euclidean_distance(coord1, coord2);
    }
    return length;
}

// The main function: shortest_paths
// node_labels: dict<string, vector<double>>
// all_paths: dict<string, list of list of strings>
py::dict shortest_paths(const py::dict& node_labels, const py::dict& all_paths) {
    py::dict shortest_paths_dict;

    for (auto item : all_paths) {
        // homotopy signature (hsig) as string
        std::string hsig = py::str(item.first);

        // all paths for this hsig: list of list of strings
        py::list paths = item.second.cast<py::list>();

        double min_length = std::numeric_limits<double>::max();
        py::list min_path;

        // Iterate over paths
        for (auto path_obj : paths) {
            // cast path to vector<string>
            std::vector<std::string> path_nodes = path_obj.cast<std::vector<std::string>>();

            double length = compute_path_length(path_nodes, node_labels);
            if (length < min_length) {
                min_length = length;
                min_path = path_obj.cast<py::list>();
            }
        }
        shortest_paths_dict[hsig.c_str()] = min_path;
    }

    return shortest_paths_dict;
}

PYBIND11_MODULE(path_length_cpp, m) {
    m.doc() = "Compute shortest paths by length from homotopy paths";

    m.def("shortest_paths", &shortest_paths,
          "Compute shortest paths from node_labels and all_paths");
}
