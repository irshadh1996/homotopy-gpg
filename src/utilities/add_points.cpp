#include <pybind11/pybind11.h>
#include <pybind11/stl.h>  // for automatic STL conversion
#include <vector>
#include <array>

namespace py = pybind11;
using Point2D = std::array<double, 2>;

std::vector<Point2D> addIntermediatePoints(const std::vector<Point2D>& points, int n_points = 5) {
    std::vector<Point2D> new_points;

    for (size_t i = 0; i + 1 < points.size(); ++i) {
        const Point2D& start = points[i];
        const Point2D& end = points[i + 1];

        for (int j = 0; j < n_points + 1; ++j) {
            double t = static_cast<double>(j) / (n_points + 1);
            Point2D intermediate {
                start[0] + t * (end[0] - start[0]),
                start[1] + t * (end[1] - start[1])
            };
            new_points.push_back(intermediate);
        }
    }

    if (!points.empty()) {
        new_points.push_back(points.back());
    }

    return new_points;
}

PYBIND11_MODULE(add_points, m) {
    m.doc() = "Add intermediate points between 2D points";

    m.def("addIntermediatePoints", &addIntermediatePoints,
          py::arg("points"),
          py::arg("n_points") = 5,
          "Add intermediate points between pairs of 2D points");
}
