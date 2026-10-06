#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <complex>
#include <vector>
#include <cmath>

namespace py = pybind11;

py::tuple compute_h_signature(py::array_t<double> path_points,
                             py::array_t<double> obstacle_centers,
                             int decimals=1)
{
    auto path = path_points.unchecked<2>();       // shape: (N, 2)
    auto obstacles = obstacle_centers.unchecked<2>(); // shape: (M, 2)

    size_t N = path.shape(0);
    size_t M = obstacles.shape(0);

    // Initialize vector of complex winding sums for each obstacle
    std::vector<std::complex<double>> H(M, {0.0, 0.0});

    const std::complex<double> I(0.0, 1.0);
    const double TWO_PI = 2.0 * M_PI;

    for (size_t i = 0; i < N - 1; i++) {
        std::complex<double> z1(path(i,0), path(i,1));
        std::complex<double> z2(path(i+1,0), path(i+1,1));
        for (size_t k = 0; k < M; k++) {
            std::complex<double> zk(obstacles(k,0), obstacles(k,1));
            // Calculate complex logarithm of (z2 - zk) / (z1 - zk)
            std::complex<double> val = std::log((z2 - zk) / (z1 - zk));
            H[k] += val / (TWO_PI * I);
        }
    }

    // Round real parts and create python tuple
    py::tuple result(M);
    double factor = std::pow(10, decimals);

    for (size_t k = 0; k < M; k++) {
        double rounded_real = std::round(H[k].real() * factor) / factor;
        result[k] = rounded_real;
    }

    return result;
}

PYBIND11_MODULE(h_signature, m) {
    m.def("compute_h_signature", &compute_h_signature,
          py::arg("path_points"),
          py::arg("obstacle_centers"),
          py::arg("decimals") = 1,
          "Compute H-Signature (homotopy class) of a path around obstacles");
}
