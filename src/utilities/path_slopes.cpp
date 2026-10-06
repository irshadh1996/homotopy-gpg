// path_slopes.cpp
#include <Python.h>
#include <vector>
#include <cmath>

// Helper function to normalize angle to [-pi, pi]
double normalize_angle(double angle) {
    angle = fmod(angle + M_PI, 2 * M_PI);
    if (angle < 0) angle += 2 * M_PI;
    return angle - M_PI;
}

// The main function exposed to Python
static PyObject* path_slopes_within_range(PyObject* self, PyObject* args) {
    PyObject* py_path_nodes;  // list of points (tuples/lists)
    double lower, upper;

    // Parse Python args: list, double, double
    if (!PyArg_ParseTuple(args, "Odd", &py_path_nodes, &lower, &upper)) {
        return NULL;
    }

    // Check if py_path_nodes is a list
    if (!PyList_Check(py_path_nodes)) {
        PyErr_SetString(PyExc_TypeError, "path_nodes must be a list");
        return NULL;
    }

    int n = PyList_Size(py_path_nodes);
    if (n < 2) {
        // If less than 2 points, no slopes, return True trivially
        Py_RETURN_TRUE;
    }

    // Extract points into C++ vector
    std::vector<std::pair<double, double>> pts;
    pts.reserve(n);
    for (int i = 0; i < n; ++i) {
        PyObject* point = PyList_GetItem(py_path_nodes, i);
        // Expecting a tuple or list of size 2
        if (!PyTuple_Check(point) && !PyList_Check(point)) {
            PyErr_SetString(PyExc_TypeError, "Each point must be a tuple or list");
            return NULL;
        }
        if (PySequence_Size(point) != 2) {
            PyErr_SetString(PyExc_ValueError, "Each point must have exactly two elements");
            return NULL;
        }
        PyObject* px = PySequence_GetItem(point, 0);
        PyObject* py = PySequence_GetItem(point, 1);
        double x = PyFloat_AsDouble(px);
        double y = PyFloat_AsDouble(py);
        Py_DECREF(px);
        Py_DECREF(py);
        if (PyErr_Occurred()) {
            return NULL;
        }
        pts.emplace_back(x, y);
    }

    // Normalize bounds
    double lb = normalize_angle(lower);
    double ub = normalize_angle(upper);

    // Compute slopes and check conditions
    for (int i = 0; i < n - 1; ++i) {
        double dx = pts[i+1].first - pts[i].first;
        double dy = pts[i+1].second - pts[i].second;
        double slope = std::atan2(dy, dx);
        slope = normalize_angle(slope);

        bool in_range;
        if (lb <= ub) {
            in_range = (slope >= lb) && (slope <= ub);
        } else {
            in_range = (slope >= lb) || (slope <= ub);
        }

        if (!in_range) {
            Py_RETURN_FALSE;
        }
    }

    Py_RETURN_TRUE;
}

// Method definition table
static PyMethodDef SlopesMethods[] = {
    {"path_slopes_within_range", path_slopes_within_range, METH_VARARGS, "Check if all path slopes are within range"},
    {NULL, NULL, 0, NULL}
};

// Module definition
static struct PyModuleDef slopesmodule = {
    PyModuleDef_HEAD_INIT,
    "slopes",   // module name
    NULL,       // module doc
    -1,         // size of per-interpreter state of the module
    SlopesMethods
};

// Module initialization function
PyMODINIT_FUNC PyInit_slopes(void) {
    return PyModule_Create(&slopesmodule);
}
