"""
Build the C++ extensions used by the planner.

Usage (from the repo root):
    python setup.py build_ext --inplace

The compiled modules are placed in src/utilities/.
"""
from setuptools import setup, Extension
from pybind11.setup_helpers import Pybind11Extension, build_ext

U = "src/utilities"

ext_modules = [
    Pybind11Extension("utilities.add_points",      [f"{U}/add_points.cpp"]),
    Pybind11Extension("utilities.graph_builder",   [f"{U}/graph_builder_new.cpp"]),
    Pybind11Extension("utilities.h_signature",     [f"{U}/h_signature.cpp"]),
    Pybind11Extension("utilities.path_length_cpp", [f"{U}/path_length.cpp"]),
    Pybind11Extension("utilities.scalar_rank",     [f"{U}/scalar_rank_debug.cpp"]),
    Extension("utilities.slopes",                  [f"{U}/path_slopes.cpp"], language="c++"),
]

setup(
    name="homotopy_gpg_extensions",
    package_dir={"": "src"},
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
)