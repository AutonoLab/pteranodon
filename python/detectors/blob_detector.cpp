#include "../cpp/include/headers/blob_detector.hpp"

#include <pybind11/stl.h>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

void init_blob_detector(py::module &m) {
    py::class_<BlobDetector>(m, "BlobDetector")
        .def(py::init<>())
        .def("detect", &BlobDetector::detect)
        .def("detectAnchor", &BlobDetector::detectAnchor);
}
