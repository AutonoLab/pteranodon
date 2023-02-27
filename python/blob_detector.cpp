#include "../cpp/include/detectors/blob_detector.hpp"

#include <pybind11/stl.h>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

void init_blob_detector(py::module &m) {
    py::class_<BlobDetector>(m, "BlobDetector")
        .def(py::init<>(), py::arg("filter_blobs") = true, py::arg("merge_blobs") = false)
        .def("detect", &BlobDetector::detect)
        .def("detect", &BlobDetector::detectAnchor);
}
