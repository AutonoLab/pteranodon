#include "../cpp/include/headers/blob_detector.hpp"
#include "blob_detector.hpp"

namespace py = pybind11;
void init_blob_detector(py::module &m) {
    py::class_<BlobDetector>(m, "BlobDetector")
        .def(py::init<>())
        .def("detect", &BlobDetector::detect)
        .def("detectAnchor", &BlobDetector::detectAnchor)
        .def("getBestScore", &BlobDetector::getBestScore);
}
