#pragma once

#include <pybind11/pybind11.h>

namespace py = pybind11;
void init_blob_detector(py::module &m);
