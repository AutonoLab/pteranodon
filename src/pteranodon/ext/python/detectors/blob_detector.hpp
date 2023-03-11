#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include "../../../../../extern/cvnp/cvnp/cvnp.h"

namespace py = pybind11;
void init_blob_detector(py::module &m);
