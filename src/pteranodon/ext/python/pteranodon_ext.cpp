#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include "../../../../extern/cvnp/cvnp/cvnp.h"

#include "converters/converters.hpp"
#include "detectors/blob_detector.hpp"

namespace py = pybind11;

void pydef_cvnp(pybind11::module& m);

PYBIND11_MODULE(_ext, m) {
    m.doc() = "Pteranodon Extensions Library";

    init_blob_detector(m);

    pydef_cvnp(m);
}
