#include <pybind11/pybind11.h>

#include "converters/converters.hpp"
#include "detectors/blob_detector.hpp"

namespace py = pybind11;

namespace mcl {

PYBIND11_MODULE(pteranodon_ext, m) {
    NDArrayConverter::init_numpy();

    m.doc() = "Pteranodon Extensions Library";

    init_blob_detector(m);
}

}  // namespace mcl
