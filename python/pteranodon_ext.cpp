#include <pybind11/pybind11.h>

#include <../extern/pybind11_opencv_numpy/ndarray_converter.h>

#include "./detectors/blob_detector.cpp"

namespace py = pybind11;

void pydef_cvnp(pybind11::module& m);

namespace mcl {

PYBIND11_MODULE(pteranodon_ext, m) {
    m.doc() = "Pteranodon Extensions Library";

    NDArrayConverter::init_numpy();
    
    init_blob_detector(m);

    pydef_cvnp(m);
}

}  // namespace mcl
