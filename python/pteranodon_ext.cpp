#include <pybind11/pybind11.h>

namespace py = pybind11;

void init_motorcycle(py::module &);

namespace mcl {

PYBIND11_MODULE(pteranodon_ext, m) {
    m.doc() = "Pteranodon Extensions Library";
    
    init_blob_detector(m);
}

}