#include <pybind11/pybind11.h>

namespace py = pybind11;

void pydef_cvnp(pybind11::module& m);

void init_blob_detector(py::module &);

namespace mcl {

PYBIND11_MODULE(pteranodon_ext, m) {
    m.doc() = "Pteranodon Extensions Library";
    
    init_blob_detector(m);

    pydef_cvnp(m);
}

}  // namespace mcl
