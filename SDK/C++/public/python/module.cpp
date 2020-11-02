
#include "module.hpp"

#include <voltu/initialise.hpp>
#include <voltu/types/errors.hpp>

namespace py = pybind11;

void py_exceptions(pybind11::module& m) {
	py::register_exception<voltu::exceptions::Exception>(m, "Error");
	py::register_exception<voltu::exceptions::BadImageChannel>(m, "ErrorBadImageChannel");
	py::register_exception<voltu::exceptions::NoFrame>(m, "ErrorNoFrame");
	py::register_exception<voltu::exceptions::AlreadyInit>(m, "ErrorAlreadyInit");
	py::register_exception<voltu::exceptions::BadSourceURI>(m, "ErrorBadSourceURI");
}

PYBIND11_MODULE(voltu, m) {
	py_exceptions(m);
	py_automatic_bindings(m);
}
