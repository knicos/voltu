
#include "module.hpp"

#include <voltu/voltu.hpp>
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
	m.attr("version") = py::make_tuple(VOLTU_VERSION_MAJOR, VOLTU_VERSION_MINOR, VOLTU_VERSION_PATCH);
	py_exceptions(m);
	py_automatic_bindings(m);
}
