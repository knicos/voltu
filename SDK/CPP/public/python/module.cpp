
#include "module.hpp"

#include <voltu/initialise.hpp>
#include <voltu/types/errors.hpp>

namespace py = pybind11;

PyModule::PyModule() {
    instance_System = voltu::instance();
}

void PyModule::py_exceptions(pybind11::module& m) {
	py::register_exception<voltu::exceptions::Exception>(m, "Error");
	py::register_exception<voltu::exceptions::BadImageChannel>(m, "ErrorBadImageChannel");
	py::register_exception<voltu::exceptions::NoFrame>(m, "ErrorNoFrame");
	py::register_exception<voltu::exceptions::AlreadyInit>(m, "ErrorAlreadyInit");
	py::register_exception<voltu::exceptions::BadSourceURI>(m, "ErrorBadSourceURI");
}

static auto module = PyModule();

PYBIND11_MODULE(voltu, m) {
	m.attr("version") = py::make_tuple(VOLTU_VERSION_MAJOR, VOLTU_VERSION_MINOR, VOLTU_VERSION_PATCH);

	module.py_exceptions(m);
	module.py_automatic_bindings(m);
}
