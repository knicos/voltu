#pragma once

#include <pybind11/pybind11.h>
#include <voltu/voltu.hpp>
#include <voltu/system.hpp>

class PyModule {
public:
    PyModule();
    void py_automatic_bindings(pybind11::module& m);
    void py_exceptions(pybind11::module& m);

private:
    std::shared_ptr<voltu::System> instance_System;
};
