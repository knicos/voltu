#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <voltu/types/channel.hpp>
#include <voltu/types/image.hpp>

#include <cstdint>
#include <exception>

namespace pybind11 {
namespace detail {

template<>
struct type_caster<voltu::ImageData> {
public:
    PYBIND11_TYPE_CASTER(voltu::ImageData, _("Image"));

    bool load(py::handle src, bool convert) {
        // TODO: C++ api should copy or otherwise take (shared) ownership
        throw std::runtime_error("ImageData conversion python->c++ not supported");
        return true;
    }

    static py::handle cast(const voltu::ImageData& src, py::return_value_policy policy, py::handle parent) {
        switch(src.format) {
            case voltu::ImageFormat::kFloat32:
                return py::array(
                    { size_t(src.height), size_t(src.width) },
                    { size_t(src.pitch), sizeof(float) },
                    reinterpret_cast<float*>(src.data)
                ).release();

            case voltu::ImageFormat::kBGRA8:
                return py::array(
                    { size_t(src.height), size_t(src.width), size_t(4) },
                    { size_t(src.pitch), size_t(4)*sizeof(uint8_t), size_t(1)*sizeof(uint8_t) },
                    reinterpret_cast<uint8_t*>(src.data)
                ).release();

            default:
                return py::array();
        }
    }
};

} // namespace detail
} // namespace pybind11

