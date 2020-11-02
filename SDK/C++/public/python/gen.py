#!/usr/bin/env python3

import sys
import os

from CppHeaderParser import CppHeader, CppParseError

# Extract line from file, as not directly with CppHeader
def read_line(file, lineno):
    with open(file) as f:
        for i, line in enumerate(f, 1):
            if i == lineno:
                return line


def get_loc_msg(data):
    return "({0}:{1})".format(data["filename"], data["line_number"])

def print_warn(msg, loc=None):
    if loc is not None:
        msg += " " + get_loc_msg(loc)
    print("WARNING: %s" % msg, file=sys.stderr)

def print_err(msg, loc=None):
    if loc is not None:
        msg += " " + get_loc_msg(loc)
    print("ERROR: %s" % msg, file=sys.stderr)

def include_in_api(data):
    return "PY_API" in data["debug"]

def create_enum_bindigs(enum, parent=[], pybind_handle="", export_values=False):
    name_full = parent + [enum["name"]]
    name_py = enum["name"]
    cpp = []
    cpp.append("py::enum_<{name_full}>({handle}, \"{name}\")".format(
        name_full = "::".join(name_full),
        handle = pybind_handle,
        name = name_py
    ))
    for val in enum["values"]:
        cpp.append("\t.value(\"{name}\", {name_cpp})".format(
            name = val["name"],
            name_cpp = "::".join(name_full + [val["name"]])
        ))

    if export_values:
        cpp.append("\t.export_values()")

    return "\n\t".join(cpp)

def create_function_bindings(func, parent=[], pybind_handle=None):
    func_name = func["name"]
    full_name = parent + [func_name]
    full_name_cpp = "::".join(full_name)

    if "PY_API" not in func["debug"]:
        print_err("%s not included in Python API" % full_name_cpp, func)
        raise ValueError("No PY_API")

    args = ["\"{0}\"".format(func_name), "&{0}".format(full_name_cpp)]

    for param in func["parameters"]:
        param_name = param["name"]

        if  param_name == "&":
            print_warn("Argument name missing for %s" % full_name_cpp, func)

            continue

        if "default" in param:
            args.append("py::arg(\"{0}\") = {1}".format(
                param_name, param["default"]))
        else:
            args.append("py::arg(\"{0}\")".format(param_name))

    if "PY_RV_LIFETIME_PARENT" in func["debug"]:
        if func["parent"] is None:
            print_err("PY_RV_LIFETIME_PARENT used for function", func)
            raise ValueError()

        args.append("py::return_value_policy::reference_internal")

    cpp = "def({0})".format(", ".join(args))
    if pybind_handle is not None:
        cpp = pybind_handle + "." + cpp

    return cpp

def create_class_bindings(cls, parent=[], pybind_handle=""):
    cls_name = cls["name"]
    full_name = parent + [cls_name]
    cpp = []

    if "PY_NO_SHARED_PTR" not in cls["debug"]:
        cls_cpp = "py::class_<{name}, std::shared_ptr<{name}>>({handle}, \"{name}\")"
    else:
        cls_cpp = "py::class_<{name}>({handle}, \"{name}\")"
    cpp.append(cls_cpp.format(handle=pybind_handle, name=cls_name))

    if cls["declaration_method"] == "struct":
        cpp.append(".def(py::init<>())")

    for method in cls["methods"]["public"]:
        if include_in_api(method):
            cpp.append("." + create_function_bindings(method, full_name))

    for field in cls["properties"]["public"]:
        if field["constant"]:
            field_cpp = ".def_property_readonly(\"{name}\", &{cpp_name})"
        else:
            field_cpp = ".def_readwrite(\"{name}\", &{cpp_name})"
        field_name = field["name"]
        field_name_cpp = "::".join(full_name + [field_name])
        cpp.append(field_cpp.format(name=field_name, cpp_name=field_name_cpp))

    return "\n\t\t".join(cpp)

if __name__ == "__main__":

    from pprint import pprint

    if (len(sys.argv) < 4):
        print("gen.py output include_directory input files ...")
        exit(1)

    handle = "m"
    fout = sys.argv[1]
    includedir = sys.argv[2]
    fsin = sys.argv[3:]

    out = []
    includes = []
    for fname in fsin:
        includes.append(fname)

        hdr = CppHeader(os.path.join(includedir, fname))
        # note .strip("::"), inconsistent behavior for classes vs enum/func

        for data in hdr.enums:
            ns = data["namespace"].strip("::") # bug? in parser
            out.append(create_enum_bindigs(data, [ns], handle) + ";")

        for data in hdr.classes.values():
            ns = data["namespace"]
            # workaround: parser does not save debug in same way as for funcs
            data["debug"] = read_line(data["filename"], data["line_number"])
            out.append(create_class_bindings(data, [ns], handle) + ";")

        for data in hdr.functions:
            ns = data["namespace"].strip("::") # bug? in parser
            if include_in_api(data):
                out.append(create_function_bindings(data, [ns], handle) + ";")

    includes = "\n".join("#include <{0}>".format(i) for i in includes)
    template_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), "automatic_bindings.cpp.in")
    with open(template_file, "r") as f:
        template = f.read()
    out = template.format(includes=includes, code="\n\t".join(out))
    with open(fout, "w",) as f:
        f.write(out)
