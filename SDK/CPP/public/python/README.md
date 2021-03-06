# Python API generator

Dependencies
 * python3-dev
 * python3-ply

Build system uses Pybind11 to generate a python module. Most of the bindings are
automatically generated by gen.py script which is automatically called by CMake
when necessary.

Headers included for automatic binding generation are defined in
SDK_AUTO_HEADERS variable in CMakeLists.txt. Included pybind headers are defined
in automatic_bindings.cpp.in, which to which generated code is inserted before
build. Several (empty) macros are used in headers to annoate Python API details.

 * PY_API function/method is to be included in Python API
 * PY_NO_SHARED_PTR shared_ptr<> is not used with instances of this class.
   See [pybind11 documentation](https://pybind11.readthedocs.io/en/latest/advanced/smart_ptrs.html?#std-shared-ptr)
   for techncal details. Shared pointers are not used for structs.
 * PY_RV_LIFETIME_PARENT lifetime of method's return valued is tied to
   lifetime of parent objects (this). ([return_value_policy::reference_internal](https://pybind11.readthedocs.io/en/latest/advanced/functions.html#return-value-policies)
   is set for this method)
 * PY_SINGLETON Singleton class, methods are exported to to module scope.
 * PY_SINGLETON_OBJECT Singleton instance is accessible as module attribute.

## Notes:
 * Binding to default constructor is generated for structs. Class constructors
   are not available via Python at the moment.
 * Keyword arguments are supported in Python when function arguments are named
   in the header.
 * Default arguments are supported (extracted from header).
 * Public class properties are available in python, read-only if const,
   otherwise read write.
 * Singletons have to be created in PyModule constructor and be available
   as class members.
 * Exceptions have to be included manually in module.cpp
 * C++ preprocessor is not used

## Not supported (yet) in automatic binding generation:
 * Nested classes
 * Static members
 * Constructors for non-POD structs.
 * Automatic documentation (Doxygen)
 * Generator does not verify that shared_ptr<> is used consistently/correctly
 * Member variables of singleton classes
