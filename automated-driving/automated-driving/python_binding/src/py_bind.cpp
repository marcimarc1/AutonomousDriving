#include <pybind11/pybind11.h>

namespace py = pybind11;

#ifdef PY_WRAPPER_MODULE_COLLISION
void init_module_collision(py::module &m);
#endif

#ifdef PY_WRAPPER_MODULE_REACH
void init_module_reach(py::module &m);
#endif

#ifdef PY_WRAPPER_MODULE_GEOMETRY
void init_module_geometry(py::module &m);
#endif

#ifdef PY_WRAPPER_MODULE_REACH_OLD
void init_module_reach_old(py::module &m);
#endif

PYBIND11_PLUGIN(pyfvks) {
    py::module m("pyfvks", "fkvs python bindings");

    #ifdef PY_WRAPPER_MODULE_COLLISION
    py::module sub_module_collision = m.def_submodule("collision");
    init_module_collision(sub_module_collision);
    #endif

    #ifdef PY_WRAPPER_MODULE_REACH
    py::module sub_module_reach = m.def_submodule("reach");
    init_module_reach(sub_module_reach);
    #endif

    #ifdef PY_WRAPPER_MODULE_GEOMETRY
    py::module sub_module_geometry = m.def_submodule("geometry");
    init_module_geometry(sub_module_geometry);
    #endif

    #ifdef PY_WRAPPER_MODULE_REACH_OLD
    // TODO
    init_module_reach_old(m);
    #endif

    return m.ptr();
}