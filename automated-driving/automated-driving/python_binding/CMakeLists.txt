project(python_bindings)

option(ADD_MODULE_REACH "module_reach" ON)
option(ADD_MODULE_REACH_OLD "module_reach_old" OFF)
option(ADD_MODULE_COLLISION "module_collision" ON)
option(ADD_MODULE_GEOMETRY "module_geometry" ON)

if(NOT EXISTS "${PYTHON_INCLUDE_DIR}/Python.h")
  message(SEND_ERROR "Can't find Python.h in ${PYTHON_INCLUDE_DIR}")
endif()

# You can use CMake's FindPythonLib, but be sure to link to the correct 
# Python library in case you have several version installed

# path to Python headers ("Python.h")
set (PYTHON_INCLUDE_DIR ${PYTHON_INCLUDE_DIR})

#set (FVKS_LIBRARY /path/to/libfvks.a)
set (FVKS_LIBRARY fvks)

include(CheckCXXCompilerFlag)

string(TOUPPER "${CMAKE_BUILD_TYPE}" U_CMAKE_BUILD_TYPE)

if (CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  CHECK_CXX_COMPILER_FLAG("-std=c++14" HAS_CPP14_FLAG)
  CHECK_CXX_COMPILER_FLAG("-std=c++11" HAS_CPP11_FLAG)

  if (HAS_CPP14_FLAG)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
  elseif (HAS_CPP11_FLAG)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  else()
    message(FATAL_ERROR "Unsupported compiler -- pybind11 requires C++11 support!")
  endif()

  # Enable link time optimization and set the default symbol
  # visibility to hidden (very important to obtain small binaries)
  if (NOT ${U_CMAKE_BUILD_TYPE} MATCHES DEBUG)
    # Default symbol visibility
    #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden")

    # Check for Link Time Optimization support
    CHECK_CXX_COMPILER_FLAG("-flto" HAS_LTO_FLAG)
    if (HAS_LTO_FLAG)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -flto")
    endif()
  endif()
endif()


# Include path for Python header files
include_directories(${PYTHON_INCLUDE_DIR})

# Include path for pybind11 header files
include_directories(../external/pybind11/include)

# Include path for Eigen header files
include_directories(../external/eigen)

# Include path for fvks header files
include_directories(../cpp/)

#include_directories("/usr/local/include")
#link_directories("/usr/local/lib")

set(PYBIND11_HEADERS
  ../external/pybind11/include/pybind11/attr.h
  ../external/pybind11/include/pybind11/cast.h
  ../external/pybind11/include/pybind11/common.h
  ../external/pybind11/include/pybind11/complex.h
  ../external/pybind11/include/pybind11/descr.h
  ../external/pybind11/include/pybind11/functional.h
  ../external/pybind11/include/pybind11/numpy.h
  ../external/pybind11/include/pybind11/operators.h
  ../external/pybind11/include/pybind11/pybind11.h
  ../external/pybind11/include/pybind11/pytypes.h
  ../external/pybind11/include/pybind11/stl.h
  ../external/pybind11/include/pybind11/typeid.h
)

set(SRC_PY_WRAPPER)
set(COMPILE_DEFINTIONS_ADDITIONS)

if (ADD_MODULE_REACH)
  set(COMPILE_DEFINTIONS_ADDITIONS ${COMPILE_DEFINTIONS_ADDITIONS} 
                                   "PY_WRAPPER_MODULE_REACH=1 ")
  set(SRC_PY_WRAPPER ${SRC_PY_WRAPPER} "src/module_reach.cpp")
endif()

if (ADD_MODULE_REACH_OLD)
  set(COMPILE_DEFINTIONS_ADDITIONS ${COMPILE_DEFINTIONS_ADDITIONS} 
                                   "PY_WRAPPER_MODULE_REACH_OLD=1 ")
  set(SRC_PY_WRAPPER ${SRC_PY_WRAPPER} "src/module_reach_old.cpp")
  include_directories("/usr/local/include")
  link_directories("/usr/local/lib")
endif()

if (ADD_MODULE_COLLISION)
  set(COMPILE_DEFINTIONS_ADDITIONS ${COMPILE_DEFINTIONS_ADDITIONS}
                                   "PY_WRAPPER_MODULE_COLLISION=1 ")
  set(SRC_PY_WRAPPER ${SRC_PY_WRAPPER} "src/module_collision.cpp")
endif()

if (ADD_MODULE_GEOMETRY)
  set(COMPILE_DEFINTIONS_ADDITIONS ${COMPILE_DEFINTIONS_ADDITIONS}
                                   "PY_WRAPPER_MODULE_GEOMETRY=1 ")
  set(SRC_PY_WRAPPER ${SRC_PY_WRAPPER} "src/module_geometry.cpp")
endif()

# Create the binding library
add_library(pyfvks SHARED
  ${PYBIND11_HEADERS}
  src/py_bind.cpp
  ${SRC_PY_WRAPPER}
)


if (ADD_MODULE_REACH_OLD)
  target_link_libraries(pyfvks ${FVKS_LIBRARY} ppl gmp)
endif()

set_target_properties(pyfvks PROPERTIES COMPILE_DEFINITIONS
                                        "${COMPILE_DEFINTIONS_ADDITIONS}")

# Don't add a 'lib' prefix to the shared library
set_target_properties(pyfvks PROPERTIES PREFIX "")

if (UNIX)
  # .SO file extension on Linux/Mac OS
  set_target_properties(pyfvks PROPERTIES SUFFIX ".so")
  target_link_libraries(pyfvks ${FVKS_LIBRARY})

  # Optimize for a small binary size
  if (NOT ${U_CMAKE_BUILD_TYPE} MATCHES DEBUG)
    set_target_properties(pyfvks PROPERTIES COMPILE_FLAGS "-Os")
  endif()

  # Strip unnecessary sections of the binary on Linux/Mac OS
  if(APPLE)    
     set_target_properties(pyfvks PROPERTIES MACOSX_RPATH ".")
     set_target_properties(pyfvks PROPERTIES LINK_FLAGS "-undefined dynamic_lookup ")
  #   if (NOT ${U_CMAKE_BUILD_TYPE} MATCHES DEBUG)
  #     add_custom_command(TARGET pyfvks POST_BUILD COMMAND strip -u -r ${PROJECT_SOURCE_DIR}/pyfvks/pyfvks.so)
  #   endif()
  # else()
  #   if (NOT ${U_CMAKE_BUILD_TYPE} MATCHES DEBUG)
  #     add_custom_command(TARGET pyfvks POST_BUILD COMMAND strip ${PROJECT_SOURCE_DIR}/pyfvks/pyfvks.so)
  #   endif()
  endif()
endif()

message(STATUS "${PYTHON_INCLUDE_DIR}")
