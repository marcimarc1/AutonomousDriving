project(fvks)
cmake_minimum_required(VERSION 2.8)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

option(ADD_MODULE_REACH "module_reach" OFF)
option(ADD_MODULE_REACH_OLD "module_reach_old" OFF)
option(ADD_MODULE_COLLISION "module_collision" ON)
option(ADD_MODULE_GEOMETRY "module_geometry" OFF)
option(ADD_TEST "unit tests" OFF)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-fPIC -std=c++11 ${CMAKE_CXX_FLAGS}")
else()
    set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
endif()

SET(EIGEN3_INCLUDE_DIR "external/eigen" )
IF( NOT EIGEN3_INCLUDE_DIR )
    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
ENDIF()
INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )

include_directories( "cpp" )

if(ADD_TEST)
  add_subdirectory(tests)
  add_subdirectory(tests/gtests)
  add_subdirectory(external/gtest)
  add_subdirectory(cpp/test)
endif()

add_subdirectory(external/gtest)

set(REACH_SRC 
  cpp/reach/polyhedron2vrep.cpp
  cpp/reach/splitaarect.cpp
  cpp/reach/reachset.cpp)

set(GEOMETRY_SRC 
  cpp/geometry/isotrapcurvilinearcoordinates.cpp
  cpp/geometry/trapcurvilinearcoordinates.cpp)

if(ADD_MODULE_REACH_OLD)
  include_directories("/usr/local/include")
  link_directories("/usr/local/lib")
  set(REACH_SRC ${REACH_SRC}
      cpp/reach/unconstrained2dreach.cpp
      cpp/reach/box_connectivity.cpp
      cpp/reach/constrained1DReach.cpp
      cpp/reach/polyhedron2.cpp
      cpp/reach/reachbox.cpp
      cpp/reach/reachbox_set.cpp)
endif()

set(COLLISION_SRC cpp/collision/shapegroup.cc
                  cpp/collision/point.cc
                  cpp/collision/collision_checker.cc
                  cpp/collision/rectangle_aabb.cc
                  cpp/collision/rectangle_obb.cc
                  cpp/collision/shape.cc
                  cpp/collision/sphere.cc
                  cpp/collision/timevariant_collisionobject.cc
                  cpp/collision/boundingvolume.cc
                  cpp/collision/boundingvolume_util.cc
                  cpp/collision/trajectory.cc
                  cpp/collision/shapegroup.cc
                  cpp/collision/triangle.cc
                  cpp/collision/polygon.cc)

set(LIB_FVKS_SRC)

if(ADD_MODULE_REACH)
  set(LIB_FVKS_SRC ${LIB_FVKS_SRC} ${REACH_SRC})
endif()

if(ADD_MODULE_COLLISION)
  set(LIB_FVKS_SRC ${LIB_FVKS_SRC} ${COLLISION_SRC})
endif()

if(ADD_MODULE_GEOMETRY)
  set(LIB_FVKS_SRC ${LIB_FVKS_SRC} ${GEOMETRY_SRC})
endif()

set(LIB_FVKS_SRC ${LIB_FVKS_SRC} ${COLLISION_SRC})

add_library(fvks ${LIB_FVKS_SRC})

if(ADD_MODULE_REACH_OLD)
  target_link_libraries(fvks ppl gmp)
endif()


add_subdirectory(python_binding)
