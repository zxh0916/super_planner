cmake_minimum_required(VERSION 3.18)
project(super_rosless LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type")

get_filename_component(REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

add_compile_definitions(
  ORIGIN_AT_CORNER
  FMT_HEADER_ONLY
  BACKWARD_HAS_DW=0
  BACKWARD_HAS_BFD=0
  BACKWARD_HAS_DWARF=0
  BACKWARD_HAS_BACKTRACE_SYMBOL=0
  ROOT_DIR="${CMAKE_CURRENT_LIST_DIR}/"
)

find_package(Eigen3 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)

execute_process(
  COMMAND "${Python3_EXECUTABLE}" -m pybind11 --cmakedir
  OUTPUT_VARIABLE PYBIND11_CMAKE_DIR
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
list(APPEND CMAKE_PREFIX_PATH "${PYBIND11_CMAKE_DIR}")
find_package(pybind11 CONFIG REQUIRED)

set(ROG_MAP_SOURCES
  "${REPO_ROOT}/rog_map/src/rog_map/counter_map.cpp"
  "${REPO_ROOT}/rog_map/src/rog_map/esdf_map.cpp"
  "${REPO_ROOT}/rog_map/src/rog_map/inf_map.cpp"
  "${REPO_ROOT}/rog_map/src/rog_map/prob_map.cpp"
  "${REPO_ROOT}/rog_map/src/rog_map/rog_map.cpp"
  "${REPO_ROOT}/rog_map/src/rog_map/sliding_map.cpp"
  "${REPO_ROOT}/rog_map/include/rog_map/rog_map_core/raycaster.cpp"
)

set(SUPER_SOURCES
  "${REPO_ROOT}/super_planner/src/super_core/astar.cpp"
  "${REPO_ROOT}/super_planner/src/super_core/ciri.cpp"
  "${REPO_ROOT}/super_planner/src/super_core/corridor_generator.cpp"
  "${REPO_ROOT}/super_planner/src/super_core/super_planner.cpp"
  "${REPO_ROOT}/super_planner/src/traj_opt/backup_traj_optimizer_s4.cpp"
  "${REPO_ROOT}/super_planner/src/traj_opt/exp_traj_optimizer_s4.cpp"
  "${REPO_ROOT}/super_planner/src/traj_opt/yaw_traj_opt.cpp"
  "${REPO_ROOT}/super_planner/src/utils/banded_system.cpp"
  "${REPO_ROOT}/super_planner/src/utils/ellipsoid.cpp"
  "${REPO_ROOT}/super_planner/src/utils/geometry_utils.cpp"
  "${REPO_ROOT}/super_planner/src/utils/lbfgs.cpp"
  "${REPO_ROOT}/super_planner/src/utils/minco.cpp"
  "${REPO_ROOT}/super_planner/src/utils/mvie.cpp"
  "${REPO_ROOT}/super_planner/src/utils/optimization_utils.cpp"
  "${REPO_ROOT}/super_planner/src/utils/piece.cpp"
  "${REPO_ROOT}/super_planner/src/utils/polynomial_interpolation.cpp"
  "${REPO_ROOT}/super_planner/src/utils/polytope.cpp"
  "${REPO_ROOT}/super_planner/src/utils/quickhull.cpp"
  "${REPO_ROOT}/super_planner/src/utils/root_finder.cpp"
  "${REPO_ROOT}/super_planner/src/utils/sdlp.cpp"
  "${REPO_ROOT}/super_planner/src/utils/trajectory.cpp"
  "${REPO_ROOT}/super_planner/src/utils/waypoint_trajectory_optimizer.cpp"
)

add_library(super_rosless_core STATIC
  ${ROG_MAP_SOURCES}
  ${SUPER_SOURCES}
  src/planner_session.cpp
)

target_include_directories(super_rosless_core PUBLIC
  "${CMAKE_CURRENT_LIST_DIR}/include"
  "${REPO_ROOT}/super_planner/include"
  "${REPO_ROOT}/rog_map/include"
  ${EIGEN3_INCLUDE_DIRS}
)

target_compile_options(super_rosless_core PRIVATE
  -O3
  -Wall
  -Wno-deprecated-declarations
  "SHELL:-include ${CMAKE_CURRENT_LIST_DIR}/include/super_core_py/compat.hpp"
)

pybind11_add_module(super_planner_py src/bindings.cpp)
target_link_libraries(super_planner_py PRIVATE super_rosless_core)
target_include_directories(super_planner_py PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/include"
  "${REPO_ROOT}/super_planner/include"
  "${REPO_ROOT}/rog_map/include"
  ${EIGEN3_INCLUDE_DIRS}
)
