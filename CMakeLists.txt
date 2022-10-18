cmake_minimum_required(VERSION 3.0)
project(generatesteps)

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(Python3 COMPONENTS NumPy)
find_package(PythonLibs REQUIRED)
find_package(Eigen3)
find_package(Glog REQUIRED)
find_package(FootstepPlanner REQUIRED)
include_directories(
  ${PYTHON_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
)

link_libraries(FootstepPlanner)
add_executable(generatesteps main.cpp foot_step_planning_ljh.cpp)

target_link_libraries(
  generatesteps
  ${PYTHON_LIBRARIES}
  glog::glog
)