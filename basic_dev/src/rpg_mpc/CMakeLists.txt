cmake_minimum_required(VERSION 2.8.3)
project(rpg_mpc)

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_simple()

# C++ standard
if(CMAKE_COMPILER_IS_GNUCC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

# ARM NEON flags
if("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "armv7l")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
  message("enabling ARM neon optimizations")
endif()

# Build flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fopenmp -O3")

# Common include paths for ACADO, qpOASES, Eigen, and this package
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/model/quadrotor_mpc_codegen
  ${CMAKE_CURRENT_SOURCE_DIR}/externals/qpoases
  ${CMAKE_CURRENT_SOURCE_DIR}/externals/qpoases/INCLUDE
  ${CMAKE_CURRENT_SOURCE_DIR}/externals/qpoases/SRC
)

cs_add_library(mpc_solver
  externals/qpoases/SRC/Bounds.cpp
  externals/qpoases/SRC/Constraints.cpp
  externals/qpoases/SRC/CyclingManager.cpp
  externals/qpoases/SRC/Indexlist.cpp
  externals/qpoases/SRC/MessageHandling.cpp
  externals/qpoases/SRC/QProblem.cpp
  externals/qpoases/SRC/QProblemB.cpp
  externals/qpoases/SRC/SubjectTo.cpp
  externals/qpoases/SRC/Utils.cpp
  externals/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  model/quadrotor_mpc_codegen/acado_qpoases_interface.cpp
  model/quadrotor_mpc_codegen/acado_integrator.c
  model/quadrotor_mpc_codegen/acado_solver.c
  model/quadrotor_mpc_codegen/acado_auxiliary_functions.c
)

target_include_directories(mpc_solver PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/model/quadrotor_mpc_codegen
  ${CMAKE_CURRENT_SOURCE_DIR}/externals/qpoases
  ${CMAKE_CURRENT_SOURCE_DIR}/externals/qpoases/INCLUDE
  ${CMAKE_CURRENT_SOURCE_DIR}/externals/qpoases/SRC
  ${EIGEN3_INCLUDE_DIRS}
)

cs_add_library(mpc_wrapper
  src/mpc_wrapper.cpp
)

target_include_directories(mpc_wrapper PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/model/quadrotor_mpc_codegen
  ${CMAKE_CURRENT_SOURCE_DIR}/externals/qpoases
  ${CMAKE_CURRENT_SOURCE_DIR}/externals/qpoases/INCLUDE
  ${CMAKE_CURRENT_SOURCE_DIR}/externals/qpoases/SRC
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(mpc_wrapper
  mpc_solver
  ${catkin_LIBRARIES}
)

cs_add_library(mpc_controller
  src/mpc_controller.cpp
)

target_include_directories(mpc_controller PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/model/quadrotor_mpc_codegen
  ${CMAKE_CURRENT_SOURCE_DIR}/externals/qpoases
  ${CMAKE_CURRENT_SOURCE_DIR}/externals/qpoases/INCLUDE
  ${CMAKE_CURRENT_SOURCE_DIR}/externals/qpoases/SRC
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(mpc_controller
  mpc_wrapper
  mpc_solver
  ${catkin_LIBRARIES}
)

cs_install()
cs_export()
