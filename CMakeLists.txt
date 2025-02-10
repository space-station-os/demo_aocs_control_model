cmake_minimum_required(VERSION 3.10)
project(aocs)

find_package(Eigen3 REQUIRED)

set(SRC_FILES
    src/iss_dynamics.cpp
    src/pd_controller.cpp
    src/L_p_func.cpp
)

add_executable(iss_dynamics ${SRC_FILES})
target_include_directories(iss_dynamics PRIVATE include ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(iss_dynamics Eigen3::Eigen)
