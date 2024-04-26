cmake_minimum_required(VERSION 3.10.1)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-std=c++14")

### ----- Create shared library ----- ###
project(dynamics)

include_directories(
        include
)

add_library(${PROJECT_NAME} SHARED wrapper.c
        M_func.c C_func.c inverse_dynamics.c forward_dynamics.c
        kinematics.c kinematics_jacobian.c)

target_include_directories(${PROJECT_NAME} PUBLIC)

target_compile_options(${PROJECT_NAME} PUBLIC)