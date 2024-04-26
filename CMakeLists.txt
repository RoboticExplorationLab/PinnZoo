cmake_minimum_required(VERSION 3.10.1)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-std=c++14")

### ----- Generate dynamics for cartpole ----- ###
project(cartpole) # Change this for each model

set(DIR models/${PROJECT_NAME}/generated_code/) # Change if not the default

include_directories(${DIR})

file(GLOB SOURCES "${DIR}*.c") # Change this for each model

add_library(${PROJECT_NAME} SHARED src/wrapper.c ${SOURCES})