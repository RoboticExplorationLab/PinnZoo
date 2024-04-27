cmake_minimum_required(VERSION 3.10.1)

### ----- Generate dynamics for cartpole ----- ###
project(cartpole) # Change this for each model

set(DIR models/${PROJECT_NAME}/generated_code/) # Change if not the default

include_directories(${DIR})

file(GLOB SOURCES "${DIR}*.c") # Change this for each model

add_library(${PROJECT_NAME} SHARED src/wrapper.c ${SOURCES})

### ----- Generate dynamics for Unitree Go1 ----- ###
project(unitree_go1) # Change this for each model

set(DIR models/${PROJECT_NAME}/generated_code/) # Change if not the default

include_directories(${DIR})

file(GLOB SOURCES "${DIR}*.c") # Change this for each model

add_library(${PROJECT_NAME} SHARED src/wrapper.c ${SOURCES})