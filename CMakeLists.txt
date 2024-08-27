cmake_minimum_required(VERSION 3.10.1)

### ----- Generate dynamics for cartpole ----- ###
project(cartpole) # Change this for each model

set(DIR models/${PROJECT_NAME}/generated_code/) # Change if not the default

include_directories(${DIR})

file(GLOB SOURCES "${DIR}*.c") 

add_library(${PROJECT_NAME} SHARED src/wrapper.c ${SOURCES})

### ----- Generate dynamics for Unitree Go1 ----- ###
project(unitree_go1) # Change this for each model

set(DIR models/${PROJECT_NAME}/generated_code/) # Change if not the default

include_directories(${DIR})

file(GLOB SOURCES "${DIR}*.c") 

add_library(${PROJECT_NAME} SHARED src/wrapper.c ${SOURCES})

### ----- Generate dynamics for Unitree Go2 ----- ###
project(unitree_go2) # Change this for each model

set(DIR models/${PROJECT_NAME}/generated_code/) # Change if not the default

include_directories(${DIR})

file(GLOB SOURCES "${DIR}*.c") 

add_library(${PROJECT_NAME} SHARED src/wrapper.c ${SOURCES})

### ----- Generate dynamics for Nadia simple knee 1 contact point per foot ----- ###
project(nadia_simple_1cp) # Change this for each model

set(DIR models/nadia/generated_code/simple_1cp/) # Change if not the default

include_directories(${DIR})

file(GLOB SOURCES "${DIR}*.c") 

add_library(${PROJECT_NAME} SHARED src/wrapper.c ${SOURCES})

### ----- Generate dynamics for Nadia simple knee 4 contact point per foot ----- ###
project(nadia_simple_4cp) # Change this for each model

set(DIR models/nadia/generated_code/simple_4cp/) # Change if not the default

include_directories(${DIR})

file(GLOB SOURCES "${DIR}*.c") 

add_library(${PROJECT_NAME} SHARED src/wrapper.c ${SOURCES})