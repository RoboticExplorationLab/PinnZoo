cmake_minimum_required(VERSION 3.10.1)

### ----- Generate dynamics for pendulum ----- ###
project(pendulum) # Change this for each model

set(DIR models/${PROJECT_NAME}/generated_code/) # Change if not the default

include_directories(${DIR})

file(GLOB SOURCES "${DIR}*.c") 

add_library(${PROJECT_NAME} SHARED src/wrapper.c ${SOURCES})

### ----- Generate dynamics for double pendulum ----- ###
project(double_pendulum) # Change this for each model

set(DIR models/${PROJECT_NAME}/generated_code/) # Change if not the default

include_directories(${DIR})

file(GLOB SOURCES "${DIR}*.c") 

add_library(${PROJECT_NAME} SHARED src/wrapper.c ${SOURCES})

### ----- Generate dynamics for cartpole ----- ###
project(cartpole) # Change this for each model

set(DIR models/${PROJECT_NAME}/generated_code/) # Change if not the default

include_directories(${DIR})

file(GLOB SOURCES "${DIR}*.c") 

add_library(${PROJECT_NAME} SHARED src/wrapper.c ${SOURCES})

### ----- Generate dynamics for double cartpole ----- ###
project(double_cartpole) # Change this for each model

set(DIR models/${PROJECT_NAME}/generated_code/) # Change if not the default

include_directories(${DIR})

file(GLOB SOURCES "${DIR}*.c") 

add_library(${PROJECT_NAME} SHARED src/wrapper.c ${SOURCES})

### ----- Generate dynamics for quadrotor ----- ###
project(quadrotor) # Change this for each model

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

### ----- Generate dynamics for Nadia simple knee 1 contact point per foot with quat orientation ----- ###
project(nadia_simple_1cp_quat) # Change this for each model

set(DIR models/nadia/generated_code/simple_1cp_quat/) # Change if not the default

include_directories(${DIR})

file(GLOB SOURCES "${DIR}*.c") 

add_library(${PROJECT_NAME} SHARED src/wrapper.c ${SOURCES})

### ----- Generate dynamics for Nadia simple knee 1 contact point per foot with axis angle orientation ----- ###
project(nadia_simple_1cp_aa) # Change this for each model

set(DIR models/nadia/generated_code/simple_1cp_aa/) # Change if not the default

include_directories(${DIR})

file(GLOB SOURCES "${DIR}*.c") 

add_library(${PROJECT_NAME} SHARED src/wrapper.c ${SOURCES})

### ----- Generate dynamics for Nadia simple knee 1 contact point per foot with axis angle orientation ----- ###
project(nadia_simple_1cp_aa_no_arms) # Change this for each model

set(DIR models/nadia/generated_code/simple_1cp_aa_no_arms/) # Change if not the default

include_directories(${DIR})

file(GLOB SOURCES "${DIR}*.c") 



add_library(${PROJECT_NAME} SHARED src/wrapper.c ${DIR}/custom_wrapper.c ${SOURCES})