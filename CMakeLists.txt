cmake_minimum_required(VERSION 3.10)
project(ESEKF_GINS_LC_Nav VERSION 1.0.0)

set(CMAKE_C_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions(-DARM_MATH_LOOPUNROLL)

set(CMSIS_ROOT "${CMAKE_SOURCE_DIR}/thirdparty/cmsis-dsp")

set(CMSIS_SOURCES
    "${CMSIS_ROOT}/Source/MatrixFunctions/MatrixFunctions.c"
    "${CMSIS_ROOT}/Source/CommonTables/CommonTables.c"
    "${CMSIS_ROOT}/Source/BasicMathFunctions/BasicMathFunctions.c"
)
set_source_files_properties(${CMSIS_SOURCES} PROPERTIES COMPILE_FLAGS "-Wno-pragmas")

# Library target
file(GLOB_RECURSE LIB_SOURCES "src/*.c")
add_library(kf_gins ${LIB_SOURCES} ${CMSIS_SOURCES})
target_include_directories(kf_gins PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    ${CMSIS_ROOT}/Include
    ${CMSIS_ROOT}/PrivateInclude
    ${CMSIS_ROOT}/ComputeLibrary/Include
)

# Tests (opt-in)
option(KF_GINS_BUILD_TESTS "Build tests" OFF)
if(KF_GINS_BUILD_TESTS)
    add_subdirectory(tests)
endif()
