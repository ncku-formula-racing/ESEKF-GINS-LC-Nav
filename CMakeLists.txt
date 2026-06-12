cmake_minimum_required(VERSION 3.16)
project(ESEKF_GINS_LC_Nav VERSION 1.0.0 LANGUAGES C)

# compile_commands.json for clangd when developing the lib standalone;
# embedding projects decide this for themselves.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()

# CMSIS-DSP source tree.  Consumers embedding this library via
# add_subdirectory() may point this at their own copy (e.g. an STM32
# project's Drivers/CMSIS/DSP); defaults to the bundled submodule for
# standalone builds.
set(KF_GINS_CMSIS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/cmsis-dsp"
    CACHE PATH "Path to a CMSIS-DSP source tree")

set(CMSIS_SOURCES
    "${KF_GINS_CMSIS_ROOT}/Source/MatrixFunctions/MatrixFunctions.c"
    "${KF_GINS_CMSIS_ROOT}/Source/CommonTables/CommonTables.c"
    "${KF_GINS_CMSIS_ROOT}/Source/BasicMathFunctions/BasicMathFunctions.c"
)
set_source_files_properties(${CMSIS_SOURCES} PROPERTIES COMPILE_FLAGS "-Wno-pragmas")

add_library(kf_gins
    src/ekf/ekf.c
    src/ins/ins.c
    src/nav/nav.c
    src/utils/utils.c
    ${CMSIS_SOURCES}
)
target_include_directories(kf_gins PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    ${KF_GINS_CMSIS_ROOT}/Include
    ${KF_GINS_CMSIS_ROOT}/PrivateInclude
)
target_compile_features(kf_gins PRIVATE c_std_11)
target_compile_definitions(kf_gins PRIVATE ARM_MATH_LOOPUNROLL)

# Tests (opt-in)
option(KF_GINS_BUILD_TESTS "Build tests" OFF)
if(KF_GINS_BUILD_TESTS)
    add_subdirectory(tests)
endif()
