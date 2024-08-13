cmake_minimum_required(VERSION 3.5)
project(mppi_ipddp)
find_package (Eigen3 REQUIRED NO_MODULE)
find_package(OpenMP REQUIRED)

# autodiff requires a c++17 enabled compiler
set(CMAKE_CXX_STANDARD 17)          # ensure cmake instructs compiler to use C++17
set(CMAKE_CXX_STANDARD_REQUIRED ON) # ensure the C++ standard given before is actually used
set(CMAKE_CXX_EXTENSIONS OFF)       # avoid compile flags of the type -std=gnu++1z added by cmake

include_directories(
    EigenRand/
    autodiff/
    ${EIGEN3_INCLUDE_DIRS}

    mppi/mppi/
    corridor/
    ipddp/ipddp/
    mppi_ipddp/
    model/
)

set(COMMON_SOURCES
    src/test_with_map.cpp
    src/log_mppi.cpp
    src/mppi_ipddp.cpp
    src/smooth_mppi.cpp
    src/mppi.cpp
    src/receding.cpp
    src/example.cpp
)

foreach(SRC ${COMMON_SOURCES})
    get_filename_component(EXE_NAME ${SRC} NAME_WE)
    add_executable(${EXE_NAME} ${SRC})
    target_compile_options(${EXE_NAME} PRIVATE -O3)
    target_link_libraries(${EXE_NAME} OpenMP::OpenMP_CXX)
endforeach()