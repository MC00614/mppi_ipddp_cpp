cmake_minimum_required(VERSION 3.5)
project(mppi_ipddp)

find_package(PythonLibs REQUIRED)
find_package (Eigen3 REQUIRED NO_MODULE)

# autodiff requires a c++17 enabled compiler
set(CMAKE_CXX_STANDARD 17)          # ensure cmake instructs compiler to use C++17
set(CMAKE_CXX_STANDARD_REQUIRED ON) # ensure the C++ standard given before is actually used
set(CMAKE_CXX_EXTENSIONS OFF)       # avoid compile flags of the type -std=gnu++1z added by cmake

include_directories(
    EigenRand/

    mppi/mppi/
    corridor/
    ipddp/ipddp/

    model/
    mppi_ipddp/
    autodiff/
    ${EIGEN3_INCLUDE_DIRS}
    matplotlibcpp/
    ${PYTHON_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
    src/main.cpp
)

# add_compile_options(-march=native)
target_compile_options(${PROJECT_NAME} PRIVATE -O3)

target_link_libraries(${PROJECT_NAME}
    ${PYTHON_LIBRARIES}
)