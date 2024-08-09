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

# MPPI COMPARE
add_executable(main
    src/main.cpp
)
target_compile_options(main PRIVATE -O3)
target_link_libraries(main
    OpenMP::OpenMP_CXX
)

# BARN TEST
add_executable(test_with_map
    src/test_with_map.cpp
)
target_compile_options(test_with_map PRIVATE -O3)
target_link_libraries(test_with_map
    OpenMP::OpenMP_CXX
)

# Log_MPPI
add_executable(log_mppi
    src/log_mppi.cpp
)
target_compile_options(log_mppi PRIVATE -O3)
target_link_libraries(log_mppi
    OpenMP::OpenMP_CXX
)

# MPPI_IPDDP
add_executable(mppi_ipddp
    src/mppi_ipddp.cpp
)
target_compile_options(mppi_ipddp PRIVATE -O3)
target_link_libraries(mppi_ipddp
    OpenMP::OpenMP_CXX
)

# Smooth MPPI
add_executable(smooth_mppi
    src/smooth_mppi.cpp
)
target_compile_options(smooth_mppi PRIVATE -O3)
target_link_libraries(smooth_mppi
    OpenMP::OpenMP_CXX
)

# MPPI
add_executable(mppi
    src/mppi.cpp
)
target_compile_options(mppi PRIVATE -O3)
target_link_libraries(mppi
    OpenMP::OpenMP_CXX
)

# Receding
add_executable(receding
    src/receding.cpp
)
target_compile_options(receding PRIVATE -O3)
target_link_libraries(receding
    OpenMP::OpenMP_CXX
)

# Example
add_executable(example
    src/example.cpp
)
target_compile_options(example PRIVATE -O3)
target_link_libraries(example
    OpenMP::OpenMP_CXX
)