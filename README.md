# MPPI-IPDDP C++

Implementation of the [MPPI-IPDDP](https://arxiv.org/abs/2208.02439) in C++.

# Key Features
- Header only solver
- CPU multi-processing
- Benchmark with MPPI variants
- Collision Checker can be modified on demand
- Custom model can be easily configured

# Usage
### Download Dependencies
- [autodiff](https://github.com/autodiff/autodiff)
- [Eigen 3.3.9](https://gitlab.com/libeigen/eigen/-/releases/3.3.9)
- [EigenRand](https://github.com/bab2min/EigenRand)
- [BARN_dataset](https://www.cs.utexas.edu/~xiao/BARN/BARN.html)
### Build with CMake
```
mkdir build && cd build
cmake.. && make
```
For simple usage, refer [example.cpp](src/example.cpp) with 2D Wheeled Mobile Robot.


### Performance comparison with MPPI variants
| MPPI | Log-MPPI |
|----------|----------|
| ![MPPI](doc/3Dplot/mppi.png) | ![Log-MPPI](doc/3Dplot/log-mppi.png) | !

| Smooth-MPPI | MPPI-IPDDP |
|----------|----------|
| ![Smooth-MPPI](doc/3Dplot/smooth-mppi.png) | ![MPPI-IPDDP](doc/3Dplot/mppi-ipddp.png) | !

To reproduce this, refer [main.cpp](src/main.cpp) with `specific Target` and `Number of Simulations`
```cpp
// Target: MPPI-IPDDP, Simulations: 10
./main MPPI-IPDDP 10
```


# Future Works
- GPU Acceleration
- More general constraint handling