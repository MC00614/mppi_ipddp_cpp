#pragma once

#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>
using namespace autodiff;

#include <eigen3/Eigen/Dense>

class ModelBase {
public:
    ModelBase();
    ~ModelBase();

    int N;
    int dim_x;
    int dim_u;
    int dim_c;
    Eigen::MatrixXd X;
    Eigen::MatrixXd U;
    Eigen::MatrixXd Y;
    Eigen::MatrixXd S;

    // Discrete Time System
    std::function<VectorXdual2nd(VectorXdual2nd, VectorXdual2nd)> f;
    std::vector<std::function<dual2nd(VectorXdual2nd, VectorXdual2nd)>> fs;
    // Stage Cost Function
    std::function<dual2nd(VectorXdual2nd, VectorXdual2nd)> q;
    // Terminal Cost Function
    std::function<dual2nd(VectorXdual2nd)> p;
    // Constraint
    std::function<VectorXdual2nd(VectorXdual2nd, VectorXdual2nd)> c;
};

ModelBase::ModelBase() {
    f = [this](const VectorXdual2nd& x, const VectorXdual2nd& u) -> VectorXdual2nd {
        VectorXdual2nd x_n(dim_x);
        for (int i = 0; i < dim_x; ++i) {
            x_n(i) = fs[i](x,u);
        }
        return x_n;
    };
};

ModelBase::~ModelBase() {
};


