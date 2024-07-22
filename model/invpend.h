#include "model_base.h"

class InvPend : public ModelBase {
public:
    InvPend();
    ~InvPend();
};

InvPend::InvPend() {
    // Stage Count
    N = 500;

    // Dimensions
    dim_x = 2;
    dim_u = 1;
    dim_c = 2;

    // Status Setting
    X = Eigen::MatrixXd::Zero(dim_x, N+1);
    X(0,0) = -M_PI;
    X(1,0) = 0.0;

    // U = 0.02*Eigen::MatrixXd::Random(dim_u, N) - Eigen::MatrixXd::Constant(dim_u, N, 0.01);
    U = Eigen::MatrixXd::Zero(dim_u, N);
    U(0,0) = 0.0;

    Y = 0.01*Eigen::MatrixXd::Ones(dim_c, N);

    S = 0.1*Eigen::MatrixXd::Ones(dim_c, N);
    
    // Discrete Time System
    auto f0 = [this](const VectorXdual2nd& x, const VectorXdual2nd& u) -> dual2nd {
        const double h = 0.05;
        return x(0) + h * x(1);
    };
    fs.push_back(f0);
    auto f1 = [this](const VectorXdual2nd& x, const VectorXdual2nd& u) -> dual2nd {
        const double h = 0.05;
        return x(1) + h * sin(x(0)) + h * u(0);
    };
    fs.push_back(f1);

    // Stage Cost Function
    q = [this](const VectorXdual2nd& x, const VectorXdual2nd& u) -> dual2nd {
        return 0.025 * (x.squaredNorm() + u.squaredNorm());
    };

    // Terminal Cost Function
    p = [this](const VectorXdual2nd& x) -> dual2nd {
        return 5.0 * x.squaredNorm();
    };

    // Constraint
    c = [this](const VectorXdual2nd& x, const VectorXdual2nd& u) -> VectorXdual2nd {
        VectorXdual2nd c_n(x.size());
        c_n(0) = u(0) - 0.25;
        c_n(1) = -u(0) - 0.25;
        return c_n;
    };
}

InvPend::~InvPend() {
}