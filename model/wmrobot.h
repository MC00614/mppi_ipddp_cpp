#include "model_base.h"

class WMRobot : public ModelBase {
public:
    WMRobot();
    ~WMRobot();

    const int consts = 4;
    const double dt = 0.1;
};

WMRobot::WMRobot() {
    // Stage Count
    N = 30;

    // Dimensions
    dim_x = 3;
    dim_u = 2;
    dim_c = consts + 2*N;

    center_point = 2;

    // Status Setting
    X = Eigen::MatrixXd::Zero(dim_x, N+1);
    X(0,0) = 0.0;
    X(1,0) = 0.0;
    X(2,0) = M_PI_2;

    // U = 0.02*Eigen::MatrixXd::Random(dim_u, N) - Eigen::MatrixXd::Constant(dim_u, N, 0.01);
    U = Eigen::MatrixXd::Zero(dim_u, N);
    U(0,0) = 0.0;

    Y = 0.01*Eigen::MatrixXd::Ones(dim_c, N);

    S = 0.1*Eigen::MatrixXd::Ones(dim_c, N);
    
    // Discrete Time System
    auto f0 = [this](const VectorXdual2nd& x, const VectorXdual2nd& u) -> dual2nd {
        return x(0) + (u(0) * cos(x(2)) * dt);
    };
    fs.push_back(f0);
    auto f1 = [this](const VectorXdual2nd& x, const VectorXdual2nd& u) -> dual2nd {
        return x(1) + (u(0) * sin(x(2)) * dt);
    };
    fs.push_back(f1);
    auto f2 = [this](const VectorXdual2nd& x, const VectorXdual2nd& u) -> dual2nd {
        return x(2) + (u(1) * dt);
    };
    fs.push_back(f2);

    // Stage Cost Function
    q = [this](const VectorXdual2nd& x, const VectorXdual2nd& u) -> dual2nd {
        return 0.01 * (u.squaredNorm());
    };

    // Terminal Cost Function
    p = [this](const VectorXdual2nd& x) -> dual2nd {
        return 300.0 * (x(0)*x(0) + (x(1)-6)*(x(1)-6) + (x(2)-M_PI_2)*(x(2)-M_PI_2));
    };

    // Constraint
    c = [this](const VectorXdual2nd& x, const VectorXdual2nd& u, const Eigen::MatrixXd &C, const Eigen::VectorXd &R) -> VectorXdual2nd {    
        VectorXdual2nd c_n(dim_c);
        c_n(0) = u(0) - 1.5;
        c_n(1) = - u(0);
        c_n(2) = u(1) - 1.5;
        c_n(3) = - u(1) - 1.5;
        for (int i = 0; i < N; ++i) {
            dual2nd distance = (x.topRows(center_point) - C.col(i)).lpNorm<2>();
            c_n(consts + (2*i)) = distance - R(i);
            c_n(consts + (2*i) + 1) = - distance - R(i);
        }
        return c_n;
    };
}

WMRobot::~WMRobot() {
}