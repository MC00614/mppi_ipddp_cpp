#include "model_base.h"

class CartPole : public ModelBase {
public:
    CartPole();
    ~CartPole();
};

CartPole::CartPole() {
    // Stage Count
    N = 200;

    // Dimensions
    dim_x = 4;
    dim_u = 1;
    dim_c = 2;

    // Status Setting
    X = Eigen::MatrixXd::Zero(dim_x, N+1);
    X(0,0) = 0.0;
    X(1,0) = 0.1;
    X(2,0) = 0.0;
    X(3,0) = 0.0;

    // U = 0.02*Eigen::MatrixXd::Random(dim_u, N) - Eigen::MatrixXd::Constant(dim_u, N, 0.01);
    U = Eigen::MatrixXd::Zero(dim_u, N);
    U(0,0) = 0.0;

    Y = 0.01*Eigen::MatrixXd::Ones(dim_c, N);

    S = 0.1*Eigen::MatrixXd::Ones(dim_c, N);
    
    // Discrete Time System
    auto f0 = [this](const VectorXdual2nd& x, const VectorXdual2nd& u) -> dual2nd {
        const double dt = 0.01f;
        return x(0) + x(2) * dt;
    };
    fs.push_back(f0);
    auto f1 = [this](const VectorXdual2nd& x, const VectorXdual2nd& u) -> dual2nd {
        const double dt = 0.01f;
        return x(1) + x(3) * dt;
    };
    fs.push_back(f1);
    auto f2 = [this](const VectorXdual2nd& x, const VectorXdual2nd& u) -> dual2nd {
        const double dt = 0.01;
        dual2nd a = (1.0 / (1.0 + 0.1 - 0.1 * cos(x(1)) * cos(x(1)))) * (u(0) + 0.1 * sin(x(1)) * (1.0 * x(3) * x(3) + 9.81 * cos(x(1))));
        return x(2) + a * dt;
    };
    fs.push_back(f2);
    auto f3 = [this](const VectorXdual2nd& x, const VectorXdual2nd& u) -> dual2nd {
        const double dt = 0.01;
        dual2nd theta_ddot = (1.0 / (1.0 * (1.0 + 0.1 - 0.1 * cos(x(1)) * sin(x(1))))) * (-u(0) * cos(x(1)) - 0.1 * 1.0 * x(3) * x(3) * sin(x(1)) * cos(x(1)) - (1.0 + 0.1) * 9.81 * sin(x(1)));
        return x(3) + theta_ddot * dt;
    };
    fs.push_back(f3);

    // Stage Cost Function
    q = [this](const VectorXdual2nd& x, const VectorXdual2nd& u) -> dual2nd {
        return 1.0*x(0)*x(0) + 10.0*x(1)*x(1) + 1.0*x(2)*x(2) + 10.0*x(3)*x(3) + 0.1 *u.squaredNorm();
    };

    // Terminal Cost Function
    p = [this](const VectorXdual2nd& x) -> dual2nd {
        return 5.0 * x.squaredNorm();
    };

    // Constraint
    c = [this](const VectorXdual2nd& x, const VectorXdual2nd& u) -> VectorXdual2nd {
        VectorXdual2nd c_n(2);
        c_n(0) = u(0) - 0.25;
        c_n(1) = -u(0) - 0.25;
        return c_n;
    };
}

CartPole::~CartPole() {
}