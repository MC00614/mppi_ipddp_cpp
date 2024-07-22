#include "model_base.h"

class CarParking : public ModelBase {
public:
    CarParking();
    ~CarParking();
};

CarParking::CarParking() {
    // Stage Count
    N = 500;

    // Dimensions
    dim_x = 4;
    dim_u = 2;
    dim_c = 4;

    // Status Setting
    X = Eigen::MatrixXd::Zero(dim_x, N+1);
    X(0,0) = 1.0;
    X(1,0) = 1.0;
    X(2,0) = 3*M_PI_2;
    X(3,0) = 0.0;

    U = 0.02 * Eigen::MatrixXd::Random(dim_u, N) - 0.01 * Eigen::MatrixXd::Ones(dim_u, N);
    U(0,0) = 0.0;
    U(1,0) = 0.0;

    Y = 0.01*Eigen::MatrixXd::Ones(dim_c, N);

    S = 0.1*Eigen::MatrixXd::Ones(dim_c, N);

    // Discrete Time System
    auto f0 = [this](const VectorXdual2nd& x, const VectorXdual2nd& u) -> dual2nd {
        const double h = 0.03;
        const double d = 2.0;
        dual2nd b = d + h*x(3)*cos(u(0)) - sqrt(d*d - h*h*x(3)*x(3)*sin(u(0))*sin(u(0)));
        return x(0) + b*cos(x(2));
    };
    fs.push_back(f0);
    auto f1 = [this](const VectorXdual2nd& x, const VectorXdual2nd& u) -> dual2nd {
        const double h = 0.03;
        const double d = 2.0;
        dual2nd b = d + h*x(3)*cos(u(0)) - sqrt(d*d - h*h*x(3)*x(3)*sin(u(0))*sin(u(0)));
        return x(1) + b*sin(x(2));
    };
    fs.push_back(f1);
    auto f2 = [this](const VectorXdual2nd& x, const VectorXdual2nd& u) -> dual2nd {
        const double h = 0.03;
        const double d = 2.0;
        return x(2) + asin((h*x(3)/d)*sin(u(0)));
    };
    fs.push_back(f2);
    auto f3 = [this](const VectorXdual2nd& x, const VectorXdual2nd& u) -> dual2nd {
        const double h = 0.03;
        const double d = 2.0;
        return x(3) + h*u(1);
    };
    fs.push_back(f3);

    // Stage Cost Function
    q = [this](const VectorXdual2nd& x, const VectorXdual2nd& u) -> dual2nd {
        return 0.01 * (u(0)*u(0) + 0.01*u(1)*u(1)) +
            0.01 * (sqrt(x(0) * x(0) + 0.1 * 0.1) - 0.1) +
            0.01 * (sqrt(x(1) * x(1) + 0.1 * 0.1) - 0.1);
    };

    // Terminal Cost Function
    p = [this](const VectorXdual2nd& x) -> dual2nd {
        return sqrt(x(0) * x(0) + 0.1 * 0.1) - 0.1 +
           sqrt(x(1) * x(1) + 0.1 * 0.1) - 0.1 +
           sqrt(x(2) * x(2) + 0.01 * 0.01) - 0.01 +
           sqrt(x(3) * x(3) + 1.0 * 1.0) - 1.0;
    };

    // Constraint
    c = [this](const VectorXdual2nd& x, const VectorXdual2nd& u) -> VectorXdual2nd {
        VectorXdual2nd c_n(x.size());
        c_n(0) = u(0) - 0.5;
        c_n(1) = -u(0) - 0.5;
        c_n(2) = u(1) - 2.0;
        c_n(3) = -u(1) - 2.0;
        return c_n;
    };
}

CarParking::~CarParking() {
}