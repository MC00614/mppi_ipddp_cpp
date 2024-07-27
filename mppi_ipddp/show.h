#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

void show2D(const Eigen::MatrixXd &mppi_X, const Eigen::MatrixXd &X, const Eigen::MatrixXd &U, const Eigen::MatrixXd &C, const Eigen::VectorXd &R,
            const std::vector<std::array<double,4>> &circles, const std::vector<std::array<double,4>> &rectangles) {
    std::vector<std::vector<double>> X_MPPI(X.rows(), std::vector<double>(X.cols()));
    for (int i = 0; i < X.rows(); ++i) {
        for (int j = 0; j < X.cols(); ++j) {
            X_MPPI[i][j] = mppi_X(i, j);
        }
    }
    std::vector<std::vector<double>> X_RES(X.rows(), std::vector<double>(X.cols()));
    for (int i = 0; i < X.rows(); ++i) {
        for (int j = 0; j < X.cols(); ++j) {
            X_RES[i][j] = X(i, j);
        }
    }

    for (int i = 0; i < circles.size(); ++i) {
        std::vector<double> cx;
        std::vector<double> cy;
        for (int t = 0; t < 361; ++t) {
            cx.push_back(circles[i][0] + circles[0][2]*cos(t/180.0*(M_PI)));
            cy.push_back(circles[i][1] + circles[0][2]*sin(t/180.0*M_PI));
        }
        plt::plot(cx, cy, "b");
    }

    for (int i = 0; i < rectangles.size(); ++i) {
        std::vector<double> cx = {rectangles[i][0], rectangles[i][1], rectangles[i][1], rectangles[i][0], rectangles[i][0]};
        std::vector<double> cy = {rectangles[i][2], rectangles[i][2], rectangles[i][3], rectangles[i][3], rectangles[i][2]};
        plt::plot(cx, cy, "b");
    }

    for (int i = 0; i < C.cols(); ++i) {
        std::vector<double> cx;
        std::vector<double> cy;
        for (int t = 0; t < 361; ++t) {
            cx.push_back(C.col(i)(0) + R(i)*cos(t/180.0*M_PI));
            cy.push_back(C.col(i)(1) + R(i)*sin(t/180.0*M_PI));
        }
        plt::plot(cx, cy, "r");
    }

    std::vector<std::vector<double>> C_RES(C.rows(), std::vector<double>(C.cols()));
    for (int i = 0; i < C.rows(); ++i) {
        for (int j = 0; j < C.cols(); ++j) {
            C_RES[i][j] = C(i, j);
        }
    }

    // plt::plot(C_RES[0], C_RES[1], "g");
    plt::plot(X_RES[0], X_RES[1], "k");
    plt::plot(X_MPPI[0], X_MPPI[1], "g");

    plt::xlim(-4, 4);
    plt::ylim(-1, 7);
    plt::grid(true);
    plt::show();
}