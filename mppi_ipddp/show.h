#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

inline void show2D(const Eigen::MatrixXd &mppi_X, const Eigen::MatrixXd &mppi_U, const std::string& file_path, double resolution) {
    std::vector<std::vector<double>> X_MPPI(mppi_X.rows(), std::vector<double>(mppi_X.cols()));
    for (int i = 0; i < mppi_X.rows(); ++i) {
        for (int j = 0; j < mppi_X.cols(); ++j) {
            X_MPPI[i][j] = mppi_X(i, j);
        }
    }
    std::vector<std::vector<double>> U_MPPI(mppi_U.rows(), std::vector<double>(mppi_U.cols()));
    for (int i = 0; i < mppi_U.rows(); ++i) {
        for (int j = 0; j < mppi_U.cols(); ++j) {
            U_MPPI[i][j] = mppi_U(i, j);
        }
    }

    std::vector<std::vector<double>> map;
    std::ifstream file(file_path);
    std::string line;

    while (std::getline(file, line)) {
        std::vector<double> row;
        std::string::size_type sz = 0;

        if (!file.is_open()) {
            throw std::runtime_error("Error opening file: " + file_path);
        }

        while (sz < line.length()) {
            double value = std::stod(line, &sz);
            row.push_back(value);
            line = line.substr(sz);
        }

        map.push_back(row);
    }
    file.close();

    plt::scatter(X_MPPI[0], X_MPPI[1], 10.0, {{"color", "b"}});
    plt::plot(X_MPPI[0], X_MPPI[1], "b");

    double hl = 0.05;
    for (int i = 0; i < map.size(); ++i) {
        for (int j = 0; j < map[0].size(); ++j) {
            if (map[i][j] == 10) {
                double mx = i*resolution;
                double my = j*resolution;
                std::vector<double> oX = {mx-hl, mx+hl, mx+hl, mx-hl, mx-hl};
                std::vector<double> oY = {my-hl,my-hl,my+hl,my+hl,my-hl};
                plt::plot(oX, oY, "k");
            }
        }
    }
    
    plt::xlim(0, 3);
    plt::ylim(0, 5);
    plt::grid(true);
    plt::show();
}

inline void show2D(const Eigen::MatrixXd &mppi_X, const Eigen::MatrixXd &mppi_U, const Eigen::MatrixXd &X, const Eigen::MatrixXd &U, const Eigen::MatrixXd &C, const Eigen::VectorXd &R,
            const std::vector<std::array<double,4>> &circles, const std::vector<std::array<double,4>> &rectangles) {

    plt::subplot(2,2,1);
    std::vector<std::vector<double>> X_MPPI(X.rows(), std::vector<double>(X.cols()));
    for (int i = 0; i < X.rows(); ++i) {
        for (int j = 0; j < X.cols(); ++j) {
            X_MPPI[i][j] = mppi_X(i, j);
        }
    }
    std::vector<std::vector<double>> U_MPPI(U.rows(), std::vector<double>(U.cols()));
    for (int i = 0; i < U.rows(); ++i) {
        for (int j = 0; j < U.cols(); ++j) {
            U_MPPI[i][j] = mppi_U(i, j);
        }
    }
    std::vector<std::vector<double>> X_RES(X.rows(), std::vector<double>(X.cols()));
    for (int i = 0; i < X.rows(); ++i) {
        for (int j = 0; j < X.cols(); ++j) {
            X_RES[i][j] = X(i, j);
        }
    }
    std::vector<std::vector<double>> U_RES(U.rows(), std::vector<double>(U.cols()));
    for (int i = 0; i < U.rows(); ++i) {
        for (int j = 0; j < U.cols(); ++j) {
            U_RES[i][j] = U(i, j);
        }
    }

    for (int i = 0; i < circles.size(); ++i) {
        std::vector<double> cx;
        std::vector<double> cy;
        for (int t = 0; t < 361; ++t) {
            cx.push_back(circles[i][0] + circles[0][2]*cos(t/180.0*(M_PI)));
            cy.push_back(circles[i][1] + circles[0][2]*sin(t/180.0*M_PI));
        }
        plt::plot(cx, cy, "k");
    }

    for (int i = 0; i < rectangles.size(); ++i) {
        std::vector<double> cx = {rectangles[i][0], rectangles[i][1], rectangles[i][1], rectangles[i][0], rectangles[i][0]};
        std::vector<double> cy = {rectangles[i][2], rectangles[i][2], rectangles[i][3], rectangles[i][3], rectangles[i][2]};
        plt::plot(cx, cy, "k");
    }

    for (int i = 0; i < C.cols(); ++i) {
        std::vector<double> cx;
        std::vector<double> cy;
        for (int t = 0; t < 361; ++t) {
            cx.push_back(C.col(i)(0) + R(i)*cos(t/180.0*M_PI));
            cy.push_back(C.col(i)(1) + R(i)*sin(t/180.0*M_PI));
        }
        plt::plot(cx, cy, "silver");
    }

    std::vector<std::vector<double>> C_RES(C.rows(), std::vector<double>(C.cols()));
    for (int i = 0; i < C.rows(); ++i) {
        for (int j = 0; j < C.cols(); ++j) {
            C_RES[i][j] = C(i, j);
        }
    }

    // plt::plot(C_RES[0], C_RES[1], "g");
    plt::named_plot("MPPI", X_MPPI[0], X_MPPI[1], "b");
    plt::named_plot("MPPI-IPDDP", X_RES[0], X_RES[1], "r");
    plt::scatter(X_MPPI[0], X_MPPI[1], 10.0, {{"color", "b"}});
    plt::scatter(X_RES[0], X_RES[1], 10.0, {{"color", "r"}});
    plt::xlim(0, 3);
    plt::ylim(0, 5);
    // plt::xlim(-4, 4);
    // plt::ylim(-1, 7);
    plt::grid(true);
    plt::legend({{"fontsize", "20"}});


    plt::subplot(2,2,3);

    for (int i = 0; i < circles.size(); ++i) {
        std::vector<double> cx;
        std::vector<double> cy;
        for (int t = 0; t < 361; ++t) {
            cx.push_back(circles[i][0] + circles[0][2]*cos(t/180.0*(M_PI)));
            cy.push_back(circles[i][1] + circles[0][2]*sin(t/180.0*M_PI));
        }
        plt::plot(cx, cy, "k");
    }

    for (int i = 0; i < rectangles.size(); ++i) {
        std::vector<double> cx = {rectangles[i][0], rectangles[i][1], rectangles[i][1], rectangles[i][0], rectangles[i][0]};
        std::vector<double> cy = {rectangles[i][2], rectangles[i][2], rectangles[i][3], rectangles[i][3], rectangles[i][2]};
        plt::plot(cx, cy, "k");
    }

    // plt::named_plot("MPPI", X_MPPI[0], X_MPPI[1], "r");
    // plt::named_plot("MPPI-IPDDP", X_RES[0], X_RES[1], "b");
    plt::scatter(X_MPPI[0], X_MPPI[1], 10.0, {{"color", "b"}});
    plt::scatter(X_RES[0], X_RES[1], 10.0, {{"color", "r"}});
    plt::xlim(0, 3);
    plt::ylim(0, 5);
    // plt::xlim(-4, 4);
    // plt::ylim(-1, 7);
    plt::grid(true);
    // plt::legend({{"fontsize", "20"}});

    plt::subplot(2,2,2);
    plt::named_plot("MPPI", U_MPPI[0]);
    plt::named_plot("MPPI-IPDDP", U_RES[0]);
    plt::ylim(-0.1, 1.6);
    plt::grid(true);
    plt::legend({{"fontsize", "20"}});
    // plt::title("U Dimension " + std::to_string(0), {{"fontsize", "20"}});
    plt::title("U Dimension 0 (v)", {{"fontsize", "20"}});

    plt::subplot(2,2,4);
    plt::named_plot("MPPI", U_MPPI[1]);
    plt::named_plot("MPPI-IPDDP", U_RES[1]);
    plt::ylim(-1.6, 1.6);
    // plt::title("U Dimension " + std::to_string(1), {{"fontsize", "20"}});
    plt::title("U Dimension 1 (w)", {{"fontsize", "20"}});
    plt::grid(true);
    plt::legend({{"fontsize", "20"}});
    plt::show();
}