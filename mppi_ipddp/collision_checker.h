#pragma once

#include <eigen3/Eigen/Dense>

class CollisionChecker {
public:
    CollisionChecker();
    ~CollisionChecker();
    double getCostGrid(const Eigen::VectorXd &x);
    double getCostCircle(const Eigen::MatrixXd &z);
};

CollisionChecker::CollisionChecker() {
}

CollisionChecker::~CollisionChecker() {
}

double CollisionChecker::getCostGrid(const Eigen::VectorXd &x) {
    return 0.0;
}

double CollisionChecker::getCostCircle(const Eigen::MatrixXd &z) {
    return 0.0;
}