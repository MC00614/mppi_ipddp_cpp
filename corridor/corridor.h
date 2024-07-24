#include "corridor_param.h"
#include "model_base.h"
#include "collision_checker.h"

#include <iostream>

class Corridor {
public:
    Corridor(ModelBase model);
    ~Corridor();
    void init(CorridorParam corridor_param);
    void setCollisionChecker(CollisionChecker *collision_checker);
    void solve(const Eigen::MatrixXd &X, Eigen::MatrixXd &C, Eigen::VectorXd &R);

    Eigen::MatrixXd getResC();
    Eigen::VectorXd getResR();

private:
    int N;
    int Nz;
    double gamma_z;
    double sigma_z;
    double lambda_c;
    double lambda_r;
    CollisionChecker *collision_checker;
    int center_point;

    Eigen::MatrixXd Z;
    Eigen::MatrixXd Zi;
    Eigen::VectorXd costs;
    Eigen::VectorXd weights;
};

Corridor::Corridor(ModelBase model) {
    this->N = model.N;
    this->center_point = model.center_point;
}

Corridor::~Corridor() {
}

void Corridor::init(CorridorParam corridor_param) {
    this->Nz = corridor_param.Nz;
    this->gamma_z = corridor_param.gamma_z;
    this->sigma_z = corridor_param.sigma_z;
    this->lambda_c = corridor_param.lambda_c;
    this->lambda_r = corridor_param.lambda_r;
    Z.resize(center_point + 1, N + 1);
    Zi.resize((center_point + 1) * Nz, N + 1);
    costs.resize(Nz);
    weights.resize(Nz);
}

void Corridor::setCollisionChecker(CollisionChecker *collision_checker) {
    this->collision_checker = collision_checker;
}

void Corridor::solve(const Eigen::MatrixXd &X, Eigen::MatrixXd &C, Eigen::VectorXd &R) {
    // not sufficiently inflated?
    double cost;

    Z.topRows(center_point) = X.topRows(center_point);
    Z.bottomRows(1) = Eigen::MatrixXd::Zero(1, N + 1);

    for (int i = 0; i < Nz; ++i) {
        cost = 0.0;
        Zi.middleRows(i * (center_point + 1), center_point + 1) = Z + Eigen::MatrixXd::Random(center_point + 1, N + 1) * this->sigma_z;

        cost += lambda_c * (Zi.middleRows(i * (center_point + 1), center_point) - Z.topRows(center_point)).lpNorm<2>();
        cost -= lambda_r * (Zi.row(i * (center_point + 1) + center_point)).sum();
        cost += collision_checker->getCostCircle(Zi.middleRows(i * (center_point + 1), center_point + 1));
        costs(i) = cost;
    }
    double min_cost = costs.minCoeff();
    weights = (-gamma_z * (costs.array() - min_cost)).exp();
    double total_weight = weights.sum();
    weights /= total_weight;
    Z = Eigen::MatrixXd::Zero(center_point + 1, N + 1);
    for (int i = 0; i < Nz; ++i) {
        Z += Zi.middleRows(i * (center_point + 1), center_point + 1) * weights(i);
    }
    C = Z.topRows(center_point);
    R = Z.bottomRows(1).transpose();
}

Eigen::MatrixXd Corridor::getResC() {
    return this->Z.topRows(center_point);
};

Eigen::VectorXd Corridor::getResR() {
    return this->Z.bottomRows(1);
};