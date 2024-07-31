#include "corridor_param.h"
#include "model_base.h"
#include "collision_checker.h"

#include <EigenRand/EigenRand>

#include <ctime>
#include <iostream>

#include <omp.h>

class Corridor {
public:
    Corridor(ModelBase model);
    ~Corridor();
    void init(CorridorParam corridor_param);
    void setCollisionChecker(CollisionChecker *collision_checker);
    void solve(const Eigen::MatrixXd &X, Eigen::MatrixXd &C, Eigen::VectorXd &R);
    void hz(Eigen::MatrixXd &Z);

    Eigen::MatrixXd getResC();
    Eigen::VectorXd getResR();

private:
    int N;
    int max_iter;
    int Nz;
    double gamma_z;
    Eigen::MatrixXd sigma_z;
    double lambda_c;
    double lambda_r;
    double r_max;
    CollisionChecker *collision_checker;
    int center_point;

    std::mt19937_64 urng{static_cast<std::uint_fast64_t>(std::time(nullptr))};
    // std::mt19937_64 urng{1};
    Eigen::Rand::NormalGen<double> norm_gen{0.0, 1.0};

    Eigen::MatrixXd Z;
};

Corridor::Corridor(ModelBase model) {
    this->N = model.N;
    this->center_point = model.center_point;
}

Corridor::~Corridor() {
}

void Corridor::init(CorridorParam corridor_param) {
    this->max_iter = corridor_param.max_iter;
    this->Nz = corridor_param.Nz;
    this->gamma_z = corridor_param.gamma_z;
    this->sigma_z = corridor_param.sigma_z;
    this->lambda_c = corridor_param.lambda_c;
    this->lambda_r = corridor_param.lambda_r;
    this->r_max = corridor_param.r_max;
    Z.resize(center_point + 1, N);
}

void Corridor::setCollisionChecker(CollisionChecker *collision_checker) {
    this->collision_checker = collision_checker;
}

void Corridor::solve(const Eigen::MatrixXd &X, Eigen::MatrixXd &C, Eigen::VectorXd &R) {
    Z.topRows(center_point) = X.topRows(center_point).leftCols(N);
    Z.bottomRows(1) = Eigen::MatrixXd::Zero(1, N);

    for (int iter = 0; iter < max_iter; ++iter) {
        #pragma omp parallel for
        for (int t = 0; t < N; ++t) {
            Eigen::MatrixXd Zi(center_point + 1, Nz);
            Zi = Z.col(t).replicate(1, Nz) + (this->sigma_z * norm_gen.template generate<Eigen::MatrixXd>(center_point + 1, Nz, urng));
            Eigen::VectorXd costs(Nz);
            costs = lambda_c * (Zi.topRows(center_point).colwise() - X.col(t).topRows(center_point)).colwise().norm();
            costs -= lambda_r * Zi.bottomRows(1).transpose();
            for (int i = 0; i < Nz; ++i) {
                if (collision_checker->getCollisionCircle(Zi.col(i))) {
                    costs(i) = 1e8;
                }
            }
            double min_cost = costs.minCoeff();
            
            Eigen::VectorXd weights;
            weights = (-gamma_z * (costs.array() - min_cost)).exp();
            double total_weight = weights.sum();
            weights /= total_weight;
            Z.col(t) = Zi * weights;
        }
        hz(Z);
    }
    C = Z.topRows(center_point);
    R = Z.bottomRows(1).transpose();
}

void Corridor::hz(Eigen::MatrixXd &Z) {
    double radius;
    Eigen::MatrixXd distance_vector;
    double distance;
    for (int j = 0; j < N; ++j) {
        radius = Z(center_point, j);
        if (radius < -r_max) {radius = -r_max;}
        else if (radius < 0.0) {radius = -radius;}
        else if (r_max < radius) {radius = r_max;}
        Z(center_point, j) = radius;

        // distance_vector = Z.topRows(center_point).col(j) - X.topRows(center_point).col(j);
        // distance = distance_vector.norm();
        // if (radius < distance) {
        //     Z.topRows(center_point).col(j) = X.topRows(center_point).col(j) + radius/distance * distance_vector;
        // }
    }
};

Eigen::MatrixXd Corridor::getResC() {
    return this->Z.topRows(center_point);
};

Eigen::VectorXd Corridor::getResR() {
    return this->Z.bottomRows(1);
};