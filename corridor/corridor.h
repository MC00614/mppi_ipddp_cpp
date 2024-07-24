#include "corridor_param.h"
#include "model_base.h"
#include "collision_checker.h"

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
    std::vector<int> center_index;
    int center_index_size;

    Eigen::MatrixXd Z;
    Eigen::MatrixXd Zi;
    Eigen::VectorXd costs;
    Eigen::VectorXd weights;
};

Corridor::Corridor(ModelBase model) {
    this->N = N;
}

Corridor::~Corridor() {
}

void Corridor::init(CorridorParam corridor_param) {
    this->Nz = corridor_param.Nz;
    this->gamma_z = corridor_param.gamma_z;
    this->sigma_z = corridor_param.sigma_z;
    this->lambda_c = corridor_param.lambda_c;
    this->lambda_r = corridor_param.lambda_r;
    this->center_index = corridor_param.center_index;
    this->center_index_size = center_index.size();
    Z.resize(center_index_size + 1, N);
    Zi.resize((center_index_size + 1) * Nz, N);
    costs.resize(Nz);
    weights.resize(Nz);
}

void Corridor::setCollisionChecker(CollisionChecker *collision_checker) {
    this->collision_checker = collision_checker;
}

void Corridor::solve(const Eigen::MatrixXd &X, Eigen::MatrixXd &C, Eigen::VectorXd &R) {
    // not sufficiently inflated?
    double cost;
    
    for (int i = 0; i < center_index_size; ++i) {
        Z.row(i) = X.row(center_index[i]);
    }
    Z.row(center_index_size) = Eigen::VectorXd::Zero(N);

    for (int i = 0; i < Nz; ++i) {
        cost = 0.0;
        Zi.middleRows(i * (center_index_size + 1), center_index_size + 1) = Z + Eigen::MatrixXd::Random(center_index_size + 1, N) * this->sigma_z;
        // COST!!
        cost += lambda_c * (Zi.middleRows(i * (center_index_size + 1), center_index_size) - Z.topRows(center_index_size)).lpNorm<2>();
        cost -= lambda_r * (Zi.row(i * (center_index_size + 1) + center_index_size)).sum();
        cost += collision_checker->getCost(Zi.middleRows(i * (center_index_size + 1), center_index_size + 1));
        costs(i) = cost;
    }
    double min_cost = costs.minCoeff();
    weights = (-gamma_z * (costs.array() - min_cost)).exp();
    double total_weight = weights.sum();
    weights /= total_weight;

    Z = Eigen::MatrixXd::Zero(center_index_size + 1, N);
    for (int i = 0; i < Nz; ++i) {
        Z += Zi.middleRows(i * (center_index_size + 1), center_index_size + 1) * weights(i);
    }

    C = this->Z.topRows(center_index_size);
    R = this->Z.bottomRows(1);
}

Eigen::MatrixXd Corridor::getResC() {
    return this->Z.topRows(center_index_size);
};

Eigen::VectorXd Corridor::getResR() {
    return this->Z.bottomRows(1);
};