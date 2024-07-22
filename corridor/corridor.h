#include "model_base.h"

#include "collision_checker.h"

class Corridor {
public:
    Corridor(ModelBase model);
    ~Corridor();
    void init(int Nz, double lambda_z, double sigma_z);
    void setCollisionChecker(CollisionChecker *collision_checker);
    void setCenterIndex(std::vector<int> center_index);
    void solve(const Eigen::MatrixXd &X);

    Eigen::MatrixXd getResC();
    Eigen::VectorXd getResR();

private:
    int N;
    int Nz;
    double lambda_z;
    double sigma_z;
    CollisionChecker *collision_checker;
    std::vector<int> center_index;
    int center_index_size;

    Eigen::MatrixXd Z;
    Eigen::MatrixXd Zi;
};

Corridor::Corridor(ModelBase model) {
    this->N = N;
}

Corridor::~Corridor() {
}

void Corridor::init(int Nz, double lambda_z, double sigma_z) {
    this->Nz = Nz;
    this->lambda_z = lambda_z;
    this->sigma_z = sigma_z;
}

void Corridor::setCollisionChecker(CollisionChecker *collision_checker) {
    this->collision_checker = collision_checker;
}

void Corridor::setCenterIndex(std::vector<int> center_index) {
    this->center_index = center_index;
    this->center_index_size = center_index.size();
    Z.resize(center_index_size + 1, N);
    Zi.resize((center_index_size + 1) * Nz, N);
}

void Corridor::solve(const Eigen::MatrixXd &X) {
    // not sufficiently inflated?
    for (int i = 0; i < center_index_size; ++i) {
        Zi.middleRows(i) = X.row(center_index[i]);
    }
    Z.row(center_index_size) = Eigen::MatrixXd::Zero(1, N);

    for (int i = 0; i < 20; ++i) {
        Z = Z + Eigen::MatrixXd::Random(center_index_size + 1, N) * this->sigma_z;
    }
    
}

Eigen::MatrixXd Corridor::getResC() {
    return this->Z.topRows(center_index_size);
};

Eigen::VectorXd Corridor::getResR() {
    return this->Z.bottomRows(1);
};