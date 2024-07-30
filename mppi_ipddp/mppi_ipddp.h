#include "mppi.h"
#include "corridor.h"
#include "ipddp.h"

#include "model_base.h"

class MPPI_IPDDP {
public:
    MPPI_IPDDP(ModelBase model);
    ~MPPI_IPDDP();

    void init(MPPIParam mppi_param, CorridorParam corridor_param, Param ipddp_param);
    void setCollisionChecker(CollisionChecker *collision_checker);
    void solve(int iter);
    void move();

    Eigen::MatrixXd X;
    Eigen::MatrixXd U;
    Eigen::MatrixXd C;
    Eigen::VectorXd R;

    // TEMP
    Eigen::MatrixXd mppi_X;
    Eigen::MatrixXd mppi_U;
    double mppi_duration;
    double corridor_duration;
    double ipddp_duration;

private:
    MPPI mppi;
    Corridor corridor;
    IPDDP ipddp;

    std::function<VectorXdual2nd(VectorXdual2nd, VectorXdual2nd)> f;

    int N;
    int dim_x;
    int dim_u;
    int center_point;
};

MPPI_IPDDP::MPPI_IPDDP(ModelBase model)
    : mppi(model), ipddp(model), corridor(model) {
    N = model.N;
    dim_x = model.dim_x;
    dim_u = model.dim_u;
    center_point = model.center_point;

    f = model.f;

    X = model.X;
    U = model.U;
}

MPPI_IPDDP::~MPPI_IPDDP() {
}

void MPPI_IPDDP::init(MPPIParam mppi_param, CorridorParam corridor_param, Param ipddp_param) {
    mppi.init(mppi_param);
    corridor.init(corridor_param);
    ipddp.init(ipddp_param);

    C.resize(center_point, N);
    R.resize(N);
}

void MPPI_IPDDP::setCollisionChecker(CollisionChecker *collision_checker) {
    mppi.setCollisionChecker(collision_checker);
    corridor.setCollisionChecker(collision_checker);
}

void MPPI_IPDDP::solve(int iter) {
    clock_t start;
    clock_t finish;
    mppi_duration = 0.0;
    corridor_duration = 0.0;
    ipddp_duration = 0.0;

    for (int i = 0; i < iter; ++i) {
        start = clock();
        // std::cout<<"mppi"<<std::endl;
        mppi.solve(X, U);
        finish = clock();
        mppi_duration += (double)(finish - start) / CLOCKS_PER_SEC;

        // TEMP //
        mppi_X = X;
        mppi_U = U;
        // TEMP //

        start = clock();
        // std::cout<<"corridor"<<std::endl;
        corridor.solve(X, C, R);
        finish = clock();
        corridor_duration += (double)(finish - start) / CLOCKS_PER_SEC;

        start = clock();
        // std::cout<<"ipddp"<<std::endl;
        ipddp.solve(X, U, C, R);
        finish = clock();
        ipddp_duration += (double)(finish - start) / CLOCKS_PER_SEC;
    }
    // std::cout << "MPPI : " << mppi_duration << " Seconds" << std::endl;
    // std::cout << "CORRIDOR : " << corridor_duration << " Seconds" << std::endl;
    // std::cout << "IPDDP : " << ipddp_duration << " Seconds" << std::endl;
    // std::cout << "Total : " << mppi_duration+corridor_duration+ipddp_duration << " Seconds" << std::endl;
    // std::cout << "" << std::endl;
}

void MPPI_IPDDP::move() {
    X.leftCols(N-1) = X.rightCols(N-1);
    U.leftCols(N-1) = U.rightCols(N-1);
    X.col(N) = f(X.col(N-1), U.col(N-1)).cast<double>();
}