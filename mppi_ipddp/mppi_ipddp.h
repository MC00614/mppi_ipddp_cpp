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
    void solve();

private:
    MPPI mppi;
    Corridor corridor;
    IPDDP ipddp;

    int N;
    int dim_x;
    int dim_u;
    Eigen::MatrixXd X;
    Eigen::MatrixXd U;
    Eigen::MatrixXd C;
    Eigen::VectorXd R;
};

MPPI_IPDDP::MPPI_IPDDP(ModelBase model)
    : mppi(model), ipddp(model), corridor(model) {
    N = model.N;
    dim_x = model.dim_x;
    dim_u = model.dim_u;

    X = model.X;
    U = model.U;
}

MPPI_IPDDP::~MPPI_IPDDP() {
}

void MPPI_IPDDP::init(MPPIParam mppi_param, CorridorParam corridor_param, Param ipddp_param) {
    mppi.init(mppi_param);
    corridor.init(corridor_param);
    ipddp.init(ipddp_param);

    C.resize(corridor_param.center_index.size(), N);
    R.resize(N);
}

void MPPI_IPDDP::setCollisionChecker(CollisionChecker *collision_checker) {
    mppi.setCollisionChecker(collision_checker);
    corridor.setCollisionChecker(collision_checker);
}

void MPPI_IPDDP::solve() {
    while (true) {
        mppi.solve(X, U);
        corridor.solve(X, C, R);
        ipddp.solve(X, U, C, R);
    }
}
