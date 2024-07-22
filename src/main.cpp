#include "invpend.h"
#include "cartpole.h"

#include "mppi.h"
#include "corridor.h"
#include "ipddp.h"

int main() {
    // auto model = InvPend();
    auto model = CartPole();

    clock_t start = clock();

    // MPPI
    MPPI mppi(model);
    int Nu = 100;
    double lambda = 1.0;
    double sigma_u = 1.0;
    mppi.init(Nu, lambda, sigma_u);
    CollisionChecker *collision_checker;
    mppi.setCollisionChecker(collision_checker);

    // Corridor
    Corridor corridor(model);
    CorridorParam corridor_param;
    corridor_param.Nz = 3000;
    corridor_param.gamma_z = 1000.0;
    corridor_param.sigma_z = 0.3;
    corridor_param.lambda_c = 0.3;
    corridor_param.lambda_r = 0.3;
    corridor_param.center_index = {0,1};
    corridor.init(corridor_param);

    // IPDDP
    Param param;
    param.tolerance = 1e-7;
    param.max_iter = 100;
    param.mu = 0;
    IPDDP ipddp(model);
    ipddp.init(param);

    Eigen::MatrixXd X;
    Eigen::MatrixXd U;
    Eigen::MatrixXd C;
    Eigen::VectorXd R;

    while (true) {
        mppi.solve();
        X = mppi.getResX();
        U = mppi.getResU();
        corridor.solve(X);
        C = corridor.getResC();
        R = corridor.getResR();
        ipddp.solve();
        // ipddp.solve(X,U,C,R);
    }

    clock_t finish = clock();
    double duration = (double)(finish - start) / CLOCKS_PER_SEC;
    std::cout << "\nIn Total : " << duration << " Seconds" << std::endl;

    return 0;
}