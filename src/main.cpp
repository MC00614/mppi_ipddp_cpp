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
    Corridor corridor()
    corridor.setCollisionChecker(collision_checker);;

    // IPDDP
    Param param;
    param.tolerance = 1e-7;
    param.max_iter = 100;
    param.mu = 0;
    IPDDP ipddp(model);
    ipddp.init(param);

    Eigen::MatrixXd X;
    Eigen::MatrixXd U;

    while (true) {
        mppi.solve();
        Eigen::MatrixXd X = mppi.getResX();
        Eigen::MatrixXd U = mppi.getResU();
        corridor.solve(X);

        ipddp.solve();
    }

    clock_t finish = clock();
    double duration = (double)(finish - start) / CLOCKS_PER_SEC;
    std::cout << "\nIn Total : " << duration << " Seconds" << std::endl;

    return 0;
}