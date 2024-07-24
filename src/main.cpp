#include "invpend.h"
#include "cartpole.h"

#include "mppi_ipddp.h"

int main() {
    // Model
    // auto model = InvPend();
    auto model = CartPole();

    clock_t start = clock();

    // MPPI Parameter
    MPPIParam mppi_param;
    mppi_param.Nu = 100;
    mppi_param.gamma_u = 1.0;
    mppi_param.sigma_u = 1.0;
    
    // Corridor Parameter
    CorridorParam corridor_param;
    corridor_param.Nz = 3000;
    corridor_param.gamma_z = 1000.0;
    corridor_param.sigma_z = 0.3;
    corridor_param.lambda_c = 0.3;
    corridor_param.lambda_r = 0.3;
    corridor_param.center_index = {0,1};

    // IPDDP Parameter
    Param ipddp_param;
    ipddp_param.tolerance = 1e-7;
    ipddp_param.max_iter = 100;
    ipddp_param.mu = 0;

    // Collision Checker
    CollisionChecker *collision_checker;

    // MPPI_IPDDP
    MPPI_IPDDP mppi_ipddp(model);
    mppi_ipddp.init(mppi_param, corridor_param, ipddp_param);
    mppi_ipddp.setCollisionChecker(collision_checker);

    mppi_ipddp.solve();

    clock_t finish = clock();
    double duration = (double)(finish - start) / CLOCKS_PER_SEC;
    std::cout << "\nIn Total : " << duration << " Seconds" << std::endl;

    return 0;
}