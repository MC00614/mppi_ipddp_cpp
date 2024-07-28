#include "wmrobot.h"

#include "collision_checker.h"
#include "mppi_ipddp.h"
#include "show.h"

int main() {
    // Model
    auto model = WMRobot();

    // MPPI Parameter
    MPPIParam mppi_param;
    mppi_param.Nu = 5000;
    mppi_param.gamma_u = 100.0;
    Eigen::VectorXd sigma_u(model.dim_u);
    // CHECK!
    sigma_u << 0.3, 0.7;
    mppi_param.sigma_u = sigma_u.asDiagonal();
    
    // Corridor Parameter
    CorridorParam corridor_param;
    corridor_param.max_iter = 20;
    corridor_param.Nz = 3000;
    corridor_param.gamma_z = 1000.0;
    Eigen::VectorXd sigma_z(model.center_point + 1);
    sigma_z << 0.3, 0.3, 0.08;
    corridor_param.sigma_z = sigma_z.asDiagonal();
    corridor_param.lambda_c = 20.0;
    corridor_param.lambda_r = 35.0;
    corridor_param.r_max = 0.5;

    // IPDDP Parameter
    Param ipddp_param;
    ipddp_param.tolerance = 1e-7;
    ipddp_param.max_iter = 500;
    ipddp_param.mu = 0.01;
    ipddp_param.infeasible = true;
    ipddp_param.q = 0.001;

    // Collision Checker
    CollisionChecker collision_checker;
    collision_checker.addRectangle(-2.5, 2.0, 3.0, 2.0);
    collision_checker.addRectangle(1.0, 2.0, 3.0, 2.0);
    collision_checker.addCircle(0.5, 1.0, 0.25);

    // MPPI_IPDDP
    MPPI_IPDDP mppi_ipddp(model);
    mppi_ipddp.init(mppi_param, corridor_param, ipddp_param);
    mppi_ipddp.setCollisionChecker(&collision_checker);

    // clock_t start = clock();

    // mppi_ipddp.solve(0);
    for (int t = 0; t < 1000; ++t) {
        mppi_ipddp.solve(1);

        show2D(mppi_ipddp.mppi_X, mppi_ipddp.mppi_U, mppi_ipddp.X, mppi_ipddp.U, mppi_ipddp.C, mppi_ipddp.R, collision_checker.circles, collision_checker.rectangles);

        mppi_ipddp.move();
    }

    // clock_t finish = clock();
    // double duration = (double)(finish - start) / CLOCKS_PER_SEC;
    // std::cout << "\nIn Total : " << duration << " Seconds" << std::endl;


    return 0;
}