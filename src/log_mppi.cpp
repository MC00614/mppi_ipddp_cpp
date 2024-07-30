#include "wmrobot.h"

#include "collision_checker.h"
#include "mppi_ipddp.h"
#include "show.h"

#include "log_mppi.h"

int main() {
    // Model
    auto model = WMRobot();

    // MPPI Parameter
    MPPIParam mppi_param;
    mppi_param.Nu = 5000;
    mppi_param.gamma_u = 100.0;
    Eigen::VectorXd sigma_u(model.dim_u);
    // CHECK!
    sigma_u << 0.5, 0.5;
    mppi_param.sigma_u = sigma_u.asDiagonal();
    
    // Corridor Parameter
    CorridorParam corridor_param;
    corridor_param.max_iter = 20;
    corridor_param.Nz = 3000;
    corridor_param.gamma_z = 1000.0;
    Eigen::VectorXd sigma_z(model.center_point + 1);
    sigma_z << 0.3, 0.3, 0.08;
    corridor_param.sigma_z = sigma_z.asDiagonal();
    corridor_param.lambda_c = 40.0;
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

    // Log_MPPI
    LogMPPI log_mppi(model);
    log_mppi.init(mppi_param);
    log_mppi.setCollisionChecker(&collision_checker);

    clock_t start;
    clock_t finish;
    double mppi_ipddp_duration = 0.0;
    double log_mppi_duration = 0.0;

    for (int t = 0; t < 1000; ++t) {
        // MPPI_IPDDP
        mppi_ipddp.solve(1);
        mppi_ipddp_duration = mppi_ipddp.mppi_duration + mppi_ipddp.corridor_duration + mppi_ipddp.ipddp_duration;
        
        // Log_MPPI
        start = clock();
        log_mppi.solve();
        finish = clock();
        log_mppi_duration = (double)(finish - start) / CLOCKS_PER_SEC;

        std::cout << "Iteration : " << t << std::endl;
        std::cout << "MPPI-IPDDP : " << mppi_ipddp_duration << " Seconds" << std::endl;
        std::cout << "LOG-MPPI : " << log_mppi_duration << " Seconds" << std::endl;
        std::cout << "" << std::endl;

        show2D(log_mppi.getResX(), log_mppi.getResU(), mppi_ipddp.X, mppi_ipddp.U, mppi_ipddp.C, mppi_ipddp.R, collision_checker.circles, collision_checker.rectangles);
    }

    return 0;
}