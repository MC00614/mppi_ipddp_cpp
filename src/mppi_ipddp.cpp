#include "wmrobot.h"

#include "collision_checker.h"
#include "mppi_ipddp.h"
#include "show.h"

#include "log_mppi.h"
#include "smooth_mppi.h"

#include <chrono>

int main() {
    // Model
    auto model = WMRobot();

    // Collision Checker
    CollisionChecker collision_checker;
    collision_checker.addRectangle(-2.5, 2.0, 3.0, 2.0);
    collision_checker.addRectangle(1.0, 2.0, 3.0, 2.0);
    collision_checker.addCircle(0.5, 1.0, 0.25);

    clock_t start;
    clock_t finish;

    Eigen::VectorXd final_state(model.dim_x);
    final_state << 0.0, 6.0, M_PI_2;
    Eigen::MatrixXd res_X;
    int max_iter = 100;
    double total_duration = 0.0;

    for (int t = 0; t < 10; ++t) {
        // MPPI Parameter
        MPPIParam mppi_param;
        mppi_param.Nu = 300;
        mppi_param.gamma_u = 100.0;
        Eigen::VectorXd sigma_u(model.dim_u);
        sigma_u << 0.5, 0.5;
        mppi_param.sigma_u = sigma_u.asDiagonal();
        
        // Corridor Parameter
        CorridorParam corridor_param;
        corridor_param.max_iter = 5;
        corridor_param.Nz = 200;
        corridor_param.gamma_z = 1000.0;
        Eigen::VectorXd sigma_z(model.center_point + 1);
        sigma_z << 0.3, 0.3, 0.08;
        corridor_param.sigma_z = sigma_z.asDiagonal();
        corridor_param.lambda_c = 35.0;
        corridor_param.lambda_r = 35.0;
        corridor_param.r_max = 0.5;

        // IPDDP Parameter
        Param ipddp_param;
        ipddp_param.tolerance = 1e-7;
        ipddp_param.max_iter = 100;
        ipddp_param.mu = 0.01;
        ipddp_param.infeasible = true;
        ipddp_param.q = 0.001;

        // MPPI_IPDDP
        MPPI_IPDDP mppi_ipddp(model);
        mppi_ipddp.init(mppi_param, corridor_param, ipddp_param);
        mppi_ipddp.setCollisionChecker(&collision_checker);

        double mppi_ipddp_duration = 0.0;
        double mppi_ipddp_duration1 = 0.0;
        double mppi_ipddp_duration2 = 0.0;
        double mppi_ipddp_duration3 = 0.0;
        int i;
        for (i = 0; i < max_iter; ++i) {
            // MPPI_IPDDP
            mppi_ipddp.solve(1);
            mppi_ipddp_duration1 += mppi_ipddp.mppi_duration;
            mppi_ipddp_duration2 += mppi_ipddp.corridor_duration;
            mppi_ipddp_duration3 += mppi_ipddp.ipddp_duration;

            res_X = mppi_ipddp.X;
            if ((final_state - res_X.col(model.N)).norm() < 0.15) {
                bool is_collision = false;
                for (int j = 0; j < model.N; ++j) {
                    if (collision_checker.getCollisionGrid(res_X.col(j))) {
                        is_collision = true;
                        break;
                    }
                }
                if (!is_collision) {
                    break;
                }
                break;
            }
            if (i + 1 == max_iter) {std::cout << "FAIL" << std::endl;}
            
        }
        show2D(mppi_ipddp.X, mppi_ipddp.U, mppi_ipddp.X, mppi_ipddp.U, mppi_ipddp.C, mppi_ipddp.R, collision_checker.circles, collision_checker.rectangles);
        mppi_ipddp_duration = mppi_ipddp_duration1 + mppi_ipddp_duration2 + mppi_ipddp_duration3;
        total_duration += mppi_ipddp_duration;
        std::cout << "\nMPPI-IPDDP : " << mppi_ipddp_duration << " Seconds / " << i << " Iteration" << std::endl;
        std::cout << mppi_ipddp_duration1 << '\t' << mppi_ipddp_duration2 << '\t' << mppi_ipddp_duration3 << std::endl;

        std::cout << "Final Error : " << (final_state - res_X.col(model.N)).norm() << std::endl;
    }
    std::cout << "Total : " << total_duration << " Seconds" << std::endl;


    return 0;
}