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
        // Smooth_MPPI
        SmoothMPPI smooth_mppi(model);
        MPPIParam smooth_mppi_param1;
        smooth_mppi_param1.Nu = 30000;
        smooth_mppi_param1.gamma_u = 10.0;
        Eigen::VectorXd sigma_u_1(model.dim_u);
        sigma_u_1 << 0.5, 0.5;
        smooth_mppi_param1.sigma_u = sigma_u_1.asDiagonal();
        SmoothMPPIParam smooth_mppi_param2;
        smooth_mppi_param2.dt = 1.0;
        smooth_mppi_param2.lambda = 15.0;
        Eigen::VectorXd w(model.dim_u);
        w << 0.8, 0.8;
        smooth_mppi_param2.w = w.asDiagonal();

        smooth_mppi.init2(smooth_mppi_param1, smooth_mppi_param2);
        smooth_mppi.setCollisionChecker(&collision_checker);
        
        double smooth_mppi_duration = 0.0;
        int i;
        for (i = 0; i < max_iter; ++i) {
            // Smooth_MPPI
            auto start = std::chrono::high_resolution_clock::now();
            smooth_mppi.solve();
            auto finish = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = finish - start;
            smooth_mppi_duration += elapsed.count();

            res_X = smooth_mppi.getResX();
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
        total_duration += smooth_mppi_duration;
        std::cout << "\nSmooth-MPPI : " << smooth_mppi_duration << " Seconds / " << i << " Iteration" << std::endl;
        std::cout << "Final Error : " << (final_state - res_X.col(model.N)).norm() << std::endl;
    }
    std::cout << "Total : " << total_duration << " Seconds" << std::endl;

    return 0;
}