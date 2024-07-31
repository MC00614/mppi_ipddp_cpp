#include "wmrobot.h"

#include "collision_checker.h"
#include "mppi_ipddp.h"
#include "show.h"

#include "mppi.h"
#include "mppi.h"
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
        // Log_MPPI
        MPPI mppi(model);
        MPPIParam mppi_param;
        mppi_param.Nu = 5000;
        mppi_param.gamma_u = 10.0;
        Eigen::VectorXd log_mppi_sigma_u(model.dim_u);
        log_mppi_sigma_u << 0.5, 0.5;
        mppi_param.sigma_u = log_mppi_sigma_u.asDiagonal();
        mppi.init(mppi_param);
        mppi.setCollisionChecker(&collision_checker);
        
        double mppi_duration = 0.0;
        int i;
        for (i = 0; i < max_iter; ++i) {
            // MPPI
            auto start = std::chrono::high_resolution_clock::now();
            mppi.solve();
            auto finish = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = finish - start;
            mppi_duration += elapsed.count();

            res_X = mppi.getResX();
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
        total_duration += mppi_duration;
        std::cout << "\nMPPI : " << mppi_duration << " Seconds / " << i << " Iteration" << std::endl;
        std::cout << "Final Error : " << (final_state - res_X.col(model.N)).norm() << std::endl;
    }
    std::cout << "Total : " << total_duration << " Seconds" << std::endl;


    return 0;
}