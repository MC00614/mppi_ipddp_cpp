#include "wmrobot.h"

#include "collision_checker.h"
#include "mppi_ipddp.h"
#include "show.h"
#include "parameter.h"

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
    Eigen::MatrixXd res_X, res_U;
    int max_iter = 100;
    double total_duration = 0.0;
    bool is_failed;
    int fail = 0;
    int sim_maxiter = 100;

    double mean_squared_curvature_x = 0.0;
    double mean_squared_curvature_u = 0.0;
    double total_variation_x = 0.0;
    double total_variation_u = 0.0;

    for (int t = 0; t < sim_maxiter; ++t) {
        is_failed = false;
        // Log_MPPI
        LogMPPI log_mppi(model);
        MPPIParam log_mppi_param1;
        log_mppi_param1.Nu = 5000;
        log_mppi_param1.gamma_u = 100.0;
        Eigen::VectorXd log_mppi_sigma_u(model.dim_u);
        log_mppi_sigma_u << 0.3, 0.3;
        log_mppi_param1.sigma_u = log_mppi_sigma_u.asDiagonal();
        log_mppi.init(log_mppi_param1);
        log_mppi.setCollisionChecker(&collision_checker);
        
        double log_mppi_duration = 0.0;
        int i;
        for (i = 0; i < max_iter; ++i) {
            // Log_MPPI
            auto start = std::chrono::high_resolution_clock::now();
            log_mppi.solve();
            auto finish = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = finish - start;
            log_mppi_duration += elapsed.count();

            res_X = log_mppi.getResX();
            res_U = log_mppi.getResU();
            if ((final_state - res_X.col(model.N)).norm() < 0.1) {
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
            }
            if (i + 1 == max_iter) {
                is_failed = true;
                std::cout<<"FAILED"<<std::endl;
            }
        }
        if (!is_failed) {
            total_duration += log_mppi_duration;
            mean_squared_curvature_x += meanSquaredCurvature(res_X);
            mean_squared_curvature_u += meanSquaredCurvature(res_U);
            total_variation_x += totalVariation(res_X);
            total_variation_u += totalVariation(res_U);
        }
        else {fail++;}

        std::cout << "\nLOG-MPPI : " << log_mppi_duration << " Seconds / " << i << " Iteration" << std::endl;
        std::cout << "Final Error : " << (final_state - res_X.col(model.N)).norm() << std::endl;
    }
    std::cout << "" << std::endl;
    std::cout << "Total " << sim_maxiter << "Simulation" << std::endl;
    std::cout << "Success Rate : " << ((sim_maxiter - fail)/(float)sim_maxiter)*100.0 << "% (Fail : " << fail << ")" << std::endl;
    std::cout << "Mean Squared Curvature X : " << mean_squared_curvature_x << std::endl;
    std::cout << "Mean Squared Curvature U : " << mean_squared_curvature_u << std::endl;
    std::cout << "Total Variation X : " << total_variation_x << std::endl;
    std::cout << "Total Variation U : " << total_variation_u << std::endl;
    std::cout << "Average : " << total_duration/(sim_maxiter-fail) << " Seconds ( Total " << total_duration << ")" << std::endl;

    return 0;
}