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
        // MPPI Parameter
        MPPIParam mppi_param;
        mppi_param.Nu = 500;
        mppi_param.gamma_u = 100.0;
        Eigen::VectorXd sigma_u(model.dim_u);
        sigma_u << 0.5, 0.5;
        mppi_param.sigma_u = sigma_u.asDiagonal();
        
        // Corridor Parameter
        CorridorParam corridor_param;
        corridor_param.max_iter = 3;
        corridor_param.Nz = 1000;
        corridor_param.gamma_z = 1000.0;
        Eigen::VectorXd sigma_z(model.center_point + 1);
        sigma_z << 0.3, 0.3, 0.1;
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
            res_U = mppi_ipddp.U;
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
            mppi_ipddp_duration = mppi_ipddp_duration1 + mppi_ipddp_duration2 + mppi_ipddp_duration3;
            total_duration += mppi_ipddp_duration;
            mean_squared_curvature_x += meanSquaredCurvature(res_X);
            mean_squared_curvature_u += meanSquaredCurvature(res_U);
            total_variation_x += totalVariation(res_X);
            total_variation_u += totalVariation(res_U);
        }
        else {fail++;}
        
        std::cout << "\nMPPI-IPDDP : " << mppi_ipddp_duration << " Seconds / " << i << " Iteration" << std::endl;
        std::cout << mppi_ipddp_duration1 << '\t' << mppi_ipddp_duration2 << '\t' << mppi_ipddp_duration3 << std::endl;
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