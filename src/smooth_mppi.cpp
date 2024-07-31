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

    double total_msc_x = 0.0;
    double total_msc_u = 0.0;
    double total_tv_x = 0.0;
    double total_tv_u = 0.0;
    double msc_x = 0.0;
    double msc_u = 0.0;
    double tv_x = 0.0;
    double tv_u = 0.0;

    std::cout << "Smooth-MPPI (" << sim_maxiter << " simulations)" << std::endl;
    std::cout << "iter_duration\titer\tfinal_error\tfailed\tmsc_x\t\tmsc_u\t\ttv_x\t\ttv_u" << std::endl;
    
    for (int t = 0; t < sim_maxiter; ++t) {
        is_failed = false;
        // Smooth_MPPI
        SmoothMPPI smooth_mppi(model);
        MPPIParam smooth_mppi_param1;
        smooth_mppi_param1.Nu = 15000;
        smooth_mppi_param1.gamma_u = 10.0;
        Eigen::VectorXd sigma_u_1(model.dim_u);
        sigma_u_1 << 0.2, 0.2;
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
            res_U = smooth_mppi.getResU();
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
                break;
            }
            if (i + 1 == max_iter) {
                is_failed = true;
            }
        }
        if (!is_failed) {
            total_duration += smooth_mppi_duration;
            msc_x = meanSquaredCurvature(res_X);
            msc_u = meanSquaredCurvature(res_U);
            tv_x = totalVariation(res_X);
            tv_u = totalVariation(res_U);
        }
        else {fail++;}
        double fs_error = (final_state - res_X.col(model.N)).norm();
        total_msc_x += msc_x;
        total_msc_u += msc_u;
        total_tv_x += tv_x;
        total_tv_u += tv_u;
        std::cout << std::fixed << std::setprecision(6);
        std::cout.fill('0');
        std::cout.width(8);
        std::cout<<smooth_mppi_duration<<'\t'<<i<<'\t'<<fs_error<<'\t'<<(int)is_failed<<"\t";
        std::cout.fill('0');
        std::cout.width(8);
        std::cout<<msc_x<<'\t'<<msc_u<<'\t'<<tv_x<<'\t'<<tv_u<<std::endl;
    }
    std::cout << "" << std::endl;
    std::cout << "Success Rate : " << (int)(((sim_maxiter - fail)/(float)sim_maxiter)*100.0) << "% (Fail : " << fail << ")" << std::endl;
    std::cout << "Mean Squared Curvature X : " << total_msc_x << std::endl;
    std::cout << "Mean Squared Curvature U : " << total_msc_u << std::endl;
    std::cout << "Total Variation X : " << total_tv_x << std::endl;
    std::cout << "Total Variation U : " << total_tv_u << std::endl;
    std::cout << "Average : " << total_duration/(sim_maxiter-fail) << " Seconds (Total " << total_duration << ")" << std::endl;

    return 0;
}