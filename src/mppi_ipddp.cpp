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

    std::cout << "MPPI-IPDDP (" << sim_maxiter << " simulations)" << std::endl;
    std::cout << "iter_duration\titer\tfinal_error\tfailed\tmsc_x\t\tmsc_u\t\ttv_x\t\ttv_u" << std::endl;
    
    for (int t = 0; t < sim_maxiter; ++t) {
        is_failed = false;
        // MPPI Parameter
        MPPIParam mppi_param;
        mppi_param.Nu = 300;
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
            }
        }
        if (!is_failed) {
            mppi_ipddp_duration = mppi_ipddp_duration1 + mppi_ipddp_duration2 + mppi_ipddp_duration3;
            total_duration += mppi_ipddp_duration;
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
        std::cout<<mppi_ipddp_duration<<'\t'<<i<<'\t'<<fs_error<<'\t'<<(int)is_failed<<"\t";
        std::cout.fill('0');
        std::cout.width(8);
        std::cout<<msc_x<<'\t'<<msc_u<<'\t'<<tv_x<<'\t'<<tv_u<<std::endl;
    }
    std::cout << "" << std::endl;
    std::cout << "Success Rate : " << (int)(((sim_maxiter - fail)/(float)sim_maxiter)*100.0) << "% (Fail : " << fail << "/" << sim_maxiter << ")" << std::endl;
    std::cout << "Mean Squared Curvature X : " << total_msc_x << std::endl;
    std::cout << "Mean Squared Curvature U : " << total_msc_u << std::endl;
    std::cout << "Total Variation X : " << total_tv_x << std::endl;
    std::cout << "Total Variation U : " << total_tv_u << std::endl;
    std::cout << "Average : " << total_duration/(sim_maxiter-fail) << " Seconds (Total " << total_duration << ")" << std::endl;

    return 0;
}